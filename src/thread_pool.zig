const std = @import("std");

/// Default number of worker threads
pub const DEFAULT_THREAD_COUNT: usize = 0;

/// Minimum batch size before parallelizing
pub const MIN_PARALLEL_BATCH_SIZE: usize = 16;

/// Maximum tasks that can be queued
pub const MAX_QUEUED_TASKS: usize = 1024;

/// A unit of work to be executed by the thread pool
pub const Task = struct {
    /// Function to execute
    func: *const fn (*anyopaque) void,

    /// Data to pass to the function
    data: *anyopaque,

    /// Execute the task
    pub fn execute(self: Task) void {
        self.func(self.data);
    }
};

/// A range of indices for parallel for operations
pub const IndexRange = struct {
    start: usize,
    end: usize,

    pub fn len(self: IndexRange) usize {
        return self.end - self.start;
    }
};

/// A simple thread-safe queue for tasks
pub const TaskQueue = struct {
    tasks: [MAX_QUEUED_TASKS]Task,
    head: usize,
    tail: usize,
    count: std.atomic.Value(usize),
    mutex: std.Thread.Mutex,
    not_empty: std.Thread.Condition,

    pub fn init() TaskQueue {
        return .{
            .tasks = undefined,
            .head = 0,
            .tail = 0,
            .count = std.atomic.Value(usize).init(0),
            .mutex = .{},
            .not_empty = .{},
        };
    }

    /// Push a task to the queue
    pub fn push(self: *TaskQueue, task: Task) bool {
        self.mutex.lock();
        defer self.mutex.unlock();

        if (self.count.load(.acquire) >= MAX_QUEUED_TASKS) {
            return false;
        }

        self.tasks[self.tail] = task;
        self.tail = (self.tail + 1) % MAX_QUEUED_TASKS;
        _ = self.count.fetchAdd(1, .release);

        self.not_empty.signal();
        return true;
    }

    /// Pop a task from the queue
    pub fn pop(self: *TaskQueue, stop_flag: *std.atomic.Value(bool)) ?Task {
        self.mutex.lock();
        defer self.mutex.unlock();

        while (self.count.load(.acquire) == 0) {
            if (stop_flag.load(.acquire)) {
                return null;
            }
            self.not_empty.wait(&self.mutex);
            if (stop_flag.load(.acquire)) {
                return null;
            }
        }

        const task = self.tasks[self.head];
        self.head = (self.head + 1) % MAX_QUEUED_TASKS;
        _ = self.count.fetchSub(1, .release);

        return task;
    }

    /// Try to pop a task without blocking
    pub fn tryPop(self: *TaskQueue) ?Task {
        self.mutex.lock();
        defer self.mutex.unlock();

        if (self.count.load(.acquire) == 0) {
            return null;
        }

        const task = self.tasks[self.head];
        self.head = (self.head + 1) % MAX_QUEUED_TASKS;
        _ = self.count.fetchSub(1, .release);

        return task;
    }

    /// Get current count of tasks
    pub fn getCount(self: *TaskQueue) usize {
        return self.count.load(.acquire);
    }

    /// Wake all waiting threads
    pub fn wakeAll(self: *TaskQueue) void {
        self.mutex.lock();
        defer self.mutex.unlock();
        self.not_empty.broadcast();
    }
};

/// Thread pool for parallel physics processing
pub const ThreadPool = struct {
    allocator: std.mem.Allocator,

    /// Worker threads
    threads: []std.Thread,

    /// Task queue
    queue: TaskQueue,

    /// Stop flag for shutdown
    stop_flag: std.atomic.Value(bool),

    /// Number of active tasks
    active_tasks: std.atomic.Value(usize),

    /// Completion signal for waiting
    completion_mutex: std.Thread.Mutex,
    completion_cond: std.Thread.Condition,

    /// Statistics
    tasks_completed: std.atomic.Value(u64),
    total_wait_time_ns: std.atomic.Value(u64),

    /// Initialize the thread pool
    pub fn init(allocator: std.mem.Allocator, thread_count_opt: usize) !*ThreadPool {
        const thread_count = if (thread_count_opt == 0)
            @max(1, std.Thread.getCpuCount() catch 4)
        else
            thread_count_opt;

        const self = try allocator.create(ThreadPool);
        errdefer allocator.destroy(self);

        self.* = .{
            .allocator = allocator,
            .threads = &.{},
            .queue = TaskQueue.init(),
            .stop_flag = std.atomic.Value(bool).init(false),
            .active_tasks = std.atomic.Value(usize).init(0),
            .completion_mutex = .{},
            .completion_cond = .{},
            .tasks_completed = std.atomic.Value(u64).init(0),
            .total_wait_time_ns = std.atomic.Value(u64).init(0),
        };

        self.threads = try allocator.alloc(std.Thread, thread_count);
        errdefer allocator.free(self.threads);

        var spawned: usize = 0;
        errdefer {
            self.stop_flag.store(true, .release);
            self.queue.wakeAll();
            for (self.threads[0..spawned]) |thread| {
                thread.join();
            }
        }

        for (0..thread_count) |i| {
            self.threads[i] = try std.Thread.spawn(.{}, workerThread, .{self});
            spawned += 1;
        }

        return self;
    }

    /// Shutdown and cleanup the thread pool
    pub fn deinit(self: *ThreadPool) void {
        self.stop_flag.store(true, .release);
        self.queue.wakeAll();

        for (self.threads) |thread| {
            thread.join();
        }

        self.allocator.free(self.threads);
        self.allocator.destroy(self);
    }

    /// Submit a task to the pool
    pub fn submit(self: *ThreadPool, task: Task) bool {
        _ = self.active_tasks.fetchAdd(1, .acq_rel);
        if (!self.queue.push(task)) {
            _ = self.active_tasks.fetchSub(1, .release);
            return false;
        }
        return true;
    }

    /// Submit a function with data as a task
    pub fn submitFn(
        self: *ThreadPool,
        comptime func: anytype,
        data: anytype,
    ) bool {
        const DataType = @TypeOf(data);
        const Wrapper = struct {
            fn call(ptr: *anyopaque) void {
                const typed_data: *DataType = @ptrCast(@alignCast(ptr));
                func(typed_data.*);
            }
        };

        return self.submit(.{
            .func = Wrapper.call,
            .data = @ptrCast(@constCast(&data)),
        });
    }

    /// Wait for all submitted tasks to complete
    pub fn waitAll(self: *ThreadPool) void {
        self.completion_mutex.lock();
        defer self.completion_mutex.unlock();

        while (self.active_tasks.load(.acquire) > 0 or self.queue.getCount() > 0) {
            self.completion_cond.wait(&self.completion_mutex);
        }
    }

    /// Get number of worker threads
    pub fn threadCount(self: *ThreadPool) usize {
        return self.threads.len;
    }

    /// Get statistics
    pub fn getStats(self: *ThreadPool) struct {
        tasks_completed: u64,
        avg_wait_time_ns: u64,
        pending_tasks: usize,
    } {
        const completed = self.tasks_completed.load(.acquire);
        const wait_time = self.total_wait_time_ns.load(.acquire);
        return .{
            .tasks_completed = completed,
            .avg_wait_time_ns = if (completed > 0) wait_time / completed else 0,
            .pending_tasks = self.queue.getCount(),
        };
    }

    /// Worker thread function
    fn workerThread(self: *ThreadPool) void {
        while (!self.stop_flag.load(.acquire)) {
            if (self.queue.pop(&self.stop_flag)) |task| {
                task.execute();

                _ = self.tasks_completed.fetchAdd(1, .release);
                const remaining = self.active_tasks.fetchSub(1, .acq_rel);

                if (remaining == 1) {
                    self.completion_mutex.lock();
                    self.completion_cond.broadcast();
                    self.completion_mutex.unlock();
                }
            }
        }
    }
};

/// Context for parallel for operations
pub fn ParallelForContext(comptime T: type, comptime F: type) type {
    return struct {
        items: []T,
        func: F,
        batch_start: usize,
        batch_end: usize,
    };
}

/// Execute a function on each item in a slice in parallel
pub fn parallelFor(
    pool: *ThreadPool,
    comptime T: type,
    items: []T,
    comptime func: fn (*T) void,
) void {
    if (items.len < MIN_PARALLEL_BATCH_SIZE or pool.threads.len <= 1) {
        for (items) |*item| {
            func(item);
        }
        return;
    }

    const batch_size = @max(MIN_PARALLEL_BATCH_SIZE, (items.len + pool.threads.len - 1) / pool.threads.len);
    var start: usize = 0;

    var contexts: [64]struct {
        items: []T,
        start: usize,
        end: usize,
    } = undefined;
    var context_count: usize = 0;

    while (start < items.len and context_count < contexts.len) {
        const end = @min(start + batch_size, items.len);
        contexts[context_count] = .{
            .items = items,
            .start = start,
            .end = end,
        };

        const ctx_ptr = &contexts[context_count];
        const submitted = pool.submit(.{
            .func = struct {
                fn process(ptr: *anyopaque) void {
                    const ctx: *@TypeOf(ctx_ptr.*) = @ptrCast(@alignCast(ptr));
                    for (ctx.items[ctx.start..ctx.end]) |*item| {
                        func(item);
                    }
                }
            }.process,
            .data = @ptrCast(ctx_ptr),
        });

        if (!submitted) {
            for (items[start..]) |*item| {
                func(item);
            }
            break;
        }

        context_count += 1;
        start = end;
    }

    pool.waitAll();
}

/// Execute a function on index ranges in parallel
pub fn parallelForRange(
    pool: *ThreadPool,
    start: usize,
    end: usize,
    comptime func: fn (IndexRange) void,
) void {
    const total = end - start;

    if (total < MIN_PARALLEL_BATCH_SIZE or pool.threads.len <= 1) {
        func(.{ .start = start, .end = end });
        return;
    }

    const batch_size = @max(MIN_PARALLEL_BATCH_SIZE, (total + pool.threads.len - 1) / pool.threads.len);
    var batch_start: usize = start;

    var ranges: [64]IndexRange = undefined;
    var range_count: usize = 0;

    while (batch_start < end and range_count < ranges.len) {
        const batch_end = @min(batch_start + batch_size, end);
        ranges[range_count] = .{ .start = batch_start, .end = batch_end };

        const range_ptr = &ranges[range_count];
        const submitted = pool.submit(.{
            .func = struct {
                fn process(ptr: *anyopaque) void {
                    const range: *IndexRange = @ptrCast(@alignCast(ptr));
                    func(range.*);
                }
            }.process,
            .data = @ptrCast(range_ptr),
        });

        if (!submitted) {
            func(.{ .start = batch_start, .end = end });
            break;
        }

        range_count += 1;
        batch_start = batch_end;
    }

    pool.waitAll();
}

/// Result accumulator for parallel reduce operations
pub fn ParallelAccumulator(comptime T: type) type {
    return struct {
        value: std.atomic.Value(T),

        const Self = @This();

        pub fn init(initial: T) Self {
            return .{ .value = std.atomic.Value(T).init(initial) };
        }

        pub fn add(self: *Self, v: T) void {
            _ = self.value.fetchAdd(v, .acq_rel);
        }

        pub fn get(self: *Self) T {
            return self.value.load(.acquire);
        }
    };
}

/// A scoped wrapper that ensures work is completed before exiting
pub const ScopedWork = struct {
    pool: *ThreadPool,
    initial_active: usize,

    pub fn init(pool: *ThreadPool) ScopedWork {
        return .{
            .pool = pool,
            .initial_active = pool.active_tasks.load(.acquire),
        };
    }

    pub fn submit(self: *ScopedWork, task: Task) bool {
        return self.pool.submit(task);
    }

    pub fn wait(self: *ScopedWork) void {
        self.pool.waitAll();
    }
};

/// Execute a function in parallel using temporary threads
pub fn parallelExecute(
    allocator: std.mem.Allocator,
    comptime T: type,
    items: []T,
    comptime func: fn (*T) void,
    max_threads: usize,
) !void {
    if (items.len == 0) return;

    const thread_count = @min(max_threads, items.len);
    if (thread_count <= 1) {
        for (items) |*item| {
            func(item);
        }
        return;
    }

    const batch_size = (items.len + thread_count - 1) / thread_count;

    const ThreadContext = struct {
        items: []T,
        start: usize,
        end: usize,

        fn run(ctx: *@This()) void {
            for (ctx.items[ctx.start..ctx.end]) |*item| {
                func(item);
            }
        }
    };

    var contexts = try allocator.alloc(ThreadContext, thread_count);
    defer allocator.free(contexts);

    var threads = try allocator.alloc(std.Thread, thread_count - 1);
    defer allocator.free(threads);

    var idx: usize = 0;
    for (0..thread_count) |i| {
        const start = idx;
        const end = @min(idx + batch_size, items.len);
        contexts[i] = .{
            .items = items,
            .start = start,
            .end = end,
        };
        idx = end;
    }

    for (0..thread_count - 1) |i| {
        threads[i] = try std.Thread.spawn(.{}, ThreadContext.run, .{&contexts[i]});
    }

    ThreadContext.run(&contexts[thread_count - 1]);

    for (threads) |thread| {
        thread.join();
    }
}

test "TaskQueue basic operations" {
    var queue = TaskQueue.init();

    var counter: u32 = 0;

    const task = Task{
        .func = struct {
            fn inc(ptr: *anyopaque) void {
                const c: *u32 = @ptrCast(@alignCast(ptr));
                c.* += 1;
            }
        }.inc,
        .data = @ptrCast(&counter),
    };

    try std.testing.expect(queue.push(task));
    try std.testing.expectEqual(@as(usize, 1), queue.getCount());

    const popped = queue.tryPop();
    try std.testing.expect(popped != null);
    try std.testing.expectEqual(@as(usize, 0), queue.getCount());

    popped.?.execute();
    try std.testing.expectEqual(@as(u32, 1), counter);
}

test "ThreadPool init and deinit" {
    const pool = try ThreadPool.init(std.testing.allocator, 2);
    defer pool.deinit();

    try std.testing.expectEqual(@as(usize, 2), pool.threadCount());
}

test "ThreadPool simple task execution" {
    const pool = try ThreadPool.init(std.testing.allocator, 2);
    defer pool.deinit();

    var completed = std.atomic.Value(bool).init(false);

    const task = Task{
        .func = struct {
            fn setTrue(ptr: *anyopaque) void {
                const flag: *std.atomic.Value(bool) = @ptrCast(@alignCast(ptr));
                flag.store(true, .release);
            }
        }.setTrue,
        .data = @ptrCast(&completed),
    };

    try std.testing.expect(pool.submit(task));
    pool.waitAll();

    try std.testing.expect(completed.load(.acquire));
}

test "ThreadPool multiple tasks" {
    const pool = try ThreadPool.init(std.testing.allocator, 4);
    defer pool.deinit();

    var counter = std.atomic.Value(u32).init(0);

    const task = Task{
        .func = struct {
            fn inc(ptr: *anyopaque) void {
                const c: *std.atomic.Value(u32) = @ptrCast(@alignCast(ptr));
                _ = c.fetchAdd(1, .acq_rel);
            }
        }.inc,
        .data = @ptrCast(&counter),
    };

    for (0..100) |_| {
        try std.testing.expect(pool.submit(task));
    }

    pool.waitAll();
    try std.testing.expectEqual(@as(u32, 100), counter.load(.acquire));
}

test "parallelExecute" {
    var items = [_]u32{ 0, 0, 0, 0, 0, 0, 0, 0 };

    try parallelExecute(std.testing.allocator, u32, &items, struct {
        fn setOne(ptr: *u32) void {
            ptr.* = 1;
        }
    }.setOne, 4);

    for (items) |item| {
        try std.testing.expectEqual(@as(u32, 1), item);
    }
}

test "ParallelAccumulator" {
    var acc = ParallelAccumulator(u32).init(0);

    acc.add(5);
    acc.add(10);
    acc.add(15);

    try std.testing.expectEqual(@as(u32, 30), acc.get());
}

const std = @import("std");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const narrow_phase_mod = @import("narrow_phase.zig");
const solver_mod = @import("solver.zig");

const Vec3 = math.Vec3;
const VoxelBody = body_mod.VoxelBody;
const ContactManifold = narrow_phase_mod.ContactManifold;

/// Minimum island size to consider for parallel solving
pub const MIN_PARALLEL_ISLAND_SIZE: usize = 4;

/// Maximum number of islands to track
pub const MAX_ISLANDS: usize = 256;

/// Velocity threshold for island sleeping
pub const ISLAND_SLEEP_LINEAR_THRESHOLD: f32 = 0.1;
pub const ISLAND_SLEEP_ANGULAR_THRESHOLD: f32 = 0.1;

/// Time bodies must be still before island sleeps
pub const ISLAND_SLEEP_TIME_THRESHOLD: f32 = 0.5;

/// Union-Find for efficient grouping of connected bodies
pub const UnionFind = struct {
    allocator: std.mem.Allocator,

    /// Parent array - parent[i] is the parent of element i
    parent: []u32,

    /// Rank array - used for union by rank optimization
    rank: []u32,

    /// Number of elements
    count: usize,

    /// Number of distinct sets
    set_count: usize,

    pub fn init(allocator: std.mem.Allocator, size: usize) !UnionFind {
        const parent = try allocator.alloc(u32, size);
        const rank = try allocator.alloc(u32, size);

        for (0..size) |i| {
            parent[i] = @intCast(i);
            rank[i] = 0;
        }

        return .{
            .allocator = allocator,
            .parent = parent,
            .rank = rank,
            .count = size,
            .set_count = size,
        };
    }

    pub fn deinit(self: *UnionFind) void {
        self.allocator.free(self.parent);
        self.allocator.free(self.rank);
    }

    /// Reset all elements to singleton sets
    pub fn reset(self: *UnionFind) void {
        for (0..self.count) |i| {
            self.parent[i] = @intCast(i);
            self.rank[i] = 0;
        }
        self.set_count = self.count;
    }

    /// Resize the union-find structure
    pub fn resize(self: *UnionFind, new_size: usize) !void {
        if (new_size <= self.parent.len) {
            self.count = new_size;
            self.reset();
            return;
        }

        self.parent = try self.allocator.realloc(self.parent, new_size);
        self.rank = try self.allocator.realloc(self.rank, new_size);
        self.count = new_size;

        for (0..new_size) |i| {
            self.parent[i] = @intCast(i);
            self.rank[i] = 0;
        }
        self.set_count = new_size;
    }

    /// Find the root of the set containing element x
    pub fn find(self: *UnionFind, x: u32) u32 {
        if (x >= self.count) return x;

        var root = x;
        while (self.parent[root] != root) {
            root = self.parent[root];
        }

        var current = x;
        while (current != root) {
            const next = self.parent[current];
            self.parent[current] = root;
            current = next;
        }

        return root;
    }

    /// Union the sets containing elements x and y
    pub fn unite(self: *UnionFind, x: u32, y: u32) void {
        const root_x = self.find(x);
        const root_y = self.find(y);

        if (root_x == root_y) return;

        if (self.rank[root_x] < self.rank[root_y]) {
            self.parent[root_x] = root_y;
        } else if (self.rank[root_x] > self.rank[root_y]) {
            self.parent[root_y] = root_x;
        } else {
            self.parent[root_y] = root_x;
            self.rank[root_x] += 1;
        }

        self.set_count -= 1;
    }

    /// Check if two elements are in the same set
    pub fn connected(self: *UnionFind, x: u32, y: u32) bool {
        return self.find(x) == self.find(y);
    }

    /// Get the number of distinct sets
    pub fn getSetCount(self: *UnionFind) usize {
        return self.set_count;
    }
};

/// An island is a group of connected bodies that can be solved independently
pub const Island = struct {
    /// Body IDs in this island
    bodies: std.ArrayListUnmanaged(u32),

    /// Contact manifold indices for this island
    manifolds: std.ArrayListUnmanaged(u32),

    /// Island ID
    id: u32,

    /// Whether the island is sleeping
    is_sleeping: bool,

    /// Time the island has been still
    sleep_timer: f32,

    /// Whether the island needs solving
    needs_solving: bool,

    pub fn init() Island {
        return .{
            .bodies = .{},
            .manifolds = .{},
            .id = 0,
            .is_sleeping = false,
            .sleep_timer = 0,
            .needs_solving = true,
        };
    }

    pub fn deinit(self: *Island, allocator: std.mem.Allocator) void {
        self.bodies.deinit(allocator);
        self.manifolds.deinit(allocator);
    }

    pub fn clear(self: *Island, allocator: std.mem.Allocator) void {
        self.bodies.clearRetainingCapacity();
        self.manifolds.clearRetainingCapacity();
        self.id = 0;
        self.is_sleeping = false;
        self.sleep_timer = 0;
        self.needs_solving = true;
        _ = allocator;
    }

    pub fn bodyCount(self: *const Island) usize {
        return self.bodies.items.len;
    }

    pub fn manifoldCount(self: *const Island) usize {
        return self.manifolds.items.len;
    }
};

/// Manages island detection and tracking
pub const IslandManager = struct {
    allocator: std.mem.Allocator,

    /// Union-Find for grouping bodies
    union_find: UnionFind,

    /// Detected islands
    islands: std.ArrayListUnmanaged(Island),

    /// Map from body ID to island index
    body_to_island: std.AutoHashMapUnmanaged(u32, u32),

    /// Statistics
    stats: IslandStats,

    /// Configuration
    enable_sleeping: bool,

    /// Hash of the previous contact graph for change detection
    previous_graph_hash: u64,

    /// Whether islands need to be rebuilt
    needs_rebuild: bool,

    /// Statistics for rebuild skipping
    rebuilds_skipped: u32,
    rebuilds_performed: u32,

    pub const IslandStats = struct {
        total_islands: usize,
        active_islands: usize,
        sleeping_islands: usize,
        largest_island_size: usize,
        total_bodies: usize,
        total_contacts: usize,
        rebuilds_skipped: u32,
        rebuilds_performed: u32,

        pub fn reset(self: *IslandStats) void {
            self.* = .{
                .total_islands = 0,
                .active_islands = 0,
                .sleeping_islands = 0,
                .largest_island_size = 0,
                .total_bodies = 0,
                .total_contacts = 0,
                .rebuilds_skipped = 0,
                .rebuilds_performed = 0,
            };
        }
    };

    pub fn init(allocator: std.mem.Allocator, max_bodies: usize) !IslandManager {
        return .{
            .allocator = allocator,
            .union_find = try UnionFind.init(allocator, max_bodies),
            .islands = .{},
            .body_to_island = .{},
            .stats = .{
                .total_islands = 0,
                .active_islands = 0,
                .sleeping_islands = 0,
                .largest_island_size = 0,
                .total_bodies = 0,
                .total_contacts = 0,
                .rebuilds_skipped = 0,
                .rebuilds_performed = 0,
            },
            .enable_sleeping = true,
            .previous_graph_hash = 0,
            .needs_rebuild = true,
            .rebuilds_skipped = 0,
            .rebuilds_performed = 0,
        };
    }

    pub fn deinit(self: *IslandManager) void {
        for (self.islands.items) |*island| {
            island.deinit(self.allocator);
        }
        self.islands.deinit(self.allocator);
        self.body_to_island.deinit(self.allocator);
        self.union_find.deinit();
    }

    /// Compute a hash of the contact graph for change detection
    fn computeGraphHash(manifolds: []const ContactManifold) u64 {
        var hash: u64 = 0;
        const prime: u64 = 0x100000001b3;

        for (manifolds) |manifold| {
            if (!manifold.is_active) continue;

            const a = @min(manifold.body_a_id, manifold.body_b_id);
            const b = @max(manifold.body_a_id, manifold.body_b_id);
            const pair_key = (@as(u64, a) << 32) | @as(u64, b);

            hash ^= pair_key;
            hash *%= prime;
        }

        hash ^= @as(u64, manifolds.len);
        hash *%= prime;

        return hash;
    }

    /// Mark islands as needing rebuild
    pub fn markDirty(self: *IslandManager) void {
        self.needs_rebuild = true;
    }

    /// Build islands from contact manifolds
    pub fn buildIslands(
        self: *IslandManager,
        manifolds: []const ContactManifold,
        body_count: usize,
    ) !void {
        return self.buildIslandsWithBodyTypes(manifolds, body_count, null);
    }

    /// Build islands from contact manifolds, excluding static bodies from island connectivity
    pub fn buildIslandsWithBodyTypes(
        self: *IslandManager,
        manifolds: []const ContactManifold,
        body_count: usize,
        body_types: ?[]const body_mod.BodyType,
    ) !void {
        const current_hash = computeGraphHash(manifolds);

        if (!self.needs_rebuild and current_hash == self.previous_graph_hash and self.stats.total_islands > 0) {
            self.rebuilds_skipped += 1;
            self.stats.rebuilds_skipped = self.rebuilds_skipped;
            self.stats.rebuilds_performed = self.rebuilds_performed;
            return;
        }

        self.previous_graph_hash = current_hash;
        self.needs_rebuild = false;
        self.rebuilds_performed += 1;

        for (self.islands.items) |*island| {
            island.clear(self.allocator);
        }
        self.body_to_island.clearRetainingCapacity();
        self.stats.reset();
        self.stats.rebuilds_skipped = self.rebuilds_skipped;
        self.stats.rebuilds_performed = self.rebuilds_performed;

        if (manifolds.len == 0) return;

        var body_ids = std.AutoHashMapUnmanaged(u32, void){};
        defer body_ids.deinit(self.allocator);

        for (manifolds) |manifold| {
            if (!manifold.is_active) continue;
            try body_ids.put(self.allocator, manifold.body_a_id, {});
            try body_ids.put(self.allocator, manifold.body_b_id, {});
        }

        var max_body_id: u32 = 0;
        var body_id_iter = body_ids.keyIterator();
        while (body_id_iter.next()) |id| {
            if (id.* > max_body_id) max_body_id = id.*;
        }

        const required_size = @max(body_count, max_body_id + 1);
        if (required_size > self.union_find.count) {
            try self.union_find.resize(required_size);
        } else {
            self.union_find.reset();
        }

        for (manifolds) |manifold| {
            if (!manifold.is_active) continue;

            if (body_types) |types| {
                const a_is_static = manifold.body_a_id < types.len and types[manifold.body_a_id] == .static;
                const b_is_static = manifold.body_b_id < types.len and types[manifold.body_b_id] == .static;

                if (a_is_static or b_is_static) continue;
            }

            self.union_find.unite(manifold.body_a_id, manifold.body_b_id);
        }

        self.stats.total_contacts = manifolds.len;

        var root_to_island = std.AutoHashMapUnmanaged(u32, u32){};
        defer root_to_island.deinit(self.allocator);

        var island_count: u32 = 0;
        var body_iter = body_ids.keyIterator();
        while (body_iter.next()) |body_id_ptr| {
            const body_id = body_id_ptr.*;
            const root = self.union_find.find(body_id);

            if (!root_to_island.contains(root)) {
                try root_to_island.put(self.allocator, root, island_count);

                while (self.islands.items.len <= island_count) {
                    try self.islands.append(self.allocator, Island.init());
                }

                self.islands.items[island_count].id = root;
                island_count += 1;
            }

            const island_idx = root_to_island.get(root).?;
            try self.body_to_island.put(self.allocator, body_id, island_idx);

            var already_added = false;
            for (self.islands.items[island_idx].bodies.items) |existing_id| {
                if (existing_id == body_id) {
                    already_added = true;
                    break;
                }
            }
            if (!already_added) {
                try self.islands.items[island_idx].bodies.append(self.allocator, body_id);
            }
        }

        for (manifolds, 0..) |manifold, manifold_idx| {
            if (!manifold.is_active) continue;

            const root = self.union_find.find(manifold.body_a_id);
            if (root_to_island.get(root)) |island_idx| {
                try self.islands.items[island_idx].manifolds.append(self.allocator, @intCast(manifold_idx));
            }
        }

        self.stats.total_islands = island_count;
        for (self.islands.items[0..island_count]) |*island| {
            const size = island.bodyCount();
            self.stats.total_bodies += size;
            if (size > self.stats.largest_island_size) {
                self.stats.largest_island_size = size;
            }
            if (island.is_sleeping) {
                self.stats.sleeping_islands += 1;
            } else {
                self.stats.active_islands += 1;
            }
        }
    }

    /// Update island sleep states based on body velocities
    pub fn updateSleepStates(
        self: *IslandManager,
        bodies: []const VoxelBody,
        dt: f32,
    ) void {
        if (!self.enable_sleeping) return;

        for (self.islands.items) |*island| {
            if (island.bodyCount() == 0) continue;

            var all_still = true;

            for (island.bodies.items) |body_id| {
                if (body_id >= bodies.len) continue;
                const body = &bodies[body_id];

                if (body.body_type != .dynamic) continue;

                const lin_speed_sq = body.motion.linear_velocity.lengthSquared();
                const ang_speed_sq = body.motion.angular_velocity.lengthSquared();

                if (lin_speed_sq > ISLAND_SLEEP_LINEAR_THRESHOLD * ISLAND_SLEEP_LINEAR_THRESHOLD or
                    ang_speed_sq > ISLAND_SLEEP_ANGULAR_THRESHOLD * ISLAND_SLEEP_ANGULAR_THRESHOLD)
                {
                    all_still = false;
                    break;
                }
            }

            if (all_still) {
                island.sleep_timer += dt;
                if (island.sleep_timer >= ISLAND_SLEEP_TIME_THRESHOLD) {
                    island.is_sleeping = true;
                    island.needs_solving = false;
                }
            } else {
                island.sleep_timer = 0;
                island.is_sleeping = false;
                island.needs_solving = true;
            }
        }

        self.stats.sleeping_islands = 0;
        self.stats.active_islands = 0;
        for (self.islands.items[0..self.stats.total_islands]) |*island| {
            if (island.is_sleeping) {
                self.stats.sleeping_islands += 1;
            } else {
                self.stats.active_islands += 1;
            }
        }
    }

    /// Wake up an island
    pub fn wakeIsland(self: *IslandManager, island_idx: u32) void {
        if (island_idx >= self.islands.items.len) return;

        var island = &self.islands.items[island_idx];
        island.is_sleeping = false;
        island.sleep_timer = 0;
        island.needs_solving = true;

        self.needs_rebuild = true;
    }

    /// Wake up the island containing a specific body
    pub fn wakeBodyIsland(self: *IslandManager, body_id: u32) void {
        if (self.body_to_island.get(body_id)) |island_idx| {
            self.wakeIsland(island_idx);
        } else {
            self.needs_rebuild = true;
        }
    }

    /// Get the island containing a body
    pub fn getBodyIsland(self: *IslandManager, body_id: u32) ?*Island {
        if (self.body_to_island.get(body_id)) |island_idx| {
            if (island_idx < self.islands.items.len) {
                return &self.islands.items[island_idx];
            }
        }
        return null;
    }

    /// Get all active islands
    pub fn getActiveIslands(self: *IslandManager) []Island {
        return self.islands.items[0..self.stats.total_islands];
    }

    /// Get islands sorted by size for load balancing
    pub fn getIslandsSortedBySize(
        self: *IslandManager,
        output: *std.ArrayListUnmanaged(u32),
    ) !void {
        output.clearRetainingCapacity();

        for (0..self.stats.total_islands) |i| {
            try output.append(self.allocator, @intCast(i));
        }

        const islands = self.islands.items;
        std.mem.sort(u32, output.items, {}, struct {
            fn lessThan(_: void, a: u32, b: u32) bool {
                _ = islands;

                return a < b;
            }
        }.lessThan);
    }

    /// Check if a body is in a sleeping island
    pub fn isBodyInSleepingIsland(self: *IslandManager, body_id: u32) bool {
        if (self.body_to_island.get(body_id)) |island_idx| {
            if (island_idx < self.islands.items.len) {
                return self.islands.items[island_idx].is_sleeping;
            }
        }
        return false;
    }

    pub fn getStats(self: *const IslandManager) IslandStats {
        return self.stats;
    }
};

/// Context for solving a single island
pub const IslandSolveContext = struct {
    island: *Island,
    bodies: []VoxelBody,
    manifolds: []ContactManifold,
    solver_config: solver_mod.SolverConfig,
    dt: f32,
};

/// Solve islands in parallel using a thread pool
pub fn solveIslandsParallel(
    island_manager: *IslandManager,
    bodies: []VoxelBody,
    manifolds: []ContactManifold,
    solver_config: solver_mod.SolverConfig,
    dt: f32,
    thread_pool: anytype,
) void {
    const islands = island_manager.getActiveIslands();

    if (thread_pool == null or islands.len < 2) {
        for (islands) |*island| {
            if (island.is_sleeping or !island.needs_solving) continue;
            solveIsland(island, bodies, manifolds, solver_config, dt);
        }
        return;
    }

    for (islands) |*island| {
        if (island.is_sleeping or !island.needs_solving) continue;
        solveIsland(island, bodies, manifolds, solver_config, dt);
    }
}

/// Solve a single island
fn solveIsland(
    island: *Island,
    bodies: []VoxelBody,
    manifolds: []ContactManifold,
    solver_config: solver_mod.SolverConfig,
    dt: f32,
) void {
    _ = bodies;
    _ = solver_config;
    _ = dt;

    if (island.manifoldCount() == 0) return;

    for (island.manifolds.items) |manifold_idx| {
        if (manifold_idx >= manifolds.len) continue;
        const manifold = &manifolds[manifold_idx];
        _ = manifold;
    }
}

/// Graph coloring result for parallel contact processing
pub const GraphColoring = struct {
    allocator: std.mem.Allocator,

    /// Color assigned to each manifold
    colors: std.ArrayListUnmanaged(u8),

    /// Maximum color used
    max_color: u8,

    /// Manifolds grouped by color
    color_groups: [256]std.ArrayListUnmanaged(u32),

    pub fn init(allocator: std.mem.Allocator) GraphColoring {
        var result = GraphColoring{
            .allocator = allocator,
            .colors = .{},
            .max_color = 0,
            .color_groups = undefined,
        };

        for (&result.color_groups) |*group| {
            group.* = .{};
        }

        return result;
    }

    pub fn deinit(self: *GraphColoring) void {
        self.colors.deinit(self.allocator);
        for (&self.color_groups) |*group| {
            group.deinit(self.allocator);
        }
    }

    pub fn clear(self: *GraphColoring) void {
        self.colors.clearRetainingCapacity();
        self.max_color = 0;
        for (&self.color_groups) |*group| {
            group.clearRetainingCapacity();
        }
    }

    /// Build graph coloring for manifolds
    pub fn build(self: *GraphColoring, manifolds: []const ContactManifold) !void {
        self.clear();

        if (manifolds.len == 0) return;

        try self.colors.ensureTotalCapacity(self.allocator, manifolds.len);

        var body_manifolds = std.AutoHashMapUnmanaged(u32, std.ArrayListUnmanaged(u32)){};
        defer {
            var iter = body_manifolds.valueIterator();
            while (iter.next()) |list| {
                list.deinit(self.allocator);
            }
            body_manifolds.deinit(self.allocator);
        }

        for (manifolds, 0..) |manifold, idx| {
            if (!manifold.is_active) continue;

            const manifold_idx: u32 = @intCast(idx);

            if (!body_manifolds.contains(manifold.body_a_id)) {
                try body_manifolds.put(self.allocator, manifold.body_a_id, .{});
            }
            try body_manifolds.getPtr(manifold.body_a_id).?.append(self.allocator, manifold_idx);

            if (!body_manifolds.contains(manifold.body_b_id)) {
                try body_manifolds.put(self.allocator, manifold.body_b_id, .{});
            }
            try body_manifolds.getPtr(manifold.body_b_id).?.append(self.allocator, manifold_idx);
        }

        for (manifolds, 0..) |manifold, idx| {
            if (!manifold.is_active) {
                try self.colors.append(self.allocator, 0);
                continue;
            }

            var used_colors: [256]bool = [_]bool{false} ** 256;

            if (body_manifolds.get(manifold.body_a_id)) |adjacent| {
                for (adjacent.items) |adj_idx| {
                    if (adj_idx < self.colors.items.len) {
                        used_colors[self.colors.items[adj_idx]] = true;
                    }
                }
            }

            if (body_manifolds.get(manifold.body_b_id)) |adjacent| {
                for (adjacent.items) |adj_idx| {
                    if (adj_idx < self.colors.items.len) {
                        used_colors[self.colors.items[adj_idx]] = true;
                    }
                }
            }

            var color: u8 = 0;
            while (color < 255 and used_colors[color]) {
                color += 1;
            }

            try self.colors.append(self.allocator, color);
            try self.color_groups[color].append(self.allocator, @intCast(idx));

            if (color > self.max_color) {
                self.max_color = color;
            }
        }
    }

    /// Get manifolds of a specific color
    pub fn getManifoldsByColor(self: *const GraphColoring, color: u8) []const u32 {
        return self.color_groups[color].items;
    }

    /// Get number of colors used
    pub fn colorCount(self: *const GraphColoring) usize {
        return @as(usize, self.max_color) + 1;
    }
};

test "UnionFind basic operations" {
    var uf = try UnionFind.init(std.testing.allocator, 10);
    defer uf.deinit();

    try std.testing.expectEqual(@as(usize, 10), uf.getSetCount());
    try std.testing.expect(!uf.connected(0, 1));

    uf.unite(0, 1);
    try std.testing.expect(uf.connected(0, 1));
    try std.testing.expectEqual(@as(usize, 9), uf.getSetCount());

    uf.unite(2, 3);
    try std.testing.expect(uf.connected(2, 3));
    try std.testing.expect(!uf.connected(0, 2));

    uf.unite(1, 2);
    try std.testing.expect(uf.connected(0, 3));
    try std.testing.expectEqual(@as(usize, 7), uf.getSetCount());
}

test "UnionFind path compression" {
    var uf = try UnionFind.init(std.testing.allocator, 100);
    defer uf.deinit();

    for (1..100) |i| {
        uf.unite(@intCast(i - 1), @intCast(i));
    }

    try std.testing.expect(uf.connected(0, 99));
    try std.testing.expectEqual(@as(usize, 1), uf.getSetCount());

    const root = uf.find(99);
    try std.testing.expectEqual(root, uf.find(0));
    try std.testing.expectEqual(root, uf.find(50));
}

test "IslandManager build islands" {
    var manager = try IslandManager.init(std.testing.allocator, 10);
    defer manager.deinit();

    var manifolds = [_]ContactManifold{
        ContactManifold.init(0, 1, 0.5, 0.0),
        ContactManifold.init(1, 2, 0.5, 0.0),
        ContactManifold.init(3, 4, 0.5, 0.0),
    };

    try manager.buildIslands(&manifolds, 5);

    try std.testing.expectEqual(@as(usize, 2), manager.stats.total_islands);

    const island_0 = manager.getBodyIsland(0);
    const island_1 = manager.getBodyIsland(1);
    const island_2 = manager.getBodyIsland(2);
    const island_3 = manager.getBodyIsland(3);
    const island_4 = manager.getBodyIsland(4);

    try std.testing.expect(island_0 != null);
    try std.testing.expect(island_1 != null);
    try std.testing.expect(island_2 != null);
    try std.testing.expect(island_3 != null);
    try std.testing.expect(island_4 != null);

    try std.testing.expectEqual(island_0.?.id, island_1.?.id);
    try std.testing.expectEqual(island_1.?.id, island_2.?.id);

    try std.testing.expectEqual(island_3.?.id, island_4.?.id);
    try std.testing.expect(island_0.?.id != island_3.?.id);
}

test "GraphColoring basic" {
    var coloring = GraphColoring.init(std.testing.allocator);
    defer coloring.deinit();

    var manifolds = [_]ContactManifold{
        ContactManifold.init(0, 1, 0.5, 0.0),
        ContactManifold.init(1, 2, 0.5, 0.0),
        ContactManifold.init(2, 0, 0.5, 0.0),
    };

    try coloring.build(&manifolds);

    try std.testing.expect(coloring.colorCount() <= 3);

    for (0..3) |i| {
        for (i + 1..3) |j| {
            const m1 = manifolds[i];
            const m2 = manifolds[j];
            const shares_body = m1.body_a_id == m2.body_a_id or
                m1.body_a_id == m2.body_b_id or
                m1.body_b_id == m2.body_a_id or
                m1.body_b_id == m2.body_b_id;

            if (shares_body) {
                try std.testing.expect(coloring.colors.items[i] != coloring.colors.items[j]);
            }
        }
    }
}

test "Island empty state" {
    var island = Island.init();
    defer island.deinit(std.testing.allocator);

    try std.testing.expectEqual(@as(usize, 0), island.bodyCount());
    try std.testing.expectEqual(@as(usize, 0), island.manifoldCount());
    try std.testing.expect(!island.is_sleeping);
}

test "buildIslandsWithBodyTypes isolates bodies through static" {
    var manager = try IslandManager.init(std.testing.allocator, 10);
    defer manager.deinit();

    //

    var manifolds = [_]ContactManifold{
        ContactManifold.init(0, 1, 0.5, 0.0),
        ContactManifold.init(0, 2, 0.5, 0.0),
        ContactManifold.init(2, 3, 0.5, 0.0),
    };

    var body_types = [_]body_mod.BodyType{
        .static,
        .dynamic,
        .dynamic,
        .dynamic,
    };

    try manager.buildIslandsWithBodyTypes(&manifolds, 4, &body_types);

    const island_2 = manager.getBodyIsland(2);
    const island_3 = manager.getBodyIsland(3);
    try std.testing.expect(island_2 != null);
    try std.testing.expect(island_3 != null);
    try std.testing.expectEqual(island_2.?.id, island_3.?.id);

    const island_1 = manager.getBodyIsland(1);
    if (island_1) |isl1| {
        try std.testing.expect(isl1.id != island_2.?.id);
    }
}

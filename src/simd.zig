const std = @import("std");
const math = @import("math.zig");

const Vec3 = math.Vec3;
const AABB = math.AABB;

/// 4-wide float vector
pub const Vec4f = @Vector(4, f32);

/// 4-wide signed integer vector
pub const Vec4i = @Vector(4, i32);

/// 4-wide unsigned integer vector
pub const Vec4u = @Vector(4, u32);

/// 4-wide boolean mask
pub const Vec4b = @Vector(4, bool);

/// Number of elements processed per SIMD operation
pub const SIMD_WIDTH: usize = 4;

/// All zeros
pub const vec4f_zero: Vec4f = @splat(0.0);

/// All ones
pub const vec4f_one: Vec4f = @splat(1.0);

/// Negative infinity
pub const vec4f_neg_inf: Vec4f = @splat(-std.math.inf(f32));

/// Positive infinity
pub const vec4f_pos_inf: Vec4f = @splat(std.math.inf(f32));

/// All true
pub const vec4b_true: Vec4b = @splat(true);

/// All false
pub const vec4b_false: Vec4b = @splat(false);

/// Splat a scalar to all lanes
pub inline fn splat(value: f32) Vec4f {
    return @splat(value);
}

/// Splat an integer to all lanes
pub inline fn splatInt(value: i32) Vec4i {
    return @splat(value);
}

/// Load 4 floats from a slice
pub inline fn load4f(ptr: []const f32) Vec4f {
    if (ptr.len < 4) {
        var result: Vec4f = @splat(0.0);
        for (ptr, 0..) |val, i| {
            result[i] = val;
        }
        return result;
    }
    return Vec4f{ ptr[0], ptr[1], ptr[2], ptr[3] };
}

/// Store 4 floats to a slice
pub inline fn store4f(dst: []f32, vec: Vec4f) void {
    const len = @min(dst.len, 4);
    for (0..len) |i| {
        dst[i] = vec[i];
    }
}

/// Horizontal minimum of 4 floats
pub inline fn hmin(v: Vec4f) f32 {
    const min01 = @min(v[0], v[1]);
    const min23 = @min(v[2], v[3]);
    return @min(min01, min23);
}

/// Horizontal maximum of 4 floats
pub inline fn hmax(v: Vec4f) f32 {
    const max01 = @max(v[0], v[1]);
    const max23 = @max(v[2], v[3]);
    return @max(max01, max23);
}

/// Horizontal sum of 4 floats
pub inline fn hsum(v: Vec4f) f32 {
    return v[0] + v[1] + v[2] + v[3];
}

/// Count true values in boolean mask
pub inline fn countTrue(mask: Vec4b) u32 {
    var count: u32 = 0;
    inline for (0..4) |i| {
        if (mask[i]) count += 1;
    }
    return count;
}

/// Check if any lane is true
pub inline fn any(mask: Vec4b) bool {
    return mask[0] or mask[1] or mask[2] or mask[3];
}

/// Check if all lanes are true
pub inline fn all(mask: Vec4b) bool {
    return mask[0] and mask[1] and mask[2] and mask[3];
}

/// Select between two vectors based on mask
pub inline fn select(mask: Vec4b, a: Vec4f, b: Vec4f) Vec4f {
    return @select(bool, mask, a, b);
}

/// Test if a single AABB overlaps with 4 AABBs stored in SOA format
/// Returns a boolean mask indicating which AABBs overlap
pub fn simdAABBTest(
    bounds_min_x: Vec4f,
    bounds_min_y: Vec4f,
    bounds_min_z: Vec4f,
    bounds_max_x: Vec4f,
    bounds_max_y: Vec4f,
    bounds_max_z: Vec4f,
    query_min: Vec3,
    query_max: Vec3,
) Vec4b {
    const qmin_x: Vec4f = @splat(query_min.x);
    const qmin_y: Vec4f = @splat(query_min.y);
    const qmin_z: Vec4f = @splat(query_min.z);
    const qmax_x: Vec4f = @splat(query_max.x);
    const qmax_y: Vec4f = @splat(query_max.y);
    const qmax_z: Vec4f = @splat(query_max.z);

    const x_max_ge_qmin: Vec4b = bounds_max_x >= qmin_x;
    const x_min_le_qmax: Vec4b = bounds_min_x <= qmax_x;
    const y_max_ge_qmin: Vec4b = bounds_max_y >= qmin_y;
    const y_min_le_qmax: Vec4b = bounds_min_y <= qmax_y;
    const z_max_ge_qmin: Vec4b = bounds_max_z >= qmin_z;
    const z_min_le_qmax: Vec4b = bounds_min_z <= qmax_z;

    const x_overlap = @select(bool, x_max_ge_qmin, x_min_le_qmax, vec4b_false);
    const y_overlap = @select(bool, y_max_ge_qmin, y_min_le_qmax, vec4b_false);
    const z_overlap = @select(bool, z_max_ge_qmin, z_min_le_qmax, vec4b_false);

    const xy_overlap = @select(bool, x_overlap, y_overlap, vec4b_false);
    return @select(bool, xy_overlap, z_overlap, vec4b_false);
}

/// Test if two sets of 4 AABBs overlap pairwise
pub fn simdAABBPairTest(
    a_min_x: Vec4f,
    a_min_y: Vec4f,
    a_min_z: Vec4f,
    a_max_x: Vec4f,
    a_max_y: Vec4f,
    a_max_z: Vec4f,
    b_min_x: Vec4f,
    b_min_y: Vec4f,
    b_min_z: Vec4f,
    b_max_x: Vec4f,
    b_max_y: Vec4f,
    b_max_z: Vec4f,
) Vec4b {
    const x_max_ge: Vec4b = a_max_x >= b_min_x;
    const x_min_le: Vec4b = a_min_x <= b_max_x;
    const y_max_ge: Vec4b = a_max_y >= b_min_y;
    const y_min_le: Vec4b = a_min_y <= b_max_y;
    const z_max_ge: Vec4b = a_max_z >= b_min_z;
    const z_min_le: Vec4b = a_min_z <= b_max_z;

    const x_overlap = @select(bool, x_max_ge, x_min_le, vec4b_false);
    const y_overlap = @select(bool, y_max_ge, y_min_le, vec4b_false);
    const z_overlap = @select(bool, z_max_ge, z_min_le, vec4b_false);

    const xy_overlap = @select(bool, x_overlap, y_overlap, vec4b_false);
    return @select(bool, xy_overlap, z_overlap, vec4b_false);
}

/// Check 4 voxel distances at once against a single position
/// Returns a mask indicating which positions are within the threshold distance
pub fn simdDistanceCheck(
    ref_pos: Vec3,
    pos_x: Vec4f,
    pos_y: Vec4f,
    pos_z: Vec4f,
    threshold_sq: f32,
) Vec4b {
    const ref_x: Vec4f = @splat(ref_pos.x);
    const ref_y: Vec4f = @splat(ref_pos.y);
    const ref_z: Vec4f = @splat(ref_pos.z);

    const dx = pos_x - ref_x;
    const dy = pos_y - ref_y;
    const dz = pos_z - ref_z;

    const dist_sq = dx * dx + dy * dy + dz * dz;
    const threshold: Vec4f = @splat(threshold_sq);

    return dist_sq < threshold;
}

/// Compute squared distances from a reference position to 4 positions
pub fn simdDistanceSquared(
    ref_pos: Vec3,
    pos_x: Vec4f,
    pos_y: Vec4f,
    pos_z: Vec4f,
) Vec4f {
    const ref_x: Vec4f = @splat(ref_pos.x);
    const ref_y: Vec4f = @splat(ref_pos.y);
    const ref_z: Vec4f = @splat(ref_pos.z);

    const dx = pos_x - ref_x;
    const dy = pos_y - ref_y;
    const dz = pos_z - ref_z;

    return dx * dx + dy * dy + dz * dz;
}

/// Compute 4 dot products in parallel
/// Returns vec4 of dot products: axis .
pub fn simdDot4(
    axis: Vec3,
    pos_x: Vec4f,
    pos_y: Vec4f,
    pos_z: Vec4f,
) Vec4f {
    const ax: Vec4f = @splat(axis.x);
    const ay: Vec4f = @splat(axis.y);
    const az: Vec4f = @splat(axis.z);

    return ax * pos_x + ay * pos_y + az * pos_z;
}

/// SOA layout for AABB bounds, optimized for SIMD batch processing
pub const AABBBoundsSOA = struct {
    allocator: std.mem.Allocator,

    min_x: []Vec4f,
    min_y: []Vec4f,
    min_z: []Vec4f,
    max_x: []Vec4f,
    max_y: []Vec4f,
    max_z: []Vec4f,

    user_data: []Vec4u,

    count: usize,

    capacity: usize,

    pub fn init(allocator: std.mem.Allocator) AABBBoundsSOA {
        return .{
            .allocator = allocator,
            .min_x = &.{},
            .min_y = &.{},
            .min_z = &.{},
            .max_x = &.{},
            .max_y = &.{},
            .max_z = &.{},
            .user_data = &.{},
            .count = 0,
            .capacity = 0,
        };
    }

    pub fn deinit(self: *AABBBoundsSOA) void {
        if (self.capacity > 0) {
            self.allocator.free(self.min_x);
            self.allocator.free(self.min_y);
            self.allocator.free(self.min_z);
            self.allocator.free(self.max_x);
            self.allocator.free(self.max_y);
            self.allocator.free(self.max_z);
            self.allocator.free(self.user_data);
        }
        self.* = init(self.allocator);
    }

    pub fn clear(self: *AABBBoundsSOA) void {
        self.count = 0;
    }

    pub fn ensureCapacity(self: *AABBBoundsSOA, min_count: usize) !void {
        const required_groups = (min_count + SIMD_WIDTH - 1) / SIMD_WIDTH;
        if (required_groups <= self.capacity) return;

        const new_capacity = @max(required_groups, self.capacity * 2);
        try self.resize(new_capacity);
    }

    fn resize(self: *AABBBoundsSOA, new_capacity: usize) !void {
        const new_min_x = try self.allocator.alloc(Vec4f, new_capacity);
        const new_min_y = try self.allocator.alloc(Vec4f, new_capacity);
        const new_min_z = try self.allocator.alloc(Vec4f, new_capacity);
        const new_max_x = try self.allocator.alloc(Vec4f, new_capacity);
        const new_max_y = try self.allocator.alloc(Vec4f, new_capacity);
        const new_max_z = try self.allocator.alloc(Vec4f, new_capacity);
        const new_user_data = try self.allocator.alloc(Vec4u, new_capacity);

        if (self.capacity > 0) {
            const copy_len = (self.count + SIMD_WIDTH - 1) / SIMD_WIDTH;
            @memcpy(new_min_x[0..copy_len], self.min_x[0..copy_len]);
            @memcpy(new_min_y[0..copy_len], self.min_y[0..copy_len]);
            @memcpy(new_min_z[0..copy_len], self.min_z[0..copy_len]);
            @memcpy(new_max_x[0..copy_len], self.max_x[0..copy_len]);
            @memcpy(new_max_y[0..copy_len], self.max_y[0..copy_len]);
            @memcpy(new_max_z[0..copy_len], self.max_z[0..copy_len]);
            @memcpy(new_user_data[0..copy_len], self.user_data[0..copy_len]);

            self.allocator.free(self.min_x);
            self.allocator.free(self.min_y);
            self.allocator.free(self.min_z);
            self.allocator.free(self.max_x);
            self.allocator.free(self.max_y);
            self.allocator.free(self.max_z);
            self.allocator.free(self.user_data);
        }

        self.min_x = new_min_x;
        self.min_y = new_min_y;
        self.min_z = new_min_z;
        self.max_x = new_max_x;
        self.max_y = new_max_y;
        self.max_z = new_max_z;
        self.user_data = new_user_data;
        self.capacity = new_capacity;
    }

    /// Add an AABB to the SOA structure
    pub fn add(self: *AABBBoundsSOA, aabb: AABB, user_data_val: u32) !void {
        try self.ensureCapacity(self.count + 1);

        const group_idx = self.count / SIMD_WIDTH;
        const lane_idx = self.count % SIMD_WIDTH;

        if (lane_idx == 0 and group_idx < self.capacity) {
            self.min_x[group_idx] = vec4f_pos_inf;
            self.min_y[group_idx] = vec4f_pos_inf;
            self.min_z[group_idx] = vec4f_pos_inf;
            self.max_x[group_idx] = vec4f_neg_inf;
            self.max_y[group_idx] = vec4f_neg_inf;
            self.max_z[group_idx] = vec4f_neg_inf;
            self.user_data[group_idx] = @splat(std.math.maxInt(u32));
        }

        self.min_x[group_idx][lane_idx] = aabb.min.x;
        self.min_y[group_idx][lane_idx] = aabb.min.y;
        self.min_z[group_idx][lane_idx] = aabb.min.z;
        self.max_x[group_idx][lane_idx] = aabb.max.x;
        self.max_y[group_idx][lane_idx] = aabb.max.y;
        self.max_z[group_idx][lane_idx] = aabb.max.z;
        self.user_data[group_idx][lane_idx] = user_data_val;

        self.count += 1;
    }

    /// Query for all AABBs overlapping the given AABB
    /// Returns user data of overlapping AABBs
    pub fn query(self: *const AABBBoundsSOA, aabb: AABB, results: anytype) !void {
        const num_groups = (self.count + SIMD_WIDTH - 1) / SIMD_WIDTH;

        for (0..num_groups) |group_idx| {
            const overlaps = simdAABBTest(
                self.min_x[group_idx],
                self.min_y[group_idx],
                self.min_z[group_idx],
                self.max_x[group_idx],
                self.max_y[group_idx],
                self.max_z[group_idx],
                aabb.min,
                aabb.max,
            );

            inline for (0..SIMD_WIDTH) |lane| {
                const global_idx = group_idx * SIMD_WIDTH + lane;
                if (global_idx < self.count and overlaps[lane]) {
                    try results.append(self.user_data[group_idx][lane]);
                }
            }
        }
    }

    /// Batch query - query multiple AABBs and collect all overlapping pairs
    pub fn batchQuery(
        self: *const AABBBoundsSOA,
        queries: []const AABB,
        query_ids: []const u32,
        pairs: *std.ArrayList(struct { a: u32, b: u32 }),
    ) !void {
        const num_groups = (self.count + SIMD_WIDTH - 1) / SIMD_WIDTH;

        for (queries, query_ids) |query_aabb, query_id| {
            for (0..num_groups) |group_idx| {
                const overlaps = simdAABBTest(
                    self.min_x[group_idx],
                    self.min_y[group_idx],
                    self.min_z[group_idx],
                    self.max_x[group_idx],
                    self.max_y[group_idx],
                    self.max_z[group_idx],
                    query_aabb.min,
                    query_aabb.max,
                );

                inline for (0..SIMD_WIDTH) |lane| {
                    const global_idx = group_idx * SIMD_WIDTH + lane;
                    const other_id = self.user_data[group_idx][lane];
                    if (global_idx < self.count and overlaps[lane] and query_id != other_id) {
                        if (query_id < other_id) {
                            try pairs.append(.{ .a = query_id, .b = other_id });
                        }
                    }
                }
            }
        }
    }
};

/// SOA layout for voxel positions, optimized for distance checks
pub const VoxelPositionsSOA = struct {
    allocator: std.mem.Allocator,

    pos_x: []Vec4f,
    pos_y: []Vec4f,
    pos_z: []Vec4f,

    voxel_indices: []Vec4i,

    count: usize,
    capacity: usize,

    pub fn init(allocator: std.mem.Allocator) VoxelPositionsSOA {
        return .{
            .allocator = allocator,
            .pos_x = &.{},
            .pos_y = &.{},
            .pos_z = &.{},
            .voxel_indices = &.{},
            .count = 0,
            .capacity = 0,
        };
    }

    pub fn deinit(self: *VoxelPositionsSOA) void {
        if (self.capacity > 0) {
            self.allocator.free(self.pos_x);
            self.allocator.free(self.pos_y);
            self.allocator.free(self.pos_z);
            self.allocator.free(self.voxel_indices);
        }
        self.* = init(self.allocator);
    }

    pub fn clear(self: *VoxelPositionsSOA) void {
        self.count = 0;
    }

    pub fn ensureCapacity(self: *VoxelPositionsSOA, min_count: usize) !void {
        const required_groups = (min_count + SIMD_WIDTH - 1) / SIMD_WIDTH;
        if (required_groups <= self.capacity) return;

        const new_capacity = @max(required_groups, self.capacity * 2, 4);

        const new_pos_x = try self.allocator.alloc(Vec4f, new_capacity);
        const new_pos_y = try self.allocator.alloc(Vec4f, new_capacity);
        const new_pos_z = try self.allocator.alloc(Vec4f, new_capacity);
        const new_voxel_indices = try self.allocator.alloc(Vec4i, new_capacity);

        if (self.capacity > 0) {
            const copy_len = (self.count + SIMD_WIDTH - 1) / SIMD_WIDTH;
            if (copy_len > 0) {
                @memcpy(new_pos_x[0..copy_len], self.pos_x[0..copy_len]);
                @memcpy(new_pos_y[0..copy_len], self.pos_y[0..copy_len]);
                @memcpy(new_pos_z[0..copy_len], self.pos_z[0..copy_len]);
                @memcpy(new_voxel_indices[0..copy_len], self.voxel_indices[0..copy_len]);
            }
            self.allocator.free(self.pos_x);
            self.allocator.free(self.pos_y);
            self.allocator.free(self.pos_z);
            self.allocator.free(self.voxel_indices);
        }

        self.pos_x = new_pos_x;
        self.pos_y = new_pos_y;
        self.pos_z = new_pos_z;
        self.voxel_indices = new_voxel_indices;
        self.capacity = new_capacity;
    }

    /// Add a voxel position
    pub fn add(self: *VoxelPositionsSOA, pos: Vec3, voxel_index: i32) !void {
        try self.ensureCapacity(self.count + 1);

        const group_idx = self.count / SIMD_WIDTH;
        const lane_idx = self.count % SIMD_WIDTH;

        if (lane_idx == 0 and group_idx < self.capacity) {
            self.pos_x[group_idx] = vec4f_zero;
            self.pos_y[group_idx] = vec4f_zero;
            self.pos_z[group_idx] = vec4f_zero;
            self.voxel_indices[group_idx] = @splat(-1);
        }

        self.pos_x[group_idx][lane_idx] = pos.x;
        self.pos_y[group_idx][lane_idx] = pos.y;
        self.pos_z[group_idx][lane_idx] = pos.z;
        self.voxel_indices[group_idx][lane_idx] = voxel_index;

        self.count += 1;
    }

    /// Find all voxels within distance threshold of reference position
    pub fn findWithinDistance(
        self: *const VoxelPositionsSOA,
        ref_pos: Vec3,
        threshold_sq: f32,
        results: anytype,
    ) !void {
        const num_groups = (self.count + SIMD_WIDTH - 1) / SIMD_WIDTH;

        for (0..num_groups) |group_idx| {
            const within = simdDistanceCheck(
                ref_pos,
                self.pos_x[group_idx],
                self.pos_y[group_idx],
                self.pos_z[group_idx],
                threshold_sq,
            );

            inline for (0..SIMD_WIDTH) |lane| {
                const global_idx = group_idx * SIMD_WIDTH + lane;
                if (global_idx < self.count and within[lane]) {
                    try results.append(self.voxel_indices[group_idx][lane]);
                }
            }
        }
    }
};

/// Process items in SIMD-aligned batches
pub fn BatchIterator(comptime T: type) type {
    return struct {
        items: []const T,
        index: usize,

        const Self = @This();

        pub fn init(items: []const T) Self {
            return .{
                .items = items,
                .index = 0,
            };
        }

        /// Get next batch of up to 4 items
        /// Returns null when exhausted
        pub fn next(self: *Self) ?struct {
            data: [SIMD_WIDTH]T,
            count: usize,
            start_index: usize,
        } {
            if (self.index >= self.items.len) return null;

            const remaining = self.items.len - self.index;
            const batch_count = @min(remaining, SIMD_WIDTH);
            const start = self.index;

            var data: [SIMD_WIDTH]T = undefined;
            for (0..batch_count) |i| {
                data[i] = self.items[self.index + i];
            }

            self.index += batch_count;

            return .{
                .data = data,
                .count = batch_count,
                .start_index = start,
            };
        }
    };
}

/// Wrapper to make ArrayListUnmanaged compatible with query interface
const ResultsWrapper = struct {
    list: *std.ArrayListUnmanaged(u32),
    allocator: std.mem.Allocator,

    pub fn append(self: ResultsWrapper, value: u32) !void {
        try self.list.append(self.allocator, value);
    }
};

const ResultsWrapperI32 = struct {
    list: *std.ArrayListUnmanaged(i32),
    allocator: std.mem.Allocator,

    pub fn append(self: ResultsWrapperI32, value: i32) !void {
        try self.list.append(self.allocator, value);
    }
};

test "Vec4f basic operations" {
    const a: Vec4f = .{ 1.0, 2.0, 3.0, 4.0 };
    const b: Vec4f = .{ 5.0, 6.0, 7.0, 8.0 };

    const sum = a + b;
    try std.testing.expectEqual(@as(f32, 6.0), sum[0]);
    try std.testing.expectEqual(@as(f32, 8.0), sum[1]);
    try std.testing.expectEqual(@as(f32, 10.0), sum[2]);
    try std.testing.expectEqual(@as(f32, 12.0), sum[3]);

    const prod = a * b;
    try std.testing.expectEqual(@as(f32, 5.0), prod[0]);
    try std.testing.expectEqual(@as(f32, 12.0), prod[1]);
}

test "horizontal operations" {
    const v: Vec4f = .{ 1.0, 5.0, 2.0, 4.0 };

    try std.testing.expectEqual(@as(f32, 1.0), hmin(v));
    try std.testing.expectEqual(@as(f32, 5.0), hmax(v));
    try std.testing.expectEqual(@as(f32, 12.0), hsum(v));
}

test "boolean mask operations" {
    const mask1: Vec4b = .{ true, false, true, false };
    const mask2: Vec4b = .{ true, true, true, true };
    const mask3: Vec4b = .{ false, false, false, false };

    try std.testing.expectEqual(@as(u32, 2), countTrue(mask1));
    try std.testing.expectEqual(@as(u32, 4), countTrue(mask2));
    try std.testing.expectEqual(@as(u32, 0), countTrue(mask3));

    try std.testing.expect(any(mask1));
    try std.testing.expect(all(mask2));
    try std.testing.expect(!any(mask3));
}

test "SIMD AABB test" {
    const min_x: Vec4f = .{ 0.0, 5.0, 10.0, 15.0 };
    const min_y: Vec4f = .{ 0.0, 0.0, 0.0, 0.0 };
    const min_z: Vec4f = .{ 0.0, 0.0, 0.0, 0.0 };
    const max_x: Vec4f = .{ 2.0, 7.0, 12.0, 17.0 };
    const max_y: Vec4f = .{ 2.0, 2.0, 2.0, 2.0 };
    const max_z: Vec4f = .{ 2.0, 2.0, 2.0, 2.0 };

    const query_min = Vec3.init(1.0, 0.0, 0.0);
    const query_max = Vec3.init(6.0, 1.0, 1.0);

    const overlaps = simdAABBTest(
        min_x,
        min_y,
        min_z,
        max_x,
        max_y,
        max_z,
        query_min,
        query_max,
    );

    try std.testing.expect(overlaps[0]);
    try std.testing.expect(overlaps[1]);
    try std.testing.expect(!overlaps[2]);
    try std.testing.expect(!overlaps[3]);
}

test "SIMD distance check" {
    const ref = Vec3.init(0.0, 0.0, 0.0);

    const pos_x: Vec4f = .{ 1.0, 2.0, 3.0, 10.0 };
    const pos_y: Vec4f = .{ 0.0, 0.0, 0.0, 0.0 };
    const pos_z: Vec4f = .{ 0.0, 0.0, 0.0, 0.0 };

    const within = simdDistanceCheck(ref, pos_x, pos_y, pos_z, 25.0);

    try std.testing.expect(within[0]);
    try std.testing.expect(within[1]);
    try std.testing.expect(within[2]);
    try std.testing.expect(!within[3]);
}

test "AABBBoundsSOA basic operations" {
    const allocator = std.testing.allocator;

    var soa = AABBBoundsSOA.init(allocator);
    defer soa.deinit();

    const aabb1 = AABB{ .min = Vec3.init(0, 0, 0), .max = Vec3.init(1, 1, 1) };
    const aabb2 = AABB{ .min = Vec3.init(5, 5, 5), .max = Vec3.init(6, 6, 6) };
    const aabb3 = AABB{ .min = Vec3.init(0.5, 0.5, 0.5), .max = Vec3.init(1.5, 1.5, 1.5) };

    try soa.add(aabb1, 0);
    try soa.add(aabb2, 1);
    try soa.add(aabb3, 2);

    try std.testing.expectEqual(@as(usize, 3), soa.count);

    var results = std.ArrayListUnmanaged(u32){};
    defer results.deinit(allocator);

    const query = AABB{ .min = Vec3.init(0.25, 0.25, 0.25), .max = Vec3.init(0.75, 0.75, 0.75) };
    try soa.query(query, ResultsWrapper{ .list = &results, .allocator = allocator });

    try std.testing.expectEqual(@as(usize, 2), results.items.len);
}

test "VoxelPositionsSOA distance query" {
    const allocator = std.testing.allocator;

    var soa = VoxelPositionsSOA.init(allocator);
    defer soa.deinit();

    try soa.add(Vec3.init(0, 0, 0), 0);
    try soa.add(Vec3.init(1, 0, 0), 1);
    try soa.add(Vec3.init(2, 0, 0), 2);
    try soa.add(Vec3.init(10, 0, 0), 3);
    try soa.add(Vec3.init(0.5, 0.5, 0), 4);

    try std.testing.expectEqual(@as(usize, 5), soa.count);

    var results = std.ArrayListUnmanaged(i32){};
    defer results.deinit(allocator);

    const ref = Vec3.init(0, 0, 0);
    try soa.findWithinDistance(ref, 2.5 * 2.5, ResultsWrapperI32{ .list = &results, .allocator = allocator });

    try std.testing.expectEqual(@as(usize, 4), results.items.len);
}

test "BatchIterator" {
    const items = [_]u32{ 1, 2, 3, 4, 5, 6, 7, 8, 9 };

    var iter = BatchIterator(u32).init(&items);

    const batch1 = iter.next().?;
    try std.testing.expectEqual(@as(usize, 4), batch1.count);
    try std.testing.expectEqual(@as(usize, 0), batch1.start_index);
    try std.testing.expectEqual(@as(u32, 1), batch1.data[0]);

    const batch2 = iter.next().?;
    try std.testing.expectEqual(@as(usize, 4), batch2.count);
    try std.testing.expectEqual(@as(usize, 4), batch2.start_index);
    try std.testing.expectEqual(@as(u32, 5), batch2.data[0]);

    const batch3 = iter.next().?;
    try std.testing.expectEqual(@as(usize, 1), batch3.count);
    try std.testing.expectEqual(@as(usize, 8), batch3.start_index);
    try std.testing.expectEqual(@as(u32, 9), batch3.data[0]);

    try std.testing.expect(iter.next() == null);
}

const std = @import("std");
const math = @import("math.zig");
const collision_voxel = @import("collision_voxel.zig");

const Vec3 = math.Vec3;
const IVec3 = math.IVec3;
const AABB = math.AABB;
const OBB = math.OBB;
const Transform = math.Transform;
const Quat = math.Quat;

const CollisionVoxel = collision_voxel.CollisionVoxel;
const VoxelType = collision_voxel.VoxelType;

/// Size of a collision brick in voxels per dimension
pub const BRICK_SIZE: usize = 8;

/// Total voxels in a brick
pub const BRICK_VOLUME: usize = BRICK_SIZE * BRICK_SIZE * BRICK_SIZE;

/// Default voxel size in world units
pub const DEFAULT_VOXEL_SIZE: f32 = 0.1;

/// Pre-computed collision data for an 8x8x8 brick
pub const CollisionBrick = struct {
    /// Brick position in brick coordinates
    brick_pos: IVec3,

    /// Local AABB in voxel-space
    local_aabb: AABB,

    /// Whether this brick has any solid voxels
    is_empty: bool,

    /// Whether all voxels in this brick are the same type
    is_homogeneous: bool,

    /// The voxel type if homogeneous
    homogeneous_type: VoxelType,

    /// Number of solid voxels in this brick
    solid_count: u16,

    /// Collision voxel data
    /// Indexed as: x + y * 8 + z * 64
    voxels: [BRICK_VOLUME]CollisionVoxel,

    pub const empty = CollisionBrick{
        .brick_pos = IVec3.zero,
        .local_aabb = AABB.empty,
        .is_empty = true,
        .is_homogeneous = true,
        .homogeneous_type = .empty,
        .solid_count = 0,
        .voxels = [_]CollisionVoxel{CollisionVoxel.empty} ** BRICK_VOLUME,
    };

    /// Initialize an empty brick at a position
    pub fn init(brick_pos: IVec3) CollisionBrick {
        var brick = CollisionBrick.empty;
        brick.brick_pos = brick_pos;
        return brick;
    }

    /// Convert local coordinates to linear index
    pub fn coordToIndex(x: u3, y: u3, z: u3) usize {
        return @as(usize, x) + @as(usize, y) * BRICK_SIZE + @as(usize, z) * BRICK_SIZE * BRICK_SIZE;
    }

    /// Convert linear index to local coordinates
    pub fn indexToCoord(index: usize) struct { x: u3, y: u3, z: u3 } {
        return .{
            .x = @intCast(index % BRICK_SIZE),
            .y = @intCast((index / BRICK_SIZE) % BRICK_SIZE),
            .z = @intCast(index / (BRICK_SIZE * BRICK_SIZE)),
        };
    }

    /// Get collision voxel at local coordinates
    pub fn getVoxel(self: *const CollisionBrick, x: u3, y: u3, z: u3) CollisionVoxel {
        return self.voxels[coordToIndex(x, y, z)];
    }

    /// Set collision voxel at local coordinates
    pub fn setVoxel(self: *CollisionBrick, x: u3, y: u3, z: u3, voxel: CollisionVoxel) void {
        self.voxels[coordToIndex(x, y, z)] = voxel;
    }

    /// Recompute metadata
    pub fn recomputeMetadata(self: *CollisionBrick, voxel_size: f32) void {
        self.solid_count = 0;
        var first_type: ?VoxelType = null;
        self.is_homogeneous = true;

        var min_bound = Vec3.splat(std.math.inf(f32));
        var max_bound = Vec3.splat(-std.math.inf(f32));

        for (0..BRICK_VOLUME) |i| {
            const voxel = self.voxels[i];
            const vtype = voxel.voxel_type;

            if (vtype != .empty) {
                self.solid_count += 1;

                const coord = indexToCoord(i);
                const local_pos = Vec3.init(
                    @as(f32, @floatFromInt(self.brick_pos.x)) * @as(f32, BRICK_SIZE) + @as(f32, @floatFromInt(coord.x)),
                    @as(f32, @floatFromInt(self.brick_pos.y)) * @as(f32, BRICK_SIZE) + @as(f32, @floatFromInt(coord.y)),
                    @as(f32, @floatFromInt(self.brick_pos.z)) * @as(f32, BRICK_SIZE) + @as(f32, @floatFromInt(coord.z)),
                ).scale(voxel_size);

                min_bound = min_bound.minComponents(local_pos);
                max_bound = max_bound.maxComponents(local_pos.add(Vec3.splat(voxel_size)));
            }

            if (first_type) |ft| {
                if (vtype != ft) {
                    self.is_homogeneous = false;
                }
            } else {
                first_type = vtype;
            }
        }

        self.is_empty = self.solid_count == 0;
        self.homogeneous_type = first_type orelse .empty;

        if (self.is_empty) {
            self.local_aabb = AABB.empty;
        } else {
            self.local_aabb = AABB.init(min_bound, max_bound);
        }
    }

    /// Check if local coordinates are valid
    pub fn isValidCoord(x: i32, y: i32, z: i32) bool {
        return x >= 0 and x < BRICK_SIZE and
            y >= 0 and y < BRICK_SIZE and
            z >= 0 and z < BRICK_SIZE;
    }
};

/// Collision representation for a voxel object
pub const VoxelCollider = struct {
    /// Allocator used for dynamic arrays
    allocator: std.mem.Allocator,

    /// Size of each voxel in world units
    voxel_size: f32,

    /// Collision bricks
    bricks: std.AutoHashMap(u64, CollisionBrick),

    /// Bounds in voxel coordinates
    min_bound: IVec3,
    max_bound: IVec3,

    /// Cached world-space AABB
    local_aabb: AABB,

    /// Total number of solid voxels
    solid_count: u32,

    /// Whether the collider needs metadata recomputation
    dirty: bool,

    /// Create a new empty collider
    pub fn init(allocator: std.mem.Allocator, voxel_size: f32) VoxelCollider {
        return .{
            .allocator = allocator,
            .voxel_size = voxel_size,
            .bricks = std.AutoHashMap(u64, CollisionBrick).init(allocator),
            .min_bound = IVec3.zero,
            .max_bound = IVec3.zero,
            .local_aabb = AABB.empty,
            .solid_count = 0,
            .dirty = true,
        };
    }

    /// Clean up allocated resources
    pub fn deinit(self: *VoxelCollider) void {
        self.bricks.deinit();
    }

    /// Hash a brick position for storage
    fn hashBrickPos(pos: IVec3) u64 {
        const ux: u64 = @bitCast(@as(i64, pos.x));
        const uy: u64 = @bitCast(@as(i64, pos.y));
        const uz: u64 = @bitCast(@as(i64, pos.z));
        return ux ^ (uy *% 0x517cc1b727220a95) ^ (uz *% 0x9e3779b97f4a7c15);
    }

    /// Convert voxel coordinates to brick position
    pub fn voxelToBrickPos(voxel_pos: IVec3) IVec3 {
        return IVec3.init(
            @divFloor(voxel_pos.x, @as(i32, BRICK_SIZE)),
            @divFloor(voxel_pos.y, @as(i32, BRICK_SIZE)),
            @divFloor(voxel_pos.z, @as(i32, BRICK_SIZE)),
        );
    }

    /// Convert voxel coordinates to local coordinates within a brick
    pub fn voxelToLocalCoord(voxel_pos: IVec3) struct { x: u3, y: u3, z: u3 } {
        return .{
            .x = @intCast(@mod(voxel_pos.x, @as(i32, BRICK_SIZE))),
            .y = @intCast(@mod(voxel_pos.y, @as(i32, BRICK_SIZE))),
            .z = @intCast(@mod(voxel_pos.z, @as(i32, BRICK_SIZE))),
        };
    }

    /// Convert voxel coordinates to local position
    pub fn voxelToLocal(self: *const VoxelCollider, voxel_pos: IVec3) Vec3 {
        const half = self.voxel_size * 0.5;
        return Vec3.init(
            @as(f32, @floatFromInt(voxel_pos.x)) * self.voxel_size + half,
            @as(f32, @floatFromInt(voxel_pos.y)) * self.voxel_size + half,
            @as(f32, @floatFromInt(voxel_pos.z)) * self.voxel_size + half,
        );
    }

    /// Convert local position to voxel coordinates
    pub fn localToVoxel(self: *const VoxelCollider, local_pos: Vec3) IVec3 {
        return IVec3.init(
            @intFromFloat(@floor(local_pos.x / self.voxel_size)),
            @intFromFloat(@floor(local_pos.y / self.voxel_size)),
            @intFromFloat(@floor(local_pos.z / self.voxel_size)),
        );
    }

    /// Get or create a brick at the given brick position
    pub fn getOrCreateBrick(self: *VoxelCollider, brick_pos: IVec3) !*CollisionBrick {
        const hash = hashBrickPos(brick_pos);
        const result = try self.bricks.getOrPut(hash);
        if (!result.found_existing) {
            result.value_ptr.* = CollisionBrick.init(brick_pos);
        }
        return result.value_ptr;
    }

    /// Get a brick at the given brick position
    pub fn getBrick(self: *const VoxelCollider, brick_pos: IVec3) ?*const CollisionBrick {
        const hash = hashBrickPos(brick_pos);
        if (self.bricks.getPtr(hash)) |ptr| {
            return ptr;
        }
        return null;
    }

    /// Get a mutable brick at the given brick position
    pub fn getBrickMut(self: *VoxelCollider, brick_pos: IVec3) ?*CollisionBrick {
        const hash = hashBrickPos(brick_pos);
        return self.bricks.getPtr(hash);
    }

    /// Get collision voxel at voxel coordinates
    pub fn getCollisionVoxel(self: *const VoxelCollider, voxel_pos: IVec3) CollisionVoxel {
        const brick_pos = voxelToBrickPos(voxel_pos);
        if (self.getBrick(brick_pos)) |brick| {
            const local = voxelToLocalCoord(voxel_pos);
            return brick.getVoxel(local.x, local.y, local.z);
        }
        return CollisionVoxel.empty;
    }

    /// Set collision voxel at voxel coordinates
    pub fn setCollisionVoxel(self: *VoxelCollider, voxel_pos: IVec3, voxel: CollisionVoxel) !void {
        const brick_pos = voxelToBrickPos(voxel_pos);
        const brick = try self.getOrCreateBrick(brick_pos);
        const local = voxelToLocalCoord(voxel_pos);
        brick.setVoxel(local.x, local.y, local.z, voxel);
        self.dirty = true;
    }

    /// Check if voxel coordinates are within bounds
    pub fn isValidCoord(self: *const VoxelCollider, voxel_pos: IVec3) bool {
        return voxel_pos.x >= self.min_bound.x and voxel_pos.x < self.max_bound.x and
            voxel_pos.y >= self.min_bound.y and voxel_pos.y < self.max_bound.y and
            voxel_pos.z >= self.min_bound.z and voxel_pos.z < self.max_bound.z;
    }

    /// Recompute all metadata
    pub fn recomputeMetadata(self: *VoxelCollider) void {
        self.min_bound = IVec3.init(std.math.maxInt(i32), std.math.maxInt(i32), std.math.maxInt(i32));
        self.max_bound = IVec3.init(std.math.minInt(i32), std.math.minInt(i32), std.math.minInt(i32));
        self.solid_count = 0;
        self.local_aabb = AABB.empty;

        var iter = self.bricks.valueIterator();
        while (iter.next()) |brick| {
            @constCast(brick).recomputeMetadata(self.voxel_size);

            if (!brick.is_empty) {
                self.solid_count += brick.solid_count;

                const brick_base_x = brick.brick_pos.x * @as(i32, BRICK_SIZE);
                const brick_base_y = brick.brick_pos.y * @as(i32, BRICK_SIZE);
                const brick_base_z = brick.brick_pos.z * @as(i32, BRICK_SIZE);

                for (0..BRICK_VOLUME) |i| {
                    const voxel = brick.voxels[i];
                    if (voxel.voxel_type != .empty) {
                        const coord = CollisionBrick.indexToCoord(i);

                        const voxel_x = brick_base_x + @as(i32, coord.x);
                        const voxel_y = brick_base_y + @as(i32, coord.y);
                        const voxel_z = brick_base_z + @as(i32, coord.z);

                        self.min_bound = IVec3.init(
                            @min(self.min_bound.x, voxel_x),
                            @min(self.min_bound.y, voxel_y),
                            @min(self.min_bound.z, voxel_z),
                        );

                        self.max_bound = IVec3.init(
                            @max(self.max_bound.x, voxel_x + 1),
                            @max(self.max_bound.y, voxel_y + 1),
                            @max(self.max_bound.z, voxel_z + 1),
                        );
                    }
                }

                self.local_aabb = self.local_aabb.merge(brick.local_aabb);
            }
        }

        if (self.solid_count == 0) {
            self.min_bound = IVec3.zero;
            self.max_bound = IVec3.zero;
        }

        self.dirty = false;
    }

    /// Get the local-space AABB
    pub fn getLocalAABB(self: *const VoxelCollider) AABB {
        return self.local_aabb;
    }

    /// Get world-space AABB for the collider with given transform
    pub fn getWorldAABB(self: *const VoxelCollider, transform: Transform) AABB {
        return self.local_aabb.transform(transform);
    }

    /// Get world-space OBB for the collider with given transform
    pub fn getWorldOBB(self: *const VoxelCollider, transform: Transform) OBB {
        return OBB.fromAABB(self.local_aabb, transform);
    }

    /// Get the local axes in world space for SAT tests
    pub fn getWorldAxes(self: *const VoxelCollider, transform: Transform) [3]Vec3 {
        _ = self;
        return .{
            transform.rotation.rotate(Vec3.unit_x),
            transform.rotation.rotate(Vec3.unit_y),
            transform.rotation.rotate(Vec3.unit_z),
        };
    }

    /// Get dimensions in voxels
    pub fn getDimensionsVoxels(self: *const VoxelCollider) IVec3 {
        return self.max_bound.sub(self.min_bound);
    }

    /// Get dimensions in world units
    pub fn getDimensions(self: *const VoxelCollider) Vec3 {
        const dims = self.getDimensionsVoxels();
        return Vec3.init(
            @as(f32, @floatFromInt(dims.x)) * self.voxel_size,
            @as(f32, @floatFromInt(dims.y)) * self.voxel_size,
            @as(f32, @floatFromInt(dims.z)) * self.voxel_size,
        );
    }

    /// Get the center of mass in local coordinates
    pub fn getCenterOfMass(self: *const VoxelCollider) Vec3 {
        if (self.solid_count == 0) return Vec3.zero;

        var sum = Vec3.zero;
        var count: u32 = 0;

        var iter = self.bricks.valueIterator();
        while (iter.next()) |brick| {
            if (brick.is_empty) continue;

            for (0..BRICK_VOLUME) |i| {
                if (brick.voxels[i].voxel_type != .empty) {
                    const coord = CollisionBrick.indexToCoord(i);
                    const voxel_pos = IVec3.init(
                        brick.brick_pos.x * @as(i32, BRICK_SIZE) + @as(i32, coord.x),
                        brick.brick_pos.y * @as(i32, BRICK_SIZE) + @as(i32, coord.y),
                        brick.brick_pos.z * @as(i32, BRICK_SIZE) + @as(i32, coord.z),
                    );

                    const pos = self.voxelToLocal(voxel_pos).add(Vec3.splat(self.voxel_size * 0.5));
                    sum = sum.add(pos);
                    count += 1;
                }
            }
        }

        return sum.scale(1.0 / @as(f32, @floatFromInt(count)));
    }

    /// Iterator over all non-empty bricks
    pub fn brickIterator(self: *const VoxelCollider) std.AutoHashMap(u64, CollisionBrick).ValueIterator {
        return self.bricks.valueIterator();
    }
};

test "CollisionBrick coordinate conversion" {
    const index = CollisionBrick.coordToIndex(3, 4, 5);
    const coord = CollisionBrick.indexToCoord(index);
    try std.testing.expectEqual(@as(u3, 3), coord.x);
    try std.testing.expectEqual(@as(u3, 4), coord.y);
    try std.testing.expectEqual(@as(u3, 5), coord.z);
}

test "CollisionBrick set and get voxel" {
    var brick = CollisionBrick.init(IVec3.zero);

    const voxel = CollisionVoxel.init(.corner, 5);
    brick.setVoxel(2, 3, 4, voxel);

    const retrieved = brick.getVoxel(2, 3, 4);
    try std.testing.expectEqual(VoxelType.corner, retrieved.voxel_type);
    try std.testing.expectEqual(@as(u5, 5), retrieved.normal_index);
}

test "CollisionBrick recompute metadata" {
    var brick = CollisionBrick.init(IVec3.zero);

    brick.setVoxel(0, 0, 0, CollisionVoxel.init(.corner, 0));
    brick.setVoxel(1, 0, 0, CollisionVoxel.init(.edge, 1));
    brick.setVoxel(0, 1, 0, CollisionVoxel.init(.face, 2));

    brick.recomputeMetadata(0.1);

    try std.testing.expectEqual(@as(u16, 3), brick.solid_count);
    try std.testing.expect(!brick.is_empty);
    try std.testing.expect(!brick.is_homogeneous);
}

test "VoxelCollider init and deinit" {
    var collider = VoxelCollider.init(std.testing.allocator, 0.1);
    defer collider.deinit();

    try std.testing.expectEqual(@as(f32, 0.1), collider.voxel_size);
    try std.testing.expectEqual(@as(u32, 0), collider.solid_count);
}

test "VoxelCollider set and get voxel" {
    var collider = VoxelCollider.init(std.testing.allocator, 0.1);
    defer collider.deinit();

    const pos = IVec3.init(5, 10, 15);
    const voxel = CollisionVoxel.init(.corner, 3);

    try collider.setCollisionVoxel(pos, voxel);

    const retrieved = collider.getCollisionVoxel(pos);
    try std.testing.expectEqual(VoxelType.corner, retrieved.voxel_type);
    try std.testing.expectEqual(@as(u5, 3), retrieved.normal_index);
}

test "VoxelCollider brick position calculation" {
    var pos = IVec3.init(15, 20, 25);
    var brick_pos = VoxelCollider.voxelToBrickPos(pos);
    try std.testing.expectEqual(@as(i32, 1), brick_pos.x);
    try std.testing.expectEqual(@as(i32, 2), brick_pos.y);
    try std.testing.expectEqual(@as(i32, 3), brick_pos.z);

    pos = IVec3.init(-1, -10, -20);
    brick_pos = VoxelCollider.voxelToBrickPos(pos);
    try std.testing.expectEqual(@as(i32, -1), brick_pos.x);
    try std.testing.expectEqual(@as(i32, -2), brick_pos.y);
    try std.testing.expectEqual(@as(i32, -3), brick_pos.z);
}

test "VoxelCollider local coordinate conversion" {
    const pos = IVec3.init(13, 18, 23);
    const local = VoxelCollider.voxelToLocalCoord(pos);
    try std.testing.expectEqual(@as(u3, 5), local.x);
    try std.testing.expectEqual(@as(u3, 2), local.y);
    try std.testing.expectEqual(@as(u3, 7), local.z);
}

test "VoxelCollider recompute metadata" {
    var collider = VoxelCollider.init(std.testing.allocator, 0.1);
    defer collider.deinit();

    try collider.setCollisionVoxel(IVec3.init(0, 0, 0), CollisionVoxel.init(.corner, 0));
    try collider.setCollisionVoxel(IVec3.init(10, 10, 10), CollisionVoxel.init(.edge, 1));

    collider.recomputeMetadata();

    try std.testing.expectEqual(@as(u32, 2), collider.solid_count);
    try std.testing.expect(!collider.dirty);
}

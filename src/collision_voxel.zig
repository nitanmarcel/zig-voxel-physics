const std = @import("std");
const math = @import("math.zig");
const Vec3 = math.Vec3;
const IVec3 = math.IVec3;

/// Classification of a voxel based on its neighborhood
pub const VoxelType = enum(u3) {
    /// No collision
    empty = 0,

    /// Corner voxel: 3+ empty face neighbors
    corner = 1,

    /// Edge voxel: exactly 2 empty face neighbors
    edge = 2,

    /// Face voxel: exactly 1 empty face neighbor
    face = 3,

    /// Inside voxel: 0 empty face neighbors
    inside = 4,

    /// Check if this voxel type participates in collision detection
    pub fn isCollidable(self: VoxelType) bool {
        return self != .empty;
    }

    /// Get the priority for collision checking
    pub fn priority(self: VoxelType) u8 {
        return switch (self) {
            .empty => 0,
            .inside => 1,
            .face => 2,
            .edge => 3,
            .corner => 4,
        };
    }
};

/// Number of possible normal directions
pub const NUM_NORMALS: usize = 26;

/// Normal direction lookup table
/// Index 0-5: Cardinal directions
/// Index 6-17: Edge directions
/// Index 18-25: Corner directions
pub const NORMAL_LUT: [NUM_NORMALS]IVec3 = .{
    IVec3.init(1, 0, 0),
    IVec3.init(-1, 0, 0),
    IVec3.init(0, 1, 0),
    IVec3.init(0, -1, 0),
    IVec3.init(0, 0, 1),
    IVec3.init(0, 0, -1),

    IVec3.init(1, 1, 0),
    IVec3.init(1, -1, 0),
    IVec3.init(-1, 1, 0),
    IVec3.init(-1, -1, 0),
    IVec3.init(1, 0, 1),
    IVec3.init(1, 0, -1),
    IVec3.init(-1, 0, 1),
    IVec3.init(-1, 0, -1),
    IVec3.init(0, 1, 1),
    IVec3.init(0, 1, -1),
    IVec3.init(0, -1, 1),
    IVec3.init(0, -1, -1),

    IVec3.init(1, 1, 1),
    IVec3.init(1, 1, -1),
    IVec3.init(1, -1, 1),
    IVec3.init(1, -1, -1),
    IVec3.init(-1, 1, 1),
    IVec3.init(-1, 1, -1),
    IVec3.init(-1, -1, 1),
    IVec3.init(-1, -1, -1),
};

/// Convert a normal lookup index to a normalized Vec3
pub fn getNormalVec3(index: u5) Vec3 {
    if (index >= NUM_NORMALS) return Vec3.zero;
    const n = NORMAL_LUT[index];
    return n.toVec3().normalize();
}

/// Find the closest normal index for a given direction
pub fn findClosestNormalIndex(dir: Vec3) u5 {
    var best_index: u5 = 0;
    var best_dot: f32 = -std.math.inf(f32);

    for (NORMAL_LUT, 0..) |normal, i| {
        const n = normal.toVec3().normalize();
        const d = dir.dot(n);
        if (d > best_dot) {
            best_dot = d;
            best_index = @intCast(i);
        }
    }

    return best_index;
}

/// Compute normal index from empty neighbor mask
pub fn computeNormalIndex(empty_neighbors: u6) u5 {
    const empty_count = @popCount(empty_neighbors);

    if (empty_count == 0) {
        return 0;
    }

    var dx: i32 = 0;
    var dy: i32 = 0;
    var dz: i32 = 0;

    if (empty_neighbors & 0b000001 != 0) dx -= 1;
    if (empty_neighbors & 0b000010 != 0) dx += 1;
    if (empty_neighbors & 0b000100 != 0) dy -= 1;
    if (empty_neighbors & 0b001000 != 0) dy += 1;
    if (empty_neighbors & 0b010000 != 0) dz -= 1;
    if (empty_neighbors & 0b100000 != 0) dz += 1;

    const target = IVec3.init(dx, dy, dz);

    for (NORMAL_LUT, 0..) |normal, i| {
        if (normal.eql(target)) {
            return @intCast(i);
        }
    }

    return findClosestNormalIndex(target.toVec3());
}

/// Compact collision data for a single voxel
pub const CollisionVoxel = packed struct {
    /// Classification of this voxel
    voxel_type: VoxelType,

    /// Index into NORMAL_LUT for outward-facing normal
    normal_index: u5,

    pub const empty = CollisionVoxel{
        .voxel_type = .empty,
        .normal_index = 0,
    };

    /// Create a collision voxel from its components
    pub fn init(voxel_type: VoxelType, normal_index: u5) CollisionVoxel {
        return .{
            .voxel_type = voxel_type,
            .normal_index = normal_index,
        };
    }

    /// Check if this voxel is empty
    pub fn isEmpty(self: CollisionVoxel) bool {
        return self.voxel_type == .empty;
    }

    /// Check if this voxel participates in collision
    pub fn isCollidable(self: CollisionVoxel) bool {
        return self.voxel_type.isCollidable();
    }

    /// Get the normal direction as a Vec3
    pub fn getNormal(self: CollisionVoxel) Vec3 {
        return getNormalVec3(self.normal_index);
    }

    /// Check if two collision voxels should test against each other
    pub fn shouldCollide(self: CollisionVoxel, other: CollisionVoxel) bool {
        if (self.voxel_type == .empty or other.voxel_type == .empty) {
            return false;
        }

        if (self.voxel_type == .corner or other.voxel_type == .corner) {
            return true;
        }

        if (self.voxel_type == .edge and other.voxel_type == .edge) {
            return true;
        }

        return false;
    }
};

// Verify packed struct size
comptime {
    if (@sizeOf(CollisionVoxel) != 1) {
        @compileError("CollisionVoxel must be exactly 1 byte");
    }
}

/// Classify a voxel based on its face neighbors
pub fn classifyVoxel(is_solid: bool, empty_neighbors: u6) CollisionVoxel {
    if (!is_solid) {
        return CollisionVoxel.empty;
    }

    const empty_count = @popCount(empty_neighbors);
    const voxel_type: VoxelType = switch (empty_count) {
        0 => .inside,
        1 => .face,
        2 => .edge,
        else => .corner,
    };

    const normal_index = computeNormalIndex(empty_neighbors);

    return CollisionVoxel.init(voxel_type, normal_index);
}

/// Build collision voxel from raw neighbor data
pub fn classifyFromNeighbors(
    is_solid: bool,
    neg_x: bool,
    pos_x: bool,
    neg_y: bool,
    pos_y: bool,
    neg_z: bool,
    pos_z: bool,
) CollisionVoxel {
    if (!is_solid) {
        return CollisionVoxel.empty;
    }

    var empty_mask: u6 = 0;
    if (!neg_x) empty_mask |= 0b000001;
    if (!pos_x) empty_mask |= 0b000010;
    if (!neg_y) empty_mask |= 0b000100;
    if (!pos_y) empty_mask |= 0b001000;
    if (!neg_z) empty_mask |= 0b010000;
    if (!pos_z) empty_mask |= 0b100000;

    return classifyVoxel(is_solid, empty_mask);
}

test "CollisionVoxel size is 1 byte" {
    try std.testing.expectEqual(@as(usize, 1), @sizeOf(CollisionVoxel));
}

test "CollisionVoxel empty" {
    const v = CollisionVoxel.empty;
    try std.testing.expect(v.isEmpty());
    try std.testing.expect(!v.isCollidable());
}

test "CollisionVoxel collision rules" {
    const corner = CollisionVoxel.init(.corner, 0);
    const edge = CollisionVoxel.init(.edge, 0);
    const face = CollisionVoxel.init(.face, 0);
    const inside = CollisionVoxel.init(.inside, 0);
    const empty_voxel = CollisionVoxel.empty;

    try std.testing.expect(corner.shouldCollide(corner));
    try std.testing.expect(corner.shouldCollide(edge));
    try std.testing.expect(corner.shouldCollide(face));
    try std.testing.expect(corner.shouldCollide(inside));

    try std.testing.expect(edge.shouldCollide(corner));
    try std.testing.expect(edge.shouldCollide(edge));
    try std.testing.expect(!edge.shouldCollide(face));
    try std.testing.expect(!edge.shouldCollide(inside));

    try std.testing.expect(face.shouldCollide(corner));
    try std.testing.expect(!face.shouldCollide(edge));
    try std.testing.expect(!face.shouldCollide(face));
    try std.testing.expect(!face.shouldCollide(inside));

    try std.testing.expect(!corner.shouldCollide(empty_voxel));
    try std.testing.expect(!empty_voxel.shouldCollide(corner));
}

test "classify voxel corner" {
    const v = classifyVoxel(true, 0b000111);
    try std.testing.expectEqual(VoxelType.corner, v.voxel_type);
    try std.testing.expect(v.isCollidable());
}

test "classify voxel edge" {
    const v = classifyVoxel(true, 0b000011);
    try std.testing.expectEqual(VoxelType.edge, v.voxel_type);
}

test "classify voxel face" {
    const v = classifyVoxel(true, 0b000001);
    try std.testing.expectEqual(VoxelType.face, v.voxel_type);
}

test "classify voxel inside" {
    const v = classifyVoxel(true, 0b000000);
    try std.testing.expectEqual(VoxelType.inside, v.voxel_type);
}

test "classify empty voxel" {
    const v = classifyVoxel(false, 0b111111);
    try std.testing.expectEqual(VoxelType.empty, v.voxel_type);
    try std.testing.expect(v.isEmpty());
}

test "normal LUT cardinal directions" {
    const pos_x = getNormalVec3(0);
    try std.testing.expectApproxEqAbs(@as(f32, 1), pos_x.x, math.EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 0), pos_x.y, math.EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 0), pos_x.z, math.EPSILON);

    const neg_y = getNormalVec3(3);
    try std.testing.expectApproxEqAbs(@as(f32, 0), neg_y.x, math.EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, -1), neg_y.y, math.EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 0), neg_y.z, math.EPSILON);
}

test "normal LUT edge directions are normalized" {
    const edge_normal = getNormalVec3(6);
    const len = edge_normal.length();
    try std.testing.expectApproxEqAbs(@as(f32, 1), len, math.EPSILON);
}

test "normal LUT corner directions are normalized" {
    const corner_normal = getNormalVec3(18);
    const len = corner_normal.length();
    try std.testing.expectApproxEqAbs(@as(f32, 1), len, math.EPSILON);
}

test "compute normal index for single face" {
    const idx = computeNormalIndex(0b000010);
    try std.testing.expectEqual(@as(u5, 0), idx);
}

test "compute normal index for edge" {
    const idx = computeNormalIndex(0b001010);
    try std.testing.expectEqual(@as(u5, 6), idx);
}

test "find closest normal index" {
    const idx = findClosestNormalIndex(Vec3.init(1, 0.1, -0.1));
    try std.testing.expectEqual(@as(u5, 0), idx);
}

const std = @import("std");

pub const PI: f32 = std.math.pi;
pub const EPSILON: f32 = 1e-6;

pub const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,

    pub const zero = Vec3{ .x = 0, .y = 0, .z = 0 };
    pub const one = Vec3{ .x = 1, .y = 1, .z = 1 };
    pub const unit_x = Vec3{ .x = 1, .y = 0, .z = 0 };
    pub const unit_y = Vec3{ .x = 0, .y = 1, .z = 0 };
    pub const unit_z = Vec3{ .x = 0, .y = 0, .z = 1 };

    pub fn init(x: f32, y: f32, z: f32) Vec3 {
        return .{ .x = x, .y = y, .z = z };
    }

    pub fn splat(v: f32) Vec3 {
        return .{ .x = v, .y = v, .z = v };
    }

    pub fn fromArray(arr: [3]f32) Vec3 {
        return .{ .x = arr[0], .y = arr[1], .z = arr[2] };
    }

    pub fn toArray(self: Vec3) [3]f32 {
        return .{ self.x, self.y, self.z };
    }

    pub fn add(self: Vec3, other: Vec3) Vec3 {
        return .{
            .x = self.x + other.x,
            .y = self.y + other.y,
            .z = self.z + other.z,
        };
    }

    pub fn sub(self: Vec3, other: Vec3) Vec3 {
        return .{
            .x = self.x - other.x,
            .y = self.y - other.y,
            .z = self.z - other.z,
        };
    }

    pub fn mul(self: Vec3, other: Vec3) Vec3 {
        return .{
            .x = self.x * other.x,
            .y = self.y * other.y,
            .z = self.z * other.z,
        };
    }

    pub fn div(self: Vec3, other: Vec3) Vec3 {
        return .{
            .x = self.x / other.x,
            .y = self.y / other.y,
            .z = self.z / other.z,
        };
    }

    pub fn scale(self: Vec3, s: f32) Vec3 {
        return .{
            .x = self.x * s,
            .y = self.y * s,
            .z = self.z * s,
        };
    }

    pub fn negate(self: Vec3) Vec3 {
        return .{
            .x = -self.x,
            .y = -self.y,
            .z = -self.z,
        };
    }

    pub fn dot(self: Vec3, other: Vec3) f32 {
        return self.x * other.x + self.y * other.y + self.z * other.z;
    }

    pub fn cross(self: Vec3, other: Vec3) Vec3 {
        return .{
            .x = self.y * other.z - self.z * other.y,
            .y = self.z * other.x - self.x * other.z,
            .z = self.x * other.y - self.y * other.x,
        };
    }

    pub fn lengthSquared(self: Vec3) f32 {
        return self.dot(self);
    }

    pub fn length(self: Vec3) f32 {
        return @sqrt(self.lengthSquared());
    }

    pub fn normalize(self: Vec3) Vec3 {
        const len = self.length();
        if (len < EPSILON) return Vec3.zero;
        return self.scale(1.0 / len);
    }

    pub fn normalizeOrZero(self: Vec3) Vec3 {
        const len_sq = self.lengthSquared();
        if (len_sq < EPSILON * EPSILON) return Vec3.zero;
        return self.scale(1.0 / @sqrt(len_sq));
    }

    pub fn distance(self: Vec3, other: Vec3) f32 {
        return self.sub(other).length();
    }

    pub fn distanceSquared(self: Vec3, other: Vec3) f32 {
        return self.sub(other).lengthSquared();
    }

    pub fn minComponents(self: Vec3, other: Vec3) Vec3 {
        return .{
            .x = @min(self.x, other.x),
            .y = @min(self.y, other.y),
            .z = @min(self.z, other.z),
        };
    }

    pub fn maxComponents(self: Vec3, other: Vec3) Vec3 {
        return .{
            .x = @max(self.x, other.x),
            .y = @max(self.y, other.y),
            .z = @max(self.z, other.z),
        };
    }

    pub fn clamp(self: Vec3, min_val: Vec3, max_val: Vec3) Vec3 {
        return .{
            .x = std.math.clamp(self.x, min_val.x, max_val.x),
            .y = std.math.clamp(self.y, min_val.y, max_val.y),
            .z = std.math.clamp(self.z, min_val.z, max_val.z),
        };
    }

    pub fn abs(self: Vec3) Vec3 {
        return .{
            .x = @abs(self.x),
            .y = @abs(self.y),
            .z = @abs(self.z),
        };
    }

    pub fn lerp(self: Vec3, other: Vec3, t: f32) Vec3 {
        return self.add(other.sub(self).scale(t));
    }

    pub fn eql(self: Vec3, other: Vec3) bool {
        return self.x == other.x and self.y == other.y and self.z == other.z;
    }

    pub fn approxEql(self: Vec3, other: Vec3, tolerance: f32) bool {
        return @abs(self.x - other.x) <= tolerance and
            @abs(self.y - other.y) <= tolerance and
            @abs(self.z - other.z) <= tolerance;
    }

    pub fn isZero(self: Vec3) bool {
        return self.lengthSquared() < EPSILON * EPSILON;
    }

    pub fn isFinite(self: Vec3) bool {
        return std.math.isFinite(self.x) and
            std.math.isFinite(self.y) and
            std.math.isFinite(self.z);
    }
};

pub const IVec3 = struct {
    x: i32,
    y: i32,
    z: i32,

    pub const zero = IVec3{ .x = 0, .y = 0, .z = 0 };

    pub fn init(x: i32, y: i32, z: i32) IVec3 {
        return .{ .x = x, .y = y, .z = z };
    }

    pub fn add(self: IVec3, other: IVec3) IVec3 {
        return .{
            .x = self.x + other.x,
            .y = self.y + other.y,
            .z = self.z + other.z,
        };
    }

    pub fn sub(self: IVec3, other: IVec3) IVec3 {
        return .{
            .x = self.x - other.x,
            .y = self.y - other.y,
            .z = self.z - other.z,
        };
    }

    pub fn toVec3(self: IVec3) Vec3 {
        return .{
            .x = @floatFromInt(self.x),
            .y = @floatFromInt(self.y),
            .z = @floatFromInt(self.z),
        };
    }

    pub fn eql(self: IVec3, other: IVec3) bool {
        return self.x == other.x and self.y == other.y and self.z == other.z;
    }
};

pub const Quat = struct {
    x: f32,
    y: f32,
    z: f32,
    w: f32,

    pub const identity = Quat{ .x = 0, .y = 0, .z = 0, .w = 1 };

    pub fn init(x: f32, y: f32, z: f32, w: f32) Quat {
        return .{ .x = x, .y = y, .z = z, .w = w };
    }

    pub fn fromAxisAngle(axis: Vec3, angle: f32) Quat {
        const half_angle = angle * 0.5;
        const s = @sin(half_angle);
        const normalized_axis = axis.normalize();
        return .{
            .x = normalized_axis.x * s,
            .y = normalized_axis.y * s,
            .z = normalized_axis.z * s,
            .w = @cos(half_angle),
        };
    }

    pub fn fromEuler(pitch: f32, yaw: f32, roll: f32) Quat {
        const cp = @cos(pitch * 0.5);
        const sp = @sin(pitch * 0.5);
        const cy = @cos(yaw * 0.5);
        const sy = @sin(yaw * 0.5);
        const cr = @cos(roll * 0.5);
        const sr = @sin(roll * 0.5);

        return .{
            .x = sp * cy * cr - cp * sy * sr,
            .y = cp * sy * cr + sp * cy * sr,
            .z = cp * cy * sr - sp * sy * cr,
            .w = cp * cy * cr + sp * sy * sr,
        };
    }

    pub fn mul(self: Quat, other: Quat) Quat {
        return .{
            .x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            .y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            .z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            .w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
        };
    }

    pub fn rotate(self: Quat, v: Vec3) Vec3 {
        const len_sq = self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w;
        const q = if (len_sq < 0.99 or len_sq > 1.01) self.normalize() else self;

        const qv = Vec3.init(q.x, q.y, q.z);
        const uv = qv.cross(v);
        const uuv = qv.cross(uv);
        return v.add(uv.scale(2.0 * q.w)).add(uuv.scale(2.0));
    }

    pub fn conjugate(self: Quat) Quat {
        return .{
            .x = -self.x,
            .y = -self.y,
            .z = -self.z,
            .w = self.w,
        };
    }

    pub fn inverse(self: Quat) Quat {
        const len_sq = self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w;
        if (len_sq < EPSILON) return Quat.identity;
        const inv_len_sq = 1.0 / len_sq;
        return .{
            .x = -self.x * inv_len_sq,
            .y = -self.y * inv_len_sq,
            .z = -self.z * inv_len_sq,
            .w = self.w * inv_len_sq,
        };
    }

    pub fn normalize(self: Quat) Quat {
        const len = @sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w);
        if (len < EPSILON) return Quat.identity;
        const inv_len = 1.0 / len;
        return .{
            .x = self.x * inv_len,
            .y = self.y * inv_len,
            .z = self.z * inv_len,
            .w = self.w * inv_len,
        };
    }

    pub fn slerp(self: Quat, other: Quat, t: f32) Quat {
        var dot_val = self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w;

        var other_adj = other;
        if (dot_val < 0) {
            other_adj = .{
                .x = -other.x,
                .y = -other.y,
                .z = -other.z,
                .w = -other.w,
            };
            dot_val = -dot_val;
        }

        if (dot_val > 0.9995) {
            const result = Quat{
                .x = self.x + (other_adj.x - self.x) * t,
                .y = self.y + (other_adj.y - self.y) * t,
                .z = self.z + (other_adj.z - self.z) * t,
                .w = self.w + (other_adj.w - self.w) * t,
            };
            return result.normalize();
        }

        const theta = std.math.acos(std.math.clamp(dot_val, -1.0, 1.0));
        const theta_t = theta * t;
        const sin_theta = @sin(theta);
        const sin_theta_t = @sin(theta_t);

        const s0 = @cos(theta_t) - dot_val * sin_theta_t / sin_theta;
        const s1 = sin_theta_t / sin_theta;

        return .{
            .x = self.x * s0 + other_adj.x * s1,
            .y = self.y * s0 + other_adj.y * s1,
            .z = self.z * s0 + other_adj.z * s1,
            .w = self.w * s0 + other_adj.w * s1,
        };
    }

    pub fn toMat3(self: Quat) Mat3 {
        const x2 = self.x + self.x;
        const y2 = self.y + self.y;
        const z2 = self.z + self.z;

        const xx = self.x * x2;
        const xy = self.x * y2;
        const xz = self.x * z2;
        const yy = self.y * y2;
        const yz = self.y * z2;
        const zz = self.z * z2;
        const wx = self.w * x2;
        const wy = self.w * y2;
        const wz = self.w * z2;

        return Mat3{
            .m = .{
                .{ 1.0 - (yy + zz), xy + wz, xz - wy },
                .{ xy - wz, 1.0 - (xx + zz), yz + wx },
                .{ xz + wy, yz - wx, 1.0 - (xx + yy) },
            },
        };
    }

    pub fn forward(self: Quat) Vec3 {
        return self.rotate(Vec3.init(0, 0, -1));
    }

    pub fn up(self: Quat) Vec3 {
        return self.rotate(Vec3.init(0, 1, 0));
    }

    pub fn right(self: Quat) Vec3 {
        return self.rotate(Vec3.init(1, 0, 0));
    }
};

pub const Mat3 = struct {
    m: [3][3]f32,

    pub const identity = Mat3{
        .m = .{
            .{ 1, 0, 0 },
            .{ 0, 1, 0 },
            .{ 0, 0, 1 },
        },
    };

    pub const zero_mat = Mat3{
        .m = .{
            .{ 0, 0, 0 },
            .{ 0, 0, 0 },
            .{ 0, 0, 0 },
        },
    };

    pub fn init(
        m00: f32,
        m01: f32,
        m02: f32,
        m10: f32,
        m11: f32,
        m12: f32,
        m20: f32,
        m21: f32,
        m22: f32,
    ) Mat3 {
        return .{
            .m = .{
                .{ m00, m01, m02 },
                .{ m10, m11, m12 },
                .{ m20, m21, m22 },
            },
        };
    }

    pub fn fromDiagonal(d: Vec3) Mat3 {
        return .{
            .m = .{
                .{ d.x, 0, 0 },
                .{ 0, d.y, 0 },
                .{ 0, 0, d.z },
            },
        };
    }

    pub fn fromScale(s: f32) Mat3 {
        return .{
            .m = .{
                .{ s, 0, 0 },
                .{ 0, s, 0 },
                .{ 0, 0, s },
            },
        };
    }

    pub fn mulVec(self: Mat3, v: Vec3) Vec3 {
        return .{
            .x = self.m[0][0] * v.x + self.m[0][1] * v.y + self.m[0][2] * v.z,
            .y = self.m[1][0] * v.x + self.m[1][1] * v.y + self.m[1][2] * v.z,
            .z = self.m[2][0] * v.x + self.m[2][1] * v.y + self.m[2][2] * v.z,
        };
    }

    pub fn mul(self: Mat3, other: Mat3) Mat3 {
        var result: Mat3 = undefined;
        for (0..3) |i| {
            for (0..3) |j| {
                result.m[i][j] = self.m[i][0] * other.m[0][j] +
                    self.m[i][1] * other.m[1][j] +
                    self.m[i][2] * other.m[2][j];
            }
        }
        return result;
    }

    pub fn transpose(self: Mat3) Mat3 {
        return .{
            .m = .{
                .{ self.m[0][0], self.m[1][0], self.m[2][0] },
                .{ self.m[0][1], self.m[1][1], self.m[2][1] },
                .{ self.m[0][2], self.m[1][2], self.m[2][2] },
            },
        };
    }

    pub fn determinant(self: Mat3) f32 {
        return self.m[0][0] * (self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1]) -
            self.m[0][1] * (self.m[1][0] * self.m[2][2] - self.m[1][2] * self.m[2][0]) +
            self.m[0][2] * (self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0]);
    }

    pub fn inverse(self: Mat3) Mat3 {
        const det = self.determinant();
        if (@abs(det) < EPSILON) return Mat3.identity;

        const inv_det = 1.0 / det;

        return .{
            .m = .{
                .{
                    (self.m[1][1] * self.m[2][2] - self.m[1][2] * self.m[2][1]) * inv_det,
                    (self.m[0][2] * self.m[2][1] - self.m[0][1] * self.m[2][2]) * inv_det,
                    (self.m[0][1] * self.m[1][2] - self.m[0][2] * self.m[1][1]) * inv_det,
                },
                .{
                    (self.m[1][2] * self.m[2][0] - self.m[1][0] * self.m[2][2]) * inv_det,
                    (self.m[0][0] * self.m[2][2] - self.m[0][2] * self.m[2][0]) * inv_det,
                    (self.m[0][2] * self.m[1][0] - self.m[0][0] * self.m[1][2]) * inv_det,
                },
                .{
                    (self.m[1][0] * self.m[2][1] - self.m[1][1] * self.m[2][0]) * inv_det,
                    (self.m[0][1] * self.m[2][0] - self.m[0][0] * self.m[2][1]) * inv_det,
                    (self.m[0][0] * self.m[1][1] - self.m[0][1] * self.m[1][0]) * inv_det,
                },
            },
        };
    }

    pub fn add(self: Mat3, other: Mat3) Mat3 {
        var result: Mat3 = undefined;
        for (0..3) |i| {
            for (0..3) |j| {
                result.m[i][j] = self.m[i][j] + other.m[i][j];
            }
        }
        return result;
    }

    pub fn scale(self: Mat3, s: f32) Mat3 {
        var result: Mat3 = undefined;
        for (0..3) |i| {
            for (0..3) |j| {
                result.m[i][j] = self.m[i][j] * s;
            }
        }
        return result;
    }
};

pub const Transform = struct {
    position: Vec3,
    rotation: Quat,

    pub const identity = Transform{
        .position = Vec3.zero,
        .rotation = Quat.identity,
    };

    pub fn init(position: Vec3, rotation: Quat) Transform {
        return .{ .position = position, .rotation = rotation };
    }

    pub fn transformPoint(self: Transform, local_point: Vec3) Vec3 {
        const normalized_rot = self.rotation.normalize();
        return normalized_rot.rotate(local_point).add(self.position);
    }

    pub fn transformDirection(self: Transform, local_dir: Vec3) Vec3 {
        return self.rotation.rotate(local_dir);
    }

    pub fn inverseTransformPoint(self: Transform, world_point: Vec3) Vec3 {
        return self.rotation.conjugate().rotate(world_point.sub(self.position));
    }

    pub fn inverseTransformDirection(self: Transform, world_dir: Vec3) Vec3 {
        return self.rotation.conjugate().rotate(world_dir);
    }

    pub fn mul(self: Transform, other: Transform) Transform {
        return .{
            .position = self.transformPoint(other.position),
            .rotation = self.rotation.mul(other.rotation),
        };
    }

    pub fn inverse(self: Transform) Transform {
        const inv_rot = self.rotation.conjugate();
        return .{
            .position = inv_rot.rotate(self.position.negate()),
            .rotation = inv_rot,
        };
    }

    pub fn lerp(self: Transform, other: Transform, t: f32) Transform {
        return .{
            .position = self.position.lerp(other.position, t),
            .rotation = self.rotation.slerp(other.rotation, t),
        };
    }
};

pub const AABB = struct {
    min: Vec3,
    max: Vec3,

    pub const empty = AABB{
        .min = Vec3.splat(std.math.inf(f32)),
        .max = Vec3.splat(-std.math.inf(f32)),
    };

    pub fn init(min: Vec3, max: Vec3) AABB {
        return .{ .min = min, .max = max };
    }

    pub fn fromCenterExtents(center_point: Vec3, half_extents: Vec3) AABB {
        return .{
            .min = center_point.sub(half_extents),
            .max = center_point.add(half_extents),
        };
    }

    pub fn fromPoints(points: []const Vec3) AABB {
        var result = AABB.empty;
        for (points) |p| {
            result = result.expand(p);
        }
        return result;
    }

    pub fn center(self: AABB) Vec3 {
        return self.min.add(self.max).scale(0.5);
    }

    pub fn extents(self: AABB) Vec3 {
        return self.max.sub(self.min).scale(0.5);
    }

    pub fn size(self: AABB) Vec3 {
        return self.max.sub(self.min);
    }

    pub fn volume(self: AABB) f32 {
        const s = self.size();
        return s.x * s.y * s.z;
    }

    pub fn surfaceArea(self: AABB) f32 {
        const s = self.size();
        return 2.0 * (s.x * s.y + s.y * s.z + s.z * s.x);
    }

    pub fn expand(self: AABB, point: Vec3) AABB {
        return .{
            .min = self.min.minComponents(point),
            .max = self.max.maxComponents(point),
        };
    }

    pub fn merge(self: AABB, other: AABB) AABB {
        return .{
            .min = self.min.minComponents(other.min),
            .max = self.max.maxComponents(other.max),
        };
    }

    pub fn inflate(self: AABB, margin: f32) AABB {
        const m = Vec3.splat(margin);
        return .{
            .min = self.min.sub(m),
            .max = self.max.add(m),
        };
    }

    pub fn overlaps(self: AABB, other: AABB) bool {
        return self.min.x <= other.max.x and self.max.x >= other.min.x and
            self.min.y <= other.max.y and self.max.y >= other.min.y and
            self.min.z <= other.max.z and self.max.z >= other.min.z;
    }

    pub fn contains(self: AABB, point: Vec3) bool {
        return point.x >= self.min.x and point.x <= self.max.x and
            point.y >= self.min.y and point.y <= self.max.y and
            point.z >= self.min.z and point.z <= self.max.z;
    }

    pub fn containsAABB(self: AABB, other: AABB) bool {
        return other.min.x >= self.min.x and other.max.x <= self.max.x and
            other.min.y >= self.min.y and other.max.y <= self.max.y and
            other.min.z >= self.min.z and other.max.z <= self.max.z;
    }

    pub fn getVertices(self: AABB) [8]Vec3 {
        return .{
            Vec3.init(self.min.x, self.min.y, self.min.z),
            Vec3.init(self.max.x, self.min.y, self.min.z),
            Vec3.init(self.min.x, self.max.y, self.min.z),
            Vec3.init(self.max.x, self.max.y, self.min.z),
            Vec3.init(self.min.x, self.min.y, self.max.z),
            Vec3.init(self.max.x, self.min.y, self.max.z),
            Vec3.init(self.min.x, self.max.y, self.max.z),
            Vec3.init(self.max.x, self.max.y, self.max.z),
        };
    }

    pub fn transform(self: AABB, t: Transform) AABB {
        const corners = self.getVertices();
        var result = AABB.empty;
        for (corners) |corner| {
            result = result.expand(t.transformPoint(corner));
        }
        return result;
    }

    pub fn isValid(self: AABB) bool {
        return self.min.x <= self.max.x and
            self.min.y <= self.max.y and
            self.min.z <= self.max.z;
    }
};

pub const OBB = struct {
    center: Vec3,
    half_extents: Vec3,
    rotation: Quat,

    pub fn init(center: Vec3, half_extents: Vec3, rotation: Quat) OBB {
        return .{
            .center = center,
            .half_extents = half_extents,
            .rotation = rotation,
        };
    }

    pub fn fromAABB(aabb: AABB, t: Transform) OBB {
        return .{
            .center = t.transformPoint(aabb.center()),
            .half_extents = aabb.extents(),
            .rotation = t.rotation,
        };
    }

    pub fn getAxes(self: OBB) [3]Vec3 {
        return .{
            self.rotation.rotate(Vec3.unit_x),
            self.rotation.rotate(Vec3.unit_y),
            self.rotation.rotate(Vec3.unit_z),
        };
    }

    pub fn getVertices(self: OBB) [8]Vec3 {
        const axes = self.getAxes();
        const ex = axes[0].scale(self.half_extents.x);
        const ey = axes[1].scale(self.half_extents.y);
        const ez = axes[2].scale(self.half_extents.z);

        return .{
            self.center.sub(ex).sub(ey).sub(ez),
            self.center.add(ex).sub(ey).sub(ez),
            self.center.sub(ex).add(ey).sub(ez),
            self.center.add(ex).add(ey).sub(ez),
            self.center.sub(ex).sub(ey).add(ez),
            self.center.add(ex).sub(ey).add(ez),
            self.center.sub(ex).add(ey).add(ez),
            self.center.add(ex).add(ey).add(ez),
        };
    }

    pub fn getWorldAABB(self: OBB) AABB {
        const vertices = self.getVertices();
        return AABB.fromPoints(&vertices);
    }
};

pub fn integrateRotation(rotation: Quat, angular_velocity: Vec3, dt: f32) Quat {
    const omega = angular_velocity.scale(dt);
    const omega_len = omega.length();

    if (omega_len < EPSILON) {
        return rotation;
    }

    const half_angle = omega_len * 0.5;
    const axis = omega.scale(1.0 / omega_len);
    const delta = Quat.fromAxisAngle(axis, half_angle * 2.0);

    return delta.mul(rotation).normalize();
}

test "Vec3 basic operations" {
    const a = Vec3.init(1, 2, 3);
    const b = Vec3.init(4, 5, 6);

    const sum = a.add(b);
    try std.testing.expectApproxEqAbs(@as(f32, 5), sum.x, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 7), sum.y, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 9), sum.z, EPSILON);

    const diff = b.sub(a);
    try std.testing.expectApproxEqAbs(@as(f32, 3), diff.x, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 3), diff.y, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 3), diff.z, EPSILON);
}

test "Vec3 dot and cross" {
    const a = Vec3.init(1, 0, 0);
    const b = Vec3.init(0, 1, 0);

    try std.testing.expectApproxEqAbs(@as(f32, 0), a.dot(b), EPSILON);

    const cross = a.cross(b);
    try std.testing.expectApproxEqAbs(@as(f32, 0), cross.x, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 0), cross.y, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 1), cross.z, EPSILON);
}

test "Vec3 normalize" {
    const v = Vec3.init(3, 4, 0);
    const n = v.normalize();

    try std.testing.expectApproxEqAbs(@as(f32, 0.6), n.x, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), n.y, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 0), n.z, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 1), n.length(), EPSILON);
}

test "Quat identity rotation" {
    const v = Vec3.init(1, 2, 3);
    const rotated = Quat.identity.rotate(v);

    try std.testing.expectApproxEqAbs(v.x, rotated.x, EPSILON);
    try std.testing.expectApproxEqAbs(v.y, rotated.y, EPSILON);
    try std.testing.expectApproxEqAbs(v.z, rotated.z, EPSILON);
}

test "Quat 90 degree rotation" {
    const q = Quat.fromAxisAngle(Vec3.unit_z, PI / 2.0);
    const v = Vec3.init(1, 0, 0);
    const rotated = q.rotate(v);

    try std.testing.expectApproxEqAbs(@as(f32, 0), rotated.x, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 1), rotated.y, EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 0), rotated.z, EPSILON);
}

test "Mat3 identity" {
    const v = Vec3.init(1, 2, 3);
    const result = Mat3.identity.mulVec(v);

    try std.testing.expectApproxEqAbs(v.x, result.x, EPSILON);
    try std.testing.expectApproxEqAbs(v.y, result.y, EPSILON);
    try std.testing.expectApproxEqAbs(v.z, result.z, EPSILON);
}

test "Mat3 inverse" {
    const m = Mat3.init(1, 2, 3, 0, 1, 4, 5, 6, 0);
    const inv = m.inverse();
    const identity = m.mul(inv);

    for (0..3) |i| {
        for (0..3) |j| {
            const expected: f32 = if (i == j) 1.0 else 0.0;
            try std.testing.expectApproxEqAbs(expected, identity.m[i][j], 0.0001);
        }
    }
}

test "Transform point round trip" {
    const t = Transform.init(Vec3.init(10, 20, 30), Quat.fromAxisAngle(Vec3.unit_y, PI / 4.0));

    const local = Vec3.init(1, 2, 3);
    const world = t.transformPoint(local);
    const back = t.inverseTransformPoint(world);

    try std.testing.expectApproxEqAbs(local.x, back.x, 0.0001);
    try std.testing.expectApproxEqAbs(local.y, back.y, 0.0001);
    try std.testing.expectApproxEqAbs(local.z, back.z, 0.0001);
}

test "AABB overlap" {
    const a = AABB.init(Vec3.init(0, 0, 0), Vec3.init(2, 2, 2));
    const b = AABB.init(Vec3.init(1, 1, 1), Vec3.init(3, 3, 3));
    const c = AABB.init(Vec3.init(5, 5, 5), Vec3.init(6, 6, 6));

    try std.testing.expect(a.overlaps(b));
    try std.testing.expect(b.overlaps(a));
    try std.testing.expect(!a.overlaps(c));
    try std.testing.expect(!c.overlaps(a));
}

test "AABB contains point" {
    const aabb = AABB.init(Vec3.init(0, 0, 0), Vec3.init(2, 2, 2));

    try std.testing.expect(aabb.contains(Vec3.init(1, 1, 1)));
    try std.testing.expect(aabb.contains(Vec3.init(0, 0, 0)));
    try std.testing.expect(aabb.contains(Vec3.init(2, 2, 2)));
    try std.testing.expect(!aabb.contains(Vec3.init(3, 1, 1)));
}

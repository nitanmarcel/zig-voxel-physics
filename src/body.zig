const std = @import("std");
const math = @import("math.zig");
const collider_mod = @import("collider.zig");

const Vec3 = math.Vec3;
const Quat = math.Quat;
const Mat3 = math.Mat3;
const Transform = math.Transform;
const AABB = math.AABB;

const VoxelCollider = collider_mod.VoxelCollider;

/// Default linear damping
pub const DEFAULT_LINEAR_DAMPING: f32 = 0.3;

/// Default angular damping
pub const DEFAULT_ANGULAR_DAMPING: f32 = 0.5;

/// Minimum linear speed before a body can sleep
pub const SLEEP_LINEAR_THRESHOLD: f32 = 0.1;

/// Minimum angular speed before a body can sleep
pub const SLEEP_ANGULAR_THRESHOLD: f32 = 0.1;

/// Time a body must be still before sleeping
pub const SLEEP_TIME_THRESHOLD: f32 = 0.5;

/// Type of physics body
pub const BodyType = enum(u2) {
    /// Static body: never moves, infinite mass
    static = 0,

    /// Kinematic body: controlled by game logic, not physics
    kinematic = 1,

    /// Dynamic body: fully simulated by physics
    dynamic = 2,

    /// Check if this body type can move
    pub fn canMove(self: BodyType) bool {
        return self != .static;
    }

    /// Check if this body type is affected by forces
    pub fn isSimulated(self: BodyType) bool {
        return self == .dynamic;
    }
};

/// Hot data for physics bodies - accessed every frame
pub const BodyMotionState = struct {
    /// World position
    position: Vec3,

    /// Orientation quaternion
    rotation: Quat,

    /// Linear velocity
    linear_velocity: Vec3,

    /// Angular velocity
    angular_velocity: Vec3,

    pub const identity = BodyMotionState{
        .position = Vec3.zero,
        .rotation = Quat.identity,
        .linear_velocity = Vec3.zero,
        .angular_velocity = Vec3.zero,
    };

    /// Get the transform for this motion state
    pub fn getTransform(self: BodyMotionState) Transform {
        return Transform.init(self.position, self.rotation);
    }

    /// Set the transform from position and rotation
    pub fn setTransform(self: *BodyMotionState, position: Vec3, rotation: Quat) void {
        self.position = position;
        self.rotation = rotation;
    }

    /// Integrate position by velocity over dt
    pub fn integratePosition(self: *BodyMotionState, dt: f32) void {
        self.position = self.position.add(self.linear_velocity.scale(dt));
        self.rotation = math.integrateRotation(self.rotation, self.angular_velocity, dt);
    }

    /// Get velocity at a world point
    pub fn getPointVelocity(self: BodyMotionState, world_point: Vec3) Vec3 {
        const r = world_point.sub(self.position);
        return self.linear_velocity.add(self.angular_velocity.cross(r));
    }

    /// Check if the body is nearly at rest
    pub fn isNearlyAtRest(self: BodyMotionState) bool {
        const linear_speed = self.linear_velocity.length();
        const angular_speed = self.angular_velocity.length();
        return linear_speed < SLEEP_LINEAR_THRESHOLD and angular_speed < SLEEP_ANGULAR_THRESHOLD;
    }
};

/// Cold data for physics bodies
pub const BodyProperties = struct {
    /// Inverse mass
    inv_mass: f32,

    /// Inverse inertia tensor in local space
    inv_inertia_local: Mat3,

    /// Center of mass offset from body origin
    center_of_mass: Vec3,

    /// Friction coefficient
    friction: f32,

    /// Restitution/bounciness
    restitution: f32,

    /// Linear damping
    linear_damping: f32,

    /// Angular damping
    angular_damping: f32,

    pub const default = BodyProperties{
        .inv_mass = 1.0,
        .inv_inertia_local = Mat3.identity,
        .center_of_mass = Vec3.zero,
        .friction = 0.5,
        .restitution = 0.0,
        .linear_damping = DEFAULT_LINEAR_DAMPING,
        .angular_damping = DEFAULT_ANGULAR_DAMPING,
    };

    pub const static_props = BodyProperties{
        .inv_mass = 0.0,
        .inv_inertia_local = Mat3.zero_mat,
        .center_of_mass = Vec3.zero,
        .friction = 0.5,
        .restitution = 0.0,
        .linear_damping = 0.0,
        .angular_damping = 0.0,
    };

    /// Get mass
    pub fn getMass(self: BodyProperties) f32 {
        if (self.inv_mass == 0.0) return std.math.inf(f32);
        return 1.0 / self.inv_mass;
    }

    /// Set mass
    pub fn setMass(self: *BodyProperties, mass: f32) void {
        if (mass <= 0.0 or !std.math.isFinite(mass)) {
            self.inv_mass = 0.0;
        } else {
            self.inv_mass = 1.0 / mass;
        }
    }

    /// Get the inertia tensor in local space
    pub fn getInertiaLocal(self: BodyProperties) Mat3 {
        return self.inv_inertia_local.inverse();
    }

    /// Set inertia tensor from local-space tensor
    pub fn setInertiaLocal(self: *BodyProperties, inertia: Mat3) void {
        self.inv_inertia_local = inertia.inverse();
    }

    /// Check if this body has infinite mass
    pub fn hasInfiniteMass(self: BodyProperties) bool {
        return self.inv_mass == 0.0;
    }
};

/// A physics body for voxel objects
pub const VoxelBody = struct {
    /// Unique identifier
    id: u32,

    /// Body type
    body_type: BodyType,

    /// Motion state
    motion: BodyMotionState,

    /// Physical properties
    properties: BodyProperties,

    /// Reference to collision shape
    collider: ?*VoxelCollider,

    /// Whether the body is sleeping
    is_sleeping: bool,

    /// Time the body has been nearly at rest
    sleep_timer: f32,

    /// Island ID for constraint solving
    island_id: ?u32,

    /// User data pointer for game logic
    user_data: ?*anyopaque,

    /// Create a new dynamic body
    pub fn initDynamic(id: u32, position: Vec3, rotation: Quat) VoxelBody {
        return .{
            .id = id,
            .body_type = .dynamic,
            .motion = .{
                .position = position,
                .rotation = rotation,
                .linear_velocity = Vec3.zero,
                .angular_velocity = Vec3.zero,
            },
            .properties = BodyProperties.default,
            .collider = null,
            .is_sleeping = false,
            .sleep_timer = 0.0,
            .island_id = null,
            .user_data = null,
        };
    }

    /// Create a new static body
    pub fn initStatic(id: u32, position: Vec3, rotation: Quat) VoxelBody {
        return .{
            .id = id,
            .body_type = .static,
            .motion = .{
                .position = position,
                .rotation = rotation,
                .linear_velocity = Vec3.zero,
                .angular_velocity = Vec3.zero,
            },
            .properties = BodyProperties.static_props,
            .collider = null,
            .is_sleeping = true,
            .sleep_timer = 0.0,
            .island_id = null,
            .user_data = null,
        };
    }

    /// Create a new kinematic body
    pub fn initKinematic(id: u32, position: Vec3, rotation: Quat) VoxelBody {
        return .{
            .id = id,
            .body_type = .kinematic,
            .motion = .{
                .position = position,
                .rotation = rotation,
                .linear_velocity = Vec3.zero,
                .angular_velocity = Vec3.zero,
            },
            .properties = BodyProperties.static_props,
            .collider = null,
            .is_sleeping = false,
            .sleep_timer = 0.0,
            .island_id = null,
            .user_data = null,
        };
    }

    /// Get the body's transform
    pub fn getTransform(self: VoxelBody) Transform {
        return self.motion.getTransform();
    }

    /// Set the body's position and rotation
    pub fn setTransform(self: *VoxelBody, position: Vec3, rotation: Quat) void {
        self.motion.setTransform(position, rotation);
        self.wakeUp();
    }

    /// Get world position
    pub fn getPosition(self: VoxelBody) Vec3 {
        return self.motion.position;
    }

    /// Set world position
    pub fn setPosition(self: *VoxelBody, position: Vec3) void {
        self.motion.position = position;
        self.wakeUp();
    }

    /// Get rotation
    pub fn getRotation(self: VoxelBody) Quat {
        return self.motion.rotation;
    }

    /// Set rotation
    pub fn setRotation(self: *VoxelBody, rotation: Quat) void {
        self.motion.rotation = rotation;
        self.wakeUp();
    }

    /// Get linear velocity
    pub fn getLinearVelocity(self: VoxelBody) Vec3 {
        return self.motion.linear_velocity;
    }

    /// Set linear velocity
    pub fn setLinearVelocity(self: *VoxelBody, velocity: Vec3) void {
        if (self.body_type != .dynamic) return;
        self.motion.linear_velocity = velocity;
        self.wakeUp();
    }

    /// Get angular velocity
    pub fn getAngularVelocity(self: VoxelBody) Vec3 {
        return self.motion.angular_velocity;
    }

    /// Set angular velocity
    pub fn setAngularVelocity(self: *VoxelBody, velocity: Vec3) void {
        if (self.body_type != .dynamic) return;
        self.motion.angular_velocity = velocity;
        self.wakeUp();
    }

    /// Get velocity at a world point
    pub fn getPointVelocity(self: VoxelBody, world_point: Vec3) Vec3 {
        return self.motion.getPointVelocity(world_point);
    }

    /// Apply a force at the center of mass
    pub fn applyForce(self: *VoxelBody, force: Vec3) void {
        if (self.body_type != .dynamic or self.properties.inv_mass == 0) return;

        self.motion.linear_velocity = self.motion.linear_velocity.add(force.scale(self.properties.inv_mass));
        self.wakeUp();
    }

    /// Apply a force at a world position
    pub fn applyForceAtPoint(self: *VoxelBody, force: Vec3, world_point: Vec3) void {
        if (self.body_type != .dynamic) return;

        self.applyForce(force);

        const r = world_point.sub(self.motion.position);
        const torque = r.cross(force);
        self.applyTorque(torque);
    }

    /// Apply a torque
    pub fn applyTorque(self: *VoxelBody, torque: Vec3) void {
        if (self.body_type != .dynamic) return;

        const inv_inertia_world = self.getWorldInvInertia();
        const angular_accel = inv_inertia_world.mulVec(torque);
        self.motion.angular_velocity = self.motion.angular_velocity.add(angular_accel);
        self.wakeUp();
    }

    /// Apply an impulse at the center of mass
    pub fn applyImpulse(self: *VoxelBody, impulse: Vec3) void {
        if (self.body_type != .dynamic or self.properties.inv_mass == 0) return;

        self.motion.linear_velocity = self.motion.linear_velocity.add(impulse.scale(self.properties.inv_mass));
        self.wakeUp();
    }

    /// Apply an impulse at a world position
    pub fn applyImpulseAtPoint(self: *VoxelBody, impulse: Vec3, world_point: Vec3) void {
        if (self.body_type != .dynamic) return;

        self.applyImpulse(impulse);

        const r = world_point.sub(self.motion.position);
        const angular_impulse = r.cross(impulse);
        self.applyAngularImpulse(angular_impulse);
    }

    /// Apply an angular impulse
    pub fn applyAngularImpulse(self: *VoxelBody, impulse: Vec3) void {
        if (self.body_type != .dynamic) return;

        const inv_inertia_world = self.getWorldInvInertia();
        const delta_omega = inv_inertia_world.mulVec(impulse);
        self.motion.angular_velocity = self.motion.angular_velocity.add(delta_omega);
        self.wakeUp();
    }

    /// Get the inverse inertia tensor in world space
    pub fn getWorldInvInertia(self: VoxelBody) Mat3 {
        if (self.properties.inv_mass == 0) return Mat3.zero_mat;

        const rot_mat = self.motion.rotation.toMat3();
        const rot_mat_t = rot_mat.transpose();
        return rot_mat.mul(self.properties.inv_inertia_local).mul(rot_mat_t);
    }

    /// Get the inertia tensor in world space
    pub fn getWorldInertia(self: VoxelBody) Mat3 {
        return self.getWorldInvInertia().inverse();
    }

    /// Wake up the body
    pub fn wakeUp(self: *VoxelBody) void {
        if (self.body_type == .static) return;
        self.is_sleeping = false;
        self.sleep_timer = 0.0;
    }

    /// Put the body to sleep
    pub fn sleep(self: *VoxelBody) void {
        if (self.body_type == .static) return;
        self.is_sleeping = true;
        self.motion.linear_velocity = Vec3.zero;
        self.motion.angular_velocity = Vec3.zero;
    }

    /// Update sleep state based on motion
    pub fn updateSleepState(self: *VoxelBody, dt: f32) void {
        if (self.body_type != .dynamic) return;
        if (self.is_sleeping) return;

        if (self.motion.isNearlyAtRest()) {
            self.sleep_timer += dt;
            if (self.sleep_timer >= SLEEP_TIME_THRESHOLD) {
                self.sleep();
            }
        } else {
            self.sleep_timer = 0.0;
        }
    }

    /// Get the world-space AABB for this body
    pub fn getWorldAABB(self: VoxelBody) AABB {
        if (self.collider) |col| {
            return col.getWorldAABB(self.getTransform());
        }
        return AABB.init(self.motion.position, self.motion.position);
    }

    /// Check if this body can collide with another
    pub fn canCollideWith(self: VoxelBody, other: VoxelBody) bool {
        if (self.body_type == .static and other.body_type == .static) {
            return false;
        }
        if (self.collider == null or other.collider == null) {
            return false;
        }
        if (self.is_sleeping and other.is_sleeping) {
            return false;
        }
        return true;
    }
};

/// Result of mass property computation
pub const MassProperties = struct {
    mass: f32,
    inv_mass: f32,
    center_of_mass: Vec3,
    inertia: Mat3,
    inv_inertia: Mat3,
};

/// Compute mass properties from a collider
pub fn computeMassProperties(collider: *const VoxelCollider, density: f32) MassProperties {
    if (collider.solid_count == 0) {
        return .{
            .mass = std.math.inf(f32),
            .inv_mass = 0.0,
            .center_of_mass = Vec3.zero,
            .inertia = Mat3.identity,
            .inv_inertia = Mat3.zero_mat,
        };
    }

    const voxel_volume = collider.voxel_size * collider.voxel_size * collider.voxel_size;
    const voxel_mass = voxel_volume * density;
    const total_mass = voxel_mass * @as(f32, @floatFromInt(collider.solid_count));

    const center_of_mass = collider.getCenterOfMass();

    var inertia = Mat3.zero_mat;

    var iter = collider.bricks.valueIterator();
    while (iter.next()) |brick| {
        if (brick.is_empty) continue;

        for (0..collider_mod.BRICK_VOLUME) |i| {
            if (brick.voxels[i].voxel_type == .empty) continue;

            const coord = collider_mod.CollisionBrick.indexToCoord(i);
            const voxel_pos = math.IVec3.init(
                brick.brick_pos.x * @as(i32, collider_mod.BRICK_SIZE) + @as(i32, coord.x),
                brick.brick_pos.y * @as(i32, collider_mod.BRICK_SIZE) + @as(i32, coord.y),
                brick.brick_pos.z * @as(i32, collider_mod.BRICK_SIZE) + @as(i32, coord.z),
            );

            const pos = collider.voxelToLocal(voxel_pos).add(Vec3.splat(collider.voxel_size * 0.5));
            const r = pos.sub(center_of_mass);
            const r2 = r.dot(r);

            inertia.m[0][0] += voxel_mass * (r2 - r.x * r.x);
            inertia.m[1][1] += voxel_mass * (r2 - r.y * r.y);
            inertia.m[2][2] += voxel_mass * (r2 - r.z * r.z);
            inertia.m[0][1] -= voxel_mass * r.x * r.y;
            inertia.m[0][2] -= voxel_mass * r.x * r.z;
            inertia.m[1][2] -= voxel_mass * r.y * r.z;
        }
    }

    inertia.m[1][0] = inertia.m[0][1];
    inertia.m[2][0] = inertia.m[0][2];
    inertia.m[2][1] = inertia.m[1][2];

    return .{
        .mass = total_mass,
        .inv_mass = 1.0 / total_mass,
        .center_of_mass = center_of_mass,
        .inertia = inertia,
        .inv_inertia = inertia.inverse(),
    };
}

test "BodyType properties" {
    try std.testing.expect(!BodyType.static.canMove());
    try std.testing.expect(BodyType.kinematic.canMove());
    try std.testing.expect(BodyType.dynamic.canMove());

    try std.testing.expect(!BodyType.static.isSimulated());
    try std.testing.expect(!BodyType.kinematic.isSimulated());
    try std.testing.expect(BodyType.dynamic.isSimulated());
}

test "BodyMotionState integration" {
    var state = BodyMotionState.identity;
    state.linear_velocity = Vec3.init(1, 0, 0);

    state.integratePosition(1.0);

    try std.testing.expectApproxEqAbs(@as(f32, 1), state.position.x, math.EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 0), state.position.y, math.EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 0), state.position.z, math.EPSILON);
}

test "VoxelBody init dynamic" {
    const body = VoxelBody.initDynamic(1, Vec3.init(1, 2, 3), Quat.identity);

    try std.testing.expectEqual(@as(u32, 1), body.id);
    try std.testing.expectEqual(BodyType.dynamic, body.body_type);
    try std.testing.expectApproxEqAbs(@as(f32, 1), body.motion.position.x, math.EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 2), body.motion.position.y, math.EPSILON);
    try std.testing.expectApproxEqAbs(@as(f32, 3), body.motion.position.z, math.EPSILON);
    try std.testing.expect(!body.is_sleeping);
}

test "VoxelBody init static" {
    const body = VoxelBody.initStatic(2, Vec3.zero, Quat.identity);

    try std.testing.expectEqual(BodyType.static, body.body_type);
    try std.testing.expectEqual(@as(f32, 0), body.properties.inv_mass);
    try std.testing.expect(body.is_sleeping);
}

test "VoxelBody apply impulse" {
    var body = VoxelBody.initDynamic(1, Vec3.zero, Quat.identity);
    body.properties.inv_mass = 0.5;

    body.applyImpulse(Vec3.init(2, 0, 0));

    try std.testing.expectApproxEqAbs(@as(f32, 1), body.motion.linear_velocity.x, math.EPSILON);
}

test "VoxelBody sleep management" {
    var body = VoxelBody.initDynamic(1, Vec3.zero, Quat.identity);

    try std.testing.expect(!body.is_sleeping);

    body.motion.linear_velocity = Vec3.zero;
    body.motion.angular_velocity = Vec3.zero;

    for (0..60) |_| {
        body.updateSleepState(1.0 / 60.0);
    }

    try std.testing.expect(body.is_sleeping);

    body.wakeUp();
    try std.testing.expect(!body.is_sleeping);
}

test "VoxelBody collision check" {
    const static_a = VoxelBody.initStatic(1, Vec3.zero, Quat.identity);
    const static_b = VoxelBody.initStatic(2, Vec3.init(1, 0, 0), Quat.identity);
    const dynamic_c = VoxelBody.initDynamic(3, Vec3.init(2, 0, 0), Quat.identity);

    try std.testing.expect(!static_a.canCollideWith(static_b));

    try std.testing.expect(!static_a.canCollideWith(dynamic_c));
}

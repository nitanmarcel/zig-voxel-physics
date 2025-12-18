const std = @import("std");
const math = @import("math.zig");
const narrow_phase = @import("narrow_phase.zig");
const body_mod = @import("body.zig");

const DEBUG_SOLVER = false;
var debug_frame_count: u32 = 0;

const Vec3 = math.Vec3;
const Quat = math.Quat;
const Mat3 = math.Mat3;
const Transform = math.Transform;
const EPSILON = math.EPSILON;
const PI = math.PI;

const ContactPoint = narrow_phase.ContactPoint;
const ContactManifold = narrow_phase.ContactManifold;
const ContactManager = narrow_phase.ContactManager;

const VoxelBody = body_mod.VoxelBody;
const BodyType = body_mod.BodyType;

/// Maximum number of velocity iterations per substep
pub const MAX_VELOCITY_ITERATIONS: u32 = 8;

/// Maximum number of position iterations
pub const MAX_POSITION_ITERATIONS: u32 = 4;

/// Default sub-steps per physics step
pub const DEFAULT_SUBSTEPS: u32 = 4;

/// Minimum separation for position correction
pub const MIN_SEPARATION: f32 = -0.02;

/// Velocity threshold for restitution
pub const RESTITUTION_THRESHOLD: f32 = 1.0;

/// Parameters for soft constraint computation
pub const SoftParams = struct {
    /// Bias rate for penetration correction
    bias_rate: f32,

    /// Mass coefficient for soft constraints
    mass_coeff: f32,

    /// Impulse coefficient for accumulated impulse
    impulse_coeff: f32,

    pub const default = SoftParams{
        .bias_rate = 0.0,
        .mass_coeff = 1.0,
        .impulse_coeff = 1.0,
    };
};

/// Compute soft constraint parameters from spring parameters
pub fn computeSoftParams(hertz: f32, damping_ratio: f32, dt: f32) SoftParams {
    if (hertz <= 0.0) {
        return SoftParams.default;
    }

    const omega = 2.0 * PI * hertz;
    const a1 = 2.0 * damping_ratio + omega * dt;
    const a2 = dt * omega * a1;
    const a3 = 1.0 / (1.0 + a2);

    return .{
        .bias_rate = omega / a1,
        .mass_coeff = a2 * a3,
        .impulse_coeff = a3,
    };
}

/// Configuration for the contact solver
pub const SolverConfig = struct {
    /// Number of sub-steps per physics step
    substeps: u32 = DEFAULT_SUBSTEPS,

    /// Number of velocity iterations per substep
    velocity_iterations: u32 = 1,

    /// Number of position iterations
    position_iterations: u32 = 2,

    /// Debug: Disable angular velocity
    disable_rotation: bool = false,

    /// Soft constraint natural frequency
    hertz: f32 = 30.0,

    /// Soft constraint damping ratio
    damping_ratio: f32 = 1.0,

    /// Position correction rate
    position_correction_rate: f32 = 0.2,

    /// Maximum position correction per iteration
    max_position_correction: f32 = 0.04,

    /// Allowed penetration  before correction kicks in
    slop: f32 = 0.005,

    /// Enable warm starting from previous frame impulses
    warm_starting: bool = true,

    /// Warm starting scale factor
    warm_start_scale: f32 = 0.8,

    /// Global gravity vector
    gravity: Vec3 = Vec3.init(0, -9.81, 0),

    /// Enable friction solving
    enable_friction: bool = true,

    /// Enable restitution
    enable_restitution: bool = true,

    pub const default = SolverConfig{};
};

/// Cached constraint data for a single contact point
const ConstraintPoint = struct {
    /// World-space contact point
    point: Vec3,

    /// Contact normal
    normal: Vec3,

    /// Local anchor on body A
    local_anchor_a: Vec3,

    /// Local anchor on body B
    local_anchor_b: Vec3,

    /// Offset from body A center to contact point
    r_a: Vec3,

    /// Offset from body B center to contact point
    r_b: Vec3,

    /// Base separation
    base_separation: f32,

    /// Effective mass for normal impulse
    normal_mass: f32,

    /// Effective mass for tangent impulses
    tangent_mass: [2]f32,

    /// Tangent directions for friction
    tangent1: Vec3,
    tangent2: Vec3,

    /// Velocity bias for penetration correction
    velocity_bias: f32,

    /// Accumulated normal impulse
    normal_impulse: f32,

    /// Accumulated tangent impulses
    tangent_impulse: [2]f32,

    /// Initial relative velocity along normal
    initial_normal_velocity: f32,

    /// Current penetration depth
    penetration: f32,
};

/// Cached constraint data for a manifold
const ConstraintManifold = struct {
    /// Body A index in the body array
    body_a_idx: usize,

    /// Body B index in the body array
    body_b_idx: usize,

    /// Combined friction coefficient
    friction: f32,

    /// Combined restitution coefficient
    restitution: f32,

    /// Number of active contact points
    point_count: u32,

    /// Constraint points
    points: [8]ConstraintPoint,

    pub fn init() ConstraintManifold {
        return .{
            .body_a_idx = 0,
            .body_b_idx = 0,
            .friction = 0.5,
            .restitution = 0.0,
            .point_count = 0,
            .points = undefined,
        };
    }
};

/// Cached body state for solver
const BodyState = struct {
    /// Position
    position: Vec3,

    /// Rotation
    rotation: Quat,

    /// Linear velocity
    linear_velocity: Vec3,

    /// Angular velocity
    angular_velocity: Vec3,

    /// Inverse mass
    inv_mass: f32,

    /// Inverse inertia in world space
    inv_inertia_world: Mat3,

    /// Body type
    body_type: BodyType,

    /// Whether the body is sleeping
    is_sleeping: bool,

    /// Original body pointer for write-back
    body_ptr: *VoxelBody,

    pub fn fromBody(body: *VoxelBody) BodyState {
        return .{
            .position = body.motion.position,
            .rotation = body.motion.rotation,
            .linear_velocity = body.motion.linear_velocity,
            .angular_velocity = body.motion.angular_velocity,
            .inv_mass = body.properties.inv_mass,
            .inv_inertia_world = body.getWorldInvInertia(),
            .body_type = body.body_type,
            .is_sleeping = body.is_sleeping,
            .body_ptr = body,
        };
    }

    /// Write state back to the original body
    pub fn writeBack(self: *BodyState) void {
        if (self.body_type == .static) return;

        if (!isFiniteVec3(self.position) or !isFiniteQuat(self.rotation)) {
            std.debug.print("WARNING: Body state corrupted (NaN/Inf detected), resetting to safe values\n", .{});

            self.position = if (isFiniteVec3(self.body_ptr.motion.position))
                self.body_ptr.motion.position
            else
                Vec3.init(0, 2, 0);
            self.rotation = Quat.identity;
            self.linear_velocity = Vec3.zero;
            self.angular_velocity = Vec3.zero;
        }

        if (!isFiniteVec3(self.linear_velocity)) {
            self.linear_velocity = Vec3.zero;
        }
        if (!isFiniteVec3(self.angular_velocity)) {
            self.angular_velocity = Vec3.zero;
        }

        const max_linear_speed: f32 = 50.0;
        const max_angular_speed: f32 = 30.0;
        const linear_speed_sq = self.linear_velocity.lengthSquared();
        const angular_speed_sq = self.angular_velocity.lengthSquared();

        if (linear_speed_sq > max_linear_speed * max_linear_speed) {
            self.linear_velocity = self.linear_velocity.scale(max_linear_speed / @sqrt(linear_speed_sq));
        }
        if (angular_speed_sq > max_angular_speed * max_angular_speed) {
            self.angular_velocity = self.angular_velocity.scale(max_angular_speed / @sqrt(angular_speed_sq));
        }

        const linear_threshold_sq: f32 = 0.002;
        const angular_threshold_sq: f32 = 0.002;

        if (linear_speed_sq < linear_threshold_sq) {
            self.linear_velocity = Vec3.zero;
        }
        if (angular_speed_sq < angular_threshold_sq) {
            self.angular_velocity = Vec3.zero;
        }

        self.rotation = self.rotation.normalize();

        self.body_ptr.motion.position = self.position;
        self.body_ptr.motion.rotation = self.rotation;
        self.body_ptr.motion.linear_velocity = self.linear_velocity;
        self.body_ptr.motion.angular_velocity = self.angular_velocity;
    }

    /// Check if a Vec3 contains only finite values
    fn isFiniteVec3(v: Vec3) bool {
        return std.math.isFinite(v.x) and std.math.isFinite(v.y) and std.math.isFinite(v.z);
    }

    /// Check if a Quat contains only finite values
    fn isFiniteQuat(q: Quat) bool {
        return std.math.isFinite(q.x) and std.math.isFinite(q.y) and std.math.isFinite(q.z) and std.math.isFinite(q.w);
    }

    /// Get velocity at a world point
    pub fn getPointVelocity(self: BodyState, world_point: Vec3) Vec3 {
        const r = world_point.sub(self.position);
        return self.linear_velocity.add(self.angular_velocity.cross(r));
    }

    /// Apply linear impulse
    pub fn applyLinearImpulse(self: *BodyState, impulse: Vec3) void {
        if (self.body_type != .dynamic or self.is_sleeping) return;
        self.linear_velocity = self.linear_velocity.add(impulse.scale(self.inv_mass));
    }

    /// Apply angular impulse
    pub fn applyAngularImpulse(self: *BodyState, impulse: Vec3) void {
        if (self.body_type != .dynamic or self.is_sleeping) return;
        self.angular_velocity = self.angular_velocity.add(self.inv_inertia_world.mulVec(impulse));
    }

    /// Apply impulse at a point
    pub fn applyImpulseAtPoint(self: *BodyState, impulse: Vec3, r: Vec3) void {
        if (self.body_type != .dynamic or self.is_sleeping) return;
        self.linear_velocity = self.linear_velocity.add(impulse.scale(self.inv_mass));

        self.angular_velocity = self.angular_velocity.add(self.inv_inertia_world.mulVec(r.cross(impulse)));
    }

    /// Apply impulse at a point
    pub fn applyLinearImpulseAtPoint(self: *BodyState, impulse: Vec3) void {
        if (self.body_type != .dynamic or self.is_sleeping) return;
        self.linear_velocity = self.linear_velocity.add(impulse.scale(self.inv_mass));
    }

    /// Integrate position by velocity
    pub fn integratePosition(self: *BodyState, dt: f32, disable_rotation: bool) void {
        if (self.body_type == .static) return;
        if (self.is_sleeping) return;

        self.position = self.position.add(self.linear_velocity.scale(dt));
        if (!disable_rotation) {
            self.rotation = math.integrateRotation(self.rotation, self.angular_velocity, dt);
        }

        if (self.body_type == .dynamic) {
            const rot_mat = self.rotation.toMat3();
            const rot_mat_t = rot_mat.transpose();

            self.inv_inertia_world = rot_mat.mul(self.body_ptr.properties.inv_inertia_local).mul(rot_mat_t);
        }
    }
};

/// TGS Soft Contact Solver
pub const ContactSolver = struct {
    allocator: std.mem.Allocator,

    /// Solver configuration
    config: SolverConfig,

    /// Cached body states
    body_states: std.array_list.AlignedManaged(BodyState, null),

    /// Body ID to index mapping
    body_id_to_index: std.AutoHashMap(u32, usize),

    /// Cached constraint manifolds
    constraints: std.array_list.AlignedManaged(ConstraintManifold, null),

    /// Reference to contact manager for impulse storage
    contact_manager: ?*ContactManager,

    /// Statistics
    stats: SolverStats,

    pub const SolverStats = struct {
        substeps_run: u32 = 0,
        velocity_iterations_run: u32 = 0,
        position_iterations_run: u32 = 0,
        constraints_solved: u32 = 0,
        max_penetration: f32 = 0,
        total_impulse: f32 = 0,

        pub fn reset(self: *SolverStats) void {
            self.* = .{};
        }
    };

    /// Initialize the solver
    pub fn init(allocator: std.mem.Allocator, config: SolverConfig) ContactSolver {
        return .{
            .allocator = allocator,
            .config = config,
            .body_states = std.array_list.AlignedManaged(BodyState, null).init(allocator),
            .body_id_to_index = std.AutoHashMap(u32, usize).init(allocator),
            .constraints = std.array_list.AlignedManaged(ConstraintManifold, null).init(allocator),
            .contact_manager = null,
            .stats = .{},
        };
    }

    /// Deinitialize the solver
    pub fn deinit(self: *ContactSolver) void {
        self.body_states.deinit();
        self.body_id_to_index.deinit();
        self.constraints.deinit();
    }

    /// Clear solver state for a new frame
    pub fn clear(self: *ContactSolver) void {
        self.body_states.clearRetainingCapacity();
        self.body_id_to_index.clearRetainingCapacity();
        self.constraints.clearRetainingCapacity();
        self.contact_manager = null;
        self.stats.reset();
    }

    /// Set the contact manager reference for impulse storage
    pub fn setContactManager(self: *ContactSolver, manager: *ContactManager) void {
        self.contact_manager = manager;
    }

    /// Add a body to the solver
    pub fn addBody(self: *ContactSolver, body: *VoxelBody) !void {
        if (self.body_id_to_index.contains(body.id)) return;

        const index = self.body_states.items.len;
        try self.body_states.append(BodyState.fromBody(body));
        try self.body_id_to_index.put(body.id, index);
    }

    /// Get body state by ID
    fn getBodyState(self: *ContactSolver, body_id: u32) ?*BodyState {
        const index = self.body_id_to_index.get(body_id) orelse return null;
        return &self.body_states.items[index];
    }

    /// Get body index by ID
    fn getBodyIndex(self: *ContactSolver, body_id: u32) ?usize {
        return self.body_id_to_index.get(body_id);
    }

    /// Add a contact manifold to solve
    pub fn addManifold(self: *ContactSolver, manifold: *const ContactManifold) !void {
        const idx_a = self.getBodyIndex(manifold.body_a_id) orelse return;
        const idx_b = self.getBodyIndex(manifold.body_b_id) orelse return;

        const state_a = &self.body_states.items[idx_a];
        const state_b = &self.body_states.items[idx_b];

        var cm = ConstraintManifold.init();
        cm.body_a_idx = idx_a;
        cm.body_b_idx = idx_b;
        cm.friction = manifold.friction;
        cm.restitution = manifold.restitution;
        cm.point_count = @intCast(manifold.contacts.len);

        for (manifold.contacts.constSlice(), 0..) |contact, i| {
            const world_anchor_a = state_a.rotation.rotate(contact.local_anchor_a).add(state_a.position);
            const world_anchor_b = state_b.rotation.rotate(contact.local_anchor_b).add(state_b.position);

            const anchor_diff = world_anchor_b.sub(world_anchor_a);
            const base_separation = -contact.penetration - anchor_diff.dot(contact.normal);

            cm.points[i] = .{
                .point = contact.point,
                .normal = contact.normal,
                .local_anchor_a = contact.local_anchor_a,
                .local_anchor_b = contact.local_anchor_b,
                .r_a = world_anchor_a.sub(state_a.position),
                .r_b = world_anchor_b.sub(state_b.position),
                .base_separation = base_separation,
                .normal_mass = 0,
                .tangent_mass = .{ 0, 0 },
                .tangent1 = Vec3.zero,
                .tangent2 = Vec3.zero,
                .velocity_bias = 0,
                .normal_impulse = contact.normal_impulse,
                .tangent_impulse = contact.tangent_impulse,
                .initial_normal_velocity = 0,
                .penetration = contact.penetration,
            };
        }

        try self.constraints.append(cm);
    }

    /// Main solve function - call this once per physics step
    pub fn solve(self: *ContactSolver, dt: f32) void {
        if (dt <= 0) return;
        if (self.body_states.items.len == 0) return;

        if (DEBUG_SOLVER) {
            debug_frame_count += 1;
            if (debug_frame_count % 60 == 0) {
                std.debug.print("\n=== Solver Frame {d}: dt={d:.6}, bodies={d}, constraints={d} ===\n", .{
                    debug_frame_count,
                    dt,
                    self.body_states.items.len,
                    self.constraints.items.len,
                });
            }
        }

        const has_constraints = self.constraints.items.len > 0;

        const substep_dt = dt / @as(f32, @floatFromInt(self.config.substeps));

        const soft_params = computeSoftParams(
            self.config.hertz,
            self.config.damping_ratio,
            substep_dt,
        );

        var substep: u32 = 0;
        while (substep < self.config.substeps) : (substep += 1) {
            self.stats.substeps_run += 1;

            self.applyGravity(substep_dt);

            if (has_constraints) {
                self.prepareConstraints(soft_params, substep_dt);

                if (self.config.warm_starting and substep == 0) {
                    self.warmStart();
                }

                var vel_iter: u32 = 0;
                while (vel_iter < self.config.velocity_iterations) : (vel_iter += 1) {
                    self.stats.velocity_iterations_run += 1;
                    self.solveVelocityConstraints(soft_params, true);
                }
            }

            self.integratePositions(substep_dt);

            if (has_constraints) {
                self.relaxConstraints();
            }
        }

        if (has_constraints) {
            var pos_iter: u32 = 0;
            while (pos_iter < self.config.position_iterations) : (pos_iter += 1) {
                self.stats.position_iterations_run += 1;
                const done = self.solvePositionConstraints();
                if (done) break;
            }
        }

        self.writeBackBodies();

        if (has_constraints) {
            self.storeImpulses();
        }
    }

    /// Apply gravity and damping to all dynamic bodies
    fn applyGravity(self: *ContactSolver, dt: f32) void {
        const gravity = self.config.gravity;

        const near_rest_threshold: f32 = 0.5;
        const near_rest_damping: f32 = 5.0;

        for (self.body_states.items) |*state| {
            if (state.body_type == .dynamic and !state.is_sleeping) {
                state.linear_velocity = state.linear_velocity.add(gravity.scale(dt));

                var linear_damping = state.body_ptr.properties.linear_damping;
                var angular_damping = state.body_ptr.properties.angular_damping;

                const linear_speed = state.linear_velocity.length();
                const angular_speed = state.angular_velocity.length();

                if (linear_speed < near_rest_threshold) {
                    const factor = 1.0 - (linear_speed / near_rest_threshold);
                    linear_damping += near_rest_damping * factor;
                }

                if (angular_speed < near_rest_threshold) {
                    const factor = 1.0 - (angular_speed / near_rest_threshold);
                    angular_damping += near_rest_damping * factor;
                }

                const linear_decay = @max(0.0, 1.0 - linear_damping * dt);
                const angular_decay = @max(0.0, 1.0 - angular_damping * dt);

                state.linear_velocity = state.linear_velocity.scale(linear_decay);
                state.angular_velocity = state.angular_velocity.scale(angular_decay);

                const velocity_epsilon: f32 = 0.001;
                if (state.linear_velocity.lengthSquared() < velocity_epsilon * velocity_epsilon) {
                    state.linear_velocity = Vec3.zero;
                }
                if (state.angular_velocity.lengthSquared() < velocity_epsilon * velocity_epsilon) {
                    state.angular_velocity = Vec3.zero;
                }
            }
        }
    }

    /// Prepare constraint data for solving
    fn prepareConstraints(self: *ContactSolver, soft_params: SoftParams, dt: f32) void {
        const dt_inv = if (dt > EPSILON) 1.0 / dt else 0.0;

        for (self.constraints.items) |*cm| {
            const state_a = &self.body_states.items[cm.body_a_idx];
            const state_b = &self.body_states.items[cm.body_b_idx];

            var i: u32 = 0;
            while (i < cm.point_count) : (i += 1) {
                var cp = &cm.points[i];

                const world_anchor_a = state_a.rotation.rotate(cp.local_anchor_a).add(state_a.position);
                const world_anchor_b = state_b.rotation.rotate(cp.local_anchor_b).add(state_b.position);

                cp.r_a = world_anchor_a.sub(state_a.position);
                cp.r_b = world_anchor_b.sub(state_b.position);

                cp.point = world_anchor_a.add(world_anchor_b).scale(0.5);

                const anchor_diff = world_anchor_b.sub(world_anchor_a);
                const current_separation = cp.base_separation + anchor_diff.dot(cp.normal);

                if (DEBUG_SOLVER and debug_frame_count % 60 == 0 and i == 0) {
                    std.debug.print("Constraint[{d}]: base_sep={d:.6}, anchor_diff.dot(n)={d:.6}, cur_sep={d:.6}, normal=({d:.3},{d:.3},{d:.3})\n", .{
                        cm.body_b_idx,
                        cp.base_separation,
                        anchor_diff.dot(cp.normal),
                        current_separation,
                        cp.normal.x,
                        cp.normal.y,
                        cp.normal.z,
                    });
                }

                const rn_a = cp.r_a.cross(cp.normal);
                const rn_b = cp.r_b.cross(cp.normal);

                var k_normal = state_a.inv_mass + state_b.inv_mass;
                k_normal += rn_a.dot(state_a.inv_inertia_world.mulVec(rn_a));
                k_normal += rn_b.dot(state_b.inv_inertia_world.mulVec(rn_b));

                cp.normal_mass = if (k_normal > EPSILON) 1.0 / k_normal else 0.0;

                const vel_a = state_a.getPointVelocity(cp.point);
                const vel_b = state_b.getPointVelocity(cp.point);
                const rel_vel = vel_b.sub(vel_a);

                const vn = rel_vel.dot(cp.normal);
                cp.initial_normal_velocity = vn;

                const max_pushout = 0.2;
                if (current_separation > 0.0) {
                    cp.velocity_bias = current_separation * dt_inv;
                    if (DEBUG_SOLVER and debug_frame_count % 60 == 0 and i == 0) {
                        std.debug.print("  -> SPECULATIVE: bias={d:.6} (sep={d:.6}, dt_inv={d:.3})\n", .{ cp.velocity_bias, current_separation, dt_inv });
                    }
                } else {
                    cp.velocity_bias = @max(soft_params.bias_rate * current_separation, -max_pushout);
                    if (DEBUG_SOLVER and debug_frame_count % 60 == 0 and i == 0) {
                        std.debug.print("  -> PENETRATING: bias={d:.6} (bias_rate={d:.3}, sep={d:.6})\n", .{ cp.velocity_bias, soft_params.bias_rate, current_separation });
                    }
                }

                if (self.config.enable_restitution and vn < -RESTITUTION_THRESHOLD) {
                    cp.velocity_bias = @max(cp.velocity_bias, -cm.restitution * vn);
                }

                if (self.config.enable_friction) {
                    cp.tangent1 = computeOrthogonal(cp.normal);
                    cp.tangent2 = cp.normal.cross(cp.tangent1);

                    const rt1_a = cp.r_a.cross(cp.tangent1);
                    const rt1_b = cp.r_b.cross(cp.tangent1);

                    var k_tangent1 = state_a.inv_mass + state_b.inv_mass;
                    k_tangent1 += rt1_a.dot(state_a.inv_inertia_world.mulVec(rt1_a));
                    k_tangent1 += rt1_b.dot(state_b.inv_inertia_world.mulVec(rt1_b));

                    cp.tangent_mass[0] = if (k_tangent1 > EPSILON) 1.0 / k_tangent1 else 0.0;

                    const rt2_a = cp.r_a.cross(cp.tangent2);
                    const rt2_b = cp.r_b.cross(cp.tangent2);

                    var k_tangent2 = state_a.inv_mass + state_b.inv_mass;
                    k_tangent2 += rt2_a.dot(state_a.inv_inertia_world.mulVec(rt2_a));
                    k_tangent2 += rt2_b.dot(state_b.inv_inertia_world.mulVec(rt2_b));

                    cp.tangent_mass[1] = if (k_tangent2 > EPSILON) 1.0 / k_tangent2 else 0.0;
                }
            }
        }
    }

    /// Apply warm starting impulses from previous frame
    fn warmStart(self: *ContactSolver) void {
        const scale = self.config.warm_start_scale;

        for (self.constraints.items) |*cm| {
            const state_a = &self.body_states.items[cm.body_a_idx];
            const state_b = &self.body_states.items[cm.body_b_idx];

            var i: u32 = 0;
            while (i < cm.point_count) : (i += 1) {
                const cp = &cm.points[i];

                const scaled_normal = cp.normal_impulse * scale;
                const scaled_tangent1 = cp.tangent_impulse[0] * scale;
                const scaled_tangent2 = cp.tangent_impulse[1] * scale;

                const impulse = cp.normal.scale(scaled_normal)
                    .add(cp.tangent1.scale(scaled_tangent1))
                    .add(cp.tangent2.scale(scaled_tangent2));

                state_a.applyImpulseAtPoint(impulse.negate(), cp.r_a);
                state_b.applyImpulseAtPoint(impulse, cp.r_b);

                cp.normal_impulse = scaled_normal;
                cp.tangent_impulse[0] = scaled_tangent1;
                cp.tangent_impulse[1] = scaled_tangent2;
            }
        }
    }

    /// Solve velocity constraints
    fn solveVelocityConstraints(self: *ContactSolver, soft_params: SoftParams, use_bias: bool) void {
        for (self.constraints.items) |*cm| {
            const state_a = &self.body_states.items[cm.body_a_idx];
            const state_b = &self.body_states.items[cm.body_b_idx];

            var i: u32 = 0;
            while (i < cm.point_count) : (i += 1) {
                const cp = &cm.points[i];
                self.stats.constraints_solved += 1;

                const vel_a = state_a.getPointVelocity(cp.point);
                const vel_b = state_b.getPointVelocity(cp.point);
                const rel_vel = vel_b.sub(vel_a);

                const vn = rel_vel.dot(cp.normal);

                var impulse: f32 = undefined;
                if (use_bias) {
                    impulse = -soft_params.mass_coeff * cp.normal_mass *
                        (vn + cp.velocity_bias) -
                        soft_params.impulse_coeff * cp.normal_impulse;
                } else {
                    impulse = -cp.normal_mass * vn;
                }

                const new_impulse = @max(cp.normal_impulse + impulse, 0);
                impulse = new_impulse - cp.normal_impulse;
                cp.normal_impulse = new_impulse;

                self.stats.total_impulse += @abs(impulse);

                const normal_impulse = cp.normal.scale(impulse);
                state_a.applyImpulseAtPoint(normal_impulse.negate(), cp.r_a);
                state_b.applyImpulseAtPoint(normal_impulse, cp.r_b);

                if (DEBUG_SOLVER and debug_frame_count % 60 == 0 and i == 0) {
                    std.debug.print("  Impulse: vn={d:.6}, bias={d:.6}, impulse={d:.6}, accum={d:.6}\n", .{
                        vn,
                        cp.velocity_bias,
                        impulse,
                        cp.normal_impulse,
                    });
                }

                if (self.config.enable_friction and cm.friction > 0) {
                    self.solveFriction(cm, cp, state_a, state_b);
                }
            }
        }
    }

    /// Solve friction constraint for a single contact point
    fn solveFriction(
        self: *ContactSolver,
        cm: *const ConstraintManifold,
        cp: *ConstraintPoint,
        state_a: *BodyState,
        state_b: *BodyState,
    ) void {
        const vel_a = state_a.getPointVelocity(cp.point);
        const vel_b = state_b.getPointVelocity(cp.point);
        const rel_vel = vel_b.sub(vel_a);

        const max_friction = cm.friction * cp.normal_impulse;

        {
            const vt1 = rel_vel.dot(cp.tangent1);
            var friction_impulse1 = -cp.tangent_mass[0] * vt1;

            const old_impulse1 = cp.tangent_impulse[0];
            cp.tangent_impulse[0] = std.math.clamp(
                old_impulse1 + friction_impulse1,
                -max_friction,
                max_friction,
            );
            friction_impulse1 = cp.tangent_impulse[0] - old_impulse1;

            self.stats.total_impulse += @abs(friction_impulse1);

            const impulse1 = cp.tangent1.scale(friction_impulse1);
            state_a.applyImpulseAtPoint(impulse1.negate(), cp.r_a);
            state_b.applyImpulseAtPoint(impulse1, cp.r_b);
        }

        {
            const vel_a2 = state_a.getPointVelocity(cp.point);
            const vel_b2 = state_b.getPointVelocity(cp.point);
            const rel_vel2 = vel_b2.sub(vel_a2);

            const vt2 = rel_vel2.dot(cp.tangent2);
            var friction_impulse2 = -cp.tangent_mass[1] * vt2;

            const old_impulse2 = cp.tangent_impulse[1];
            cp.tangent_impulse[1] = std.math.clamp(
                old_impulse2 + friction_impulse2,
                -max_friction,
                max_friction,
            );
            friction_impulse2 = cp.tangent_impulse[1] - old_impulse2;

            self.stats.total_impulse += @abs(friction_impulse2);

            const impulse2 = cp.tangent2.scale(friction_impulse2);
            state_a.applyImpulseAtPoint(impulse2.negate(), cp.r_a);
            state_b.applyImpulseAtPoint(impulse2, cp.r_b);
        }
    }

    /// Integrate positions using current velocities
    fn integratePositions(self: *ContactSolver, dt: f32) void {
        for (self.body_states.items) |*state| {
            state.integratePosition(dt, self.config.disable_rotation);
        }
    }

    /// Relax soft constraints
    fn relaxConstraints(self: *ContactSolver) void {
        const hard_params = SoftParams{
            .bias_rate = 0.0,
            .mass_coeff = 1.0,
            .impulse_coeff = 1.0,
        };

        self.solveVelocityConstraints(hard_params, false);
    }

    /// Solve position constraints
    /// Returns true if all constraints are satisfied within tolerance
    fn solvePositionConstraints(self: *ContactSolver) bool {
        var min_separation: f32 = 0;

        for (self.constraints.items) |*cm| {
            const state_a = &self.body_states.items[cm.body_a_idx];
            const state_b = &self.body_states.items[cm.body_b_idx];

            if (state_a.body_type == .static and state_b.body_type == .static) {
                continue;
            }

            var i: u32 = 0;
            while (i < cm.point_count) : (i += 1) {
                const cp = &cm.points[i];

                const world_a = state_a.rotation.rotate(cp.local_anchor_a).add(state_a.position);
                const world_b = state_b.rotation.rotate(cp.local_anchor_b).add(state_b.position);

                const anchor_diff = world_b.sub(world_a);
                const separation = cp.base_separation + anchor_diff.dot(cp.normal);

                min_separation = @min(min_separation, separation);
                self.stats.max_penetration = @max(self.stats.max_penetration, -separation);

                if (separation >= -self.config.slop) continue;

                const correction = @max(
                    self.config.position_correction_rate * (separation + self.config.slop),
                    -self.config.max_position_correction,
                );

                const r_a = world_a.sub(state_a.position);
                const r_b = world_b.sub(state_b.position);
                const rn_a = r_a.cross(cp.normal);
                const rn_b = r_b.cross(cp.normal);

                var k = state_a.inv_mass + state_b.inv_mass;
                k += rn_a.dot(state_a.inv_inertia_world.mulVec(rn_a));
                k += rn_b.dot(state_b.inv_inertia_world.mulVec(rn_b));

                if (k < EPSILON) continue;

                const position_impulse = cp.normal.scale(-correction / k);

                if (state_a.body_type == .dynamic and !state_a.is_sleeping) {
                    state_a.position = state_a.position.sub(position_impulse.scale(state_a.inv_mass));
                    state_a.rotation = integrateRotationSmall(
                        state_a.rotation,
                        state_a.inv_inertia_world.mulVec(r_a.cross(position_impulse.negate())),
                    );
                }

                if (state_b.body_type == .dynamic and !state_b.is_sleeping) {
                    state_b.position = state_b.position.add(position_impulse.scale(state_b.inv_mass));
                    state_b.rotation = integrateRotationSmall(
                        state_b.rotation,
                        state_b.inv_inertia_world.mulVec(r_b.cross(position_impulse)),
                    );
                }
            }
        }

        return min_separation >= -3.0 * self.config.slop;
    }

    /// Write body states back to the original bodies
    fn writeBackBodies(self: *ContactSolver) void {
        for (self.body_states.items) |*state| {
            state.writeBack();
        }
    }

    /// Store accumulated impulses back to contacts for warm starting next frame
    fn storeImpulses(self: *ContactSolver) void {
        const manager = self.contact_manager orelse return;

        for (self.constraints.items) |*cm| {
            const body_a_state = &self.body_states.items[cm.body_a_idx];
            const body_b_state = &self.body_states.items[cm.body_b_idx];

            const body_a_id = body_a_state.body_ptr.id;
            const body_b_id = body_b_state.body_ptr.id;

            if (manager.getManifoldMut(body_a_id, body_b_id)) |manifold| {
                const contact_count = @min(cm.point_count, manifold.contacts.len);
                var i: u32 = 0;
                while (i < contact_count) : (i += 1) {
                    const cp = &cm.points[i];
                    var contact = &manifold.contacts.buffer[i];

                    contact.normal_impulse = cp.normal_impulse;
                    contact.tangent_impulse = cp.tangent_impulse;

                    contact.normal_mass = cp.normal_mass;
                    contact.tangent_mass = cp.tangent_mass;
                }
            }
        }
    }

    /// Get solver statistics
    pub fn getStats(self: *const ContactSolver) SolverStats {
        return self.stats;
    }
};

/// Compute a vector orthogonal to the given vector
fn computeOrthogonal(v: Vec3) Vec3 {
    if (@abs(v.x) < @abs(v.y)) {
        if (@abs(v.x) < @abs(v.z)) {
            return Vec3.init(0, -v.z, v.y).normalizeOrZero();
        } else {
            return Vec3.init(-v.y, v.x, 0).normalizeOrZero();
        }
    } else {
        if (@abs(v.y) < @abs(v.z)) {
            return Vec3.init(-v.z, 0, v.x).normalizeOrZero();
        } else {
            return Vec3.init(-v.y, v.x, 0).normalizeOrZero();
        }
    }
}

/// Integrate rotation by a small angular displacement
fn integrateRotationSmall(rotation: Quat, angular_displacement: Vec3) Quat {
    const half_w = Quat.init(
        angular_displacement.x * 0.5,
        angular_displacement.y * 0.5,
        angular_displacement.z * 0.5,
        0,
    );

    const dq = Quat.init(
        half_w.w * rotation.x + half_w.x * rotation.w + half_w.y * rotation.z - half_w.z * rotation.y,
        half_w.w * rotation.y - half_w.x * rotation.z + half_w.y * rotation.w + half_w.z * rotation.x,
        half_w.w * rotation.z + half_w.x * rotation.y - half_w.y * rotation.x + half_w.z * rotation.w,
        half_w.w * rotation.w - half_w.x * rotation.x - half_w.y * rotation.y - half_w.z * rotation.z,
    );

    return Quat.init(
        rotation.x + dq.x,
        rotation.y + dq.y,
        rotation.z + dq.z,
        rotation.w + dq.w,
    ).normalize();
}

test "SoftParams computation" {
    const params = computeSoftParams(30.0, 1.0, 1.0 / 240.0);

    try std.testing.expect(params.bias_rate > 0);
    try std.testing.expect(params.mass_coeff > 0);
    try std.testing.expect(params.mass_coeff < 1);
    try std.testing.expect(params.impulse_coeff > 0);
    try std.testing.expect(params.impulse_coeff < 1);
}

test "SoftParams zero hertz" {
    const params = computeSoftParams(0.0, 1.0, 1.0 / 60.0);

    try std.testing.expectEqual(@as(f32, 0.0), params.bias_rate);
    try std.testing.expectEqual(@as(f32, 1.0), params.mass_coeff);
    try std.testing.expectEqual(@as(f32, 1.0), params.impulse_coeff);
}

test "computeOrthogonal" {
    const v1 = Vec3.init(1, 0, 0);
    const orth1 = computeOrthogonal(v1);
    try std.testing.expect(@abs(v1.dot(orth1)) < EPSILON);

    const v2 = Vec3.init(0, 1, 0);
    const orth2 = computeOrthogonal(v2);
    try std.testing.expect(@abs(v2.dot(orth2)) < EPSILON);

    const v3 = Vec3.init(0, 0, 1);
    const orth3 = computeOrthogonal(v3);
    try std.testing.expect(@abs(v3.dot(orth3)) < EPSILON);

    const v4 = Vec3.init(1, 1, 1).normalize();
    const orth4 = computeOrthogonal(v4);
    try std.testing.expect(@abs(v4.dot(orth4)) < EPSILON);
}

test "integrateRotationSmall identity" {
    const q = Quat.identity;
    const result = integrateRotationSmall(q, Vec3.zero);

    try std.testing.expect(@abs(result.x - q.x) < EPSILON);
    try std.testing.expect(@abs(result.y - q.y) < EPSILON);
    try std.testing.expect(@abs(result.z - q.z) < EPSILON);
    try std.testing.expect(@abs(result.w - q.w) < EPSILON);
}

test "ContactSolver init and deinit" {
    var solver = ContactSolver.init(std.testing.allocator, .{});
    defer solver.deinit();

    try std.testing.expectEqual(@as(usize, 0), solver.body_states.items.len);
    try std.testing.expectEqual(@as(usize, 0), solver.constraints.items.len);
}

test "SolverConfig defaults" {
    const config = SolverConfig.default;

    try std.testing.expectEqual(@as(u32, DEFAULT_SUBSTEPS), config.substeps);
    try std.testing.expectEqual(@as(u32, 1), config.velocity_iterations);
    try std.testing.expectEqual(@as(u32, 2), config.position_iterations);
    try std.testing.expectEqual(@as(f32, 30.0), config.hertz);
    try std.testing.expectEqual(@as(f32, 1.0), config.damping_ratio);
    try std.testing.expect(config.warm_starting);
    try std.testing.expect(config.enable_friction);
    try std.testing.expect(config.enable_restitution);
}

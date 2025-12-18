const std = @import("std");
const math = @import("math.zig");
const body_mod = @import("body.zig");
const broad_phase_mod = @import("broad_phase.zig");
const narrow_phase_mod = @import("narrow_phase.zig");
const solver_mod = @import("solver.zig");
const collider_mod = @import("collider.zig");
const simd_mod = @import("simd.zig");
const thread_pool_mod = @import("thread_pool.zig");
const islands_mod = @import("islands.zig");

const Vec3 = math.Vec3;
const Quat = math.Quat;
const AABB = math.AABB;
const Transform = math.Transform;

const VoxelBody = body_mod.VoxelBody;
const BodyType = body_mod.BodyType;
const MassProperties = body_mod.MassProperties;
const computeMassProperties = body_mod.computeMassProperties;

const BroadPhase = broad_phase_mod.BroadPhase;
const BodyPair = broad_phase_mod.BodyPair;

const NarrowPhase = narrow_phase_mod.NarrowPhase;
const ContactManifold = narrow_phase_mod.ContactManifold;
const ContactManager = narrow_phase_mod.ContactManager;

const ContactSolver = solver_mod.ContactSolver;
const SolverConfig = solver_mod.SolverConfig;

const VoxelCollider = collider_mod.VoxelCollider;

const AABBBoundsSOA = simd_mod.AABBBoundsSOA;
const ThreadPool = thread_pool_mod.ThreadPool;
const IslandManager = islands_mod.IslandManager;
const GraphColoring = islands_mod.GraphColoring;

/// Generate a unique key for a body pair
fn pairKey(body_a: u32, body_b: u32) u64 {
    const a = @min(body_a, body_b);
    const b = @max(body_a, body_b);
    return (@as(u64, a) << 32) | @as(u64, b);
}

/// Physics system configuration
pub const PhysicsConfig = struct {
    /// Gravity acceleration
    gravity: Vec3 = Vec3.init(0, -9.81, 0),

    /// Fixed timestep for physics simulation
    fixed_timestep: f32 = 1.0 / 60.0,

    /// Maximum substeps per frame to prevent spiral of death
    max_substeps: u32 = 8,

    /// Solver configuration
    solver: SolverConfig = .{},

    /// Maximum number of bodies
    max_bodies: u32 = 1024,

    /// Default material properties
    default_friction: f32 = 0.6,
    default_restitution: f32 = 0.2,
    default_density: f32 = 1.0,

    /// Enable sleeping optimization
    enable_sleeping: bool = true,

    /// Enable warm starting for contacts
    enable_warm_starting: bool = true,

    /// Enable multithreaded physics
    enable_threading: bool = false,

    /// Number of worker threads
    thread_count: usize = 0,

    /// Enable island-based solving for parallel constraint solving
    enable_islands: bool = true,

    /// Enable SIMD optimizations in broad phase
    enable_simd_broad_phase: bool = true,

    /// Minimum batch size for parallel processing
    min_parallel_batch_size: usize = 16,
};

/// Physics system statistics for profiling and debugging
pub const PhysicsStats = struct {
    total_bodies: u32 = 0,
    dynamic_bodies: u32 = 0,
    static_bodies: u32 = 0,
    sleeping_bodies: u32 = 0,
    active_bodies: u32 = 0,

    broad_phase_pairs: u32 = 0,
    narrow_phase_tested: u32 = 0,
    narrow_phase_colliding: u32 = 0,
    contacts_generated: u32 = 0,
    voxels_checked: u32 = 0,
    pairs_parallel: u32 = 0,
    pairs_sequential: u32 = 0,
    pairs_skipped_sleeping: u32 = 0,
    cache_hits: u32 = 0,
    cache_misses: u32 = 0,

    solver_iterations: u32 = 0,
    max_penetration: f32 = 0,
    manifolds_skipped_sleeping: u32 = 0,

    total_time_ms: f32 = 0,
    broad_phase_time_ms: f32 = 0,
    narrow_phase_time_ms: f32 = 0,
    solver_time_ms: f32 = 0,
    integration_time_ms: f32 = 0,

    island_count: u32 = 0,
    active_islands: u32 = 0,
    sleeping_islands: u32 = 0,
    largest_island_size: u32 = 0,
    threads_used: u32 = 0,
    island_build_time_ms: f32 = 0,

    substeps_this_frame: u32 = 0,
    accumulated_time: f32 = 0,

    pub fn reset(self: *PhysicsStats) void {
        self.broad_phase_pairs = 0;
        self.narrow_phase_tested = 0;
        self.narrow_phase_colliding = 0;
        self.contacts_generated = 0;
        self.voxels_checked = 0;
        self.pairs_parallel = 0;
        self.pairs_sequential = 0;
        self.pairs_skipped_sleeping = 0;
        self.cache_hits = 0;
        self.cache_misses = 0;
        self.solver_iterations = 0;
        self.max_penetration = 0;
        self.manifolds_skipped_sleeping = 0;
        self.total_time_ms = 0;
        self.broad_phase_time_ms = 0;
        self.narrow_phase_time_ms = 0;
        self.solver_time_ms = 0;
        self.integration_time_ms = 0;
        self.substeps_this_frame = 0;
        self.island_count = 0;
        self.active_islands = 0;
        self.sleeping_islands = 0;
        self.largest_island_size = 0;
        self.island_build_time_ms = 0;
    }
};

/// Handle to a physics body
pub const BodyHandle = struct {
    index: u32,
    generation: u32,

    pub const invalid = BodyHandle{ .index = std.math.maxInt(u32), .generation = 0 };

    pub fn isValid(self: BodyHandle) bool {
        return self.index != std.math.maxInt(u32);
    }
};

/// Internal body slot with generation tracking
const BodySlot = struct {
    body: VoxelBody,
    generation: u32,
    active: bool,
    owns_collider: bool,

    pub const empty = BodySlot{
        .body = undefined,
        .generation = 0,
        .active = false,
        .owns_collider = false,
    };
};

/// Unified physics system coordinating all physics components
pub const VoxelPhysicsSystem = struct {
    allocator: std.mem.Allocator,

    config: PhysicsConfig,

    bodies: []BodySlot,
    body_count: u32,
    next_body_id: u32,
    free_list: std.ArrayListUnmanaged(u32),

    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,

    solver: ContactSolver,

    thread_pool: ?*ThreadPool,
    island_manager: ?IslandManager,
    graph_coloring: ?GraphColoring,
    simd_aabb_cache: ?AABBBoundsSOA,

    stats: PhysicsStats,

    accumulated_time: f32,

    on_collision_start: ?*const fn (body_a: BodyHandle, body_b: BodyHandle, manifold: *const ContactManifold) void,
    on_collision_end: ?*const fn (body_a: BodyHandle, body_b: BodyHandle) void,

    /// Initialize the physics system
    pub fn init(allocator: std.mem.Allocator, config: PhysicsConfig) !VoxelPhysicsSystem {
        const bodies = try allocator.alloc(BodySlot, config.max_bodies);
        @memset(bodies, BodySlot.empty);

        var solver_config = config.solver;
        solver_config.gravity = config.gravity;

        var thread_pool: ?*ThreadPool = null;
        if (config.enable_threading) {
            thread_pool = ThreadPool.init(allocator, config.thread_count) catch null;
        }

        var island_manager: ?IslandManager = null;
        if (config.enable_islands) {
            island_manager = IslandManager.init(allocator, config.max_bodies) catch null;
        }

        var graph_coloring: ?GraphColoring = null;
        if (config.enable_threading) {
            graph_coloring = GraphColoring.init(allocator);
        }

        var simd_aabb_cache: ?AABBBoundsSOA = null;
        if (config.enable_simd_broad_phase) {
            simd_aabb_cache = AABBBoundsSOA.init(allocator);
        }

        return VoxelPhysicsSystem{
            .allocator = allocator,
            .config = config,
            .bodies = bodies,
            .body_count = 0,
            .next_body_id = 1,
            .free_list = .{},
            .broad_phase = BroadPhase.init(allocator),
            .narrow_phase = NarrowPhase.init(allocator, .{}),
            .solver = ContactSolver.init(allocator, solver_config),
            .thread_pool = thread_pool,
            .island_manager = island_manager,
            .graph_coloring = graph_coloring,
            .simd_aabb_cache = simd_aabb_cache,
            .stats = .{},
            .accumulated_time = 0,
            .on_collision_start = null,
            .on_collision_end = null,
        };
    }

    /// Deinitialize the physics system
    pub fn deinit(self: *VoxelPhysicsSystem) void {
        if (self.thread_pool) |pool| {
            pool.deinit();
        }
        if (self.island_manager) |*manager| {
            manager.deinit();
        }
        if (self.graph_coloring) |*coloring| {
            coloring.deinit();
        }
        if (self.simd_aabb_cache) |*cache| {
            cache.deinit();
        }

        for (self.bodies) |*slot| {
            if (slot.active and slot.owns_collider) {
                if (slot.body.collider) |collider| {
                    const mutable_collider = @constCast(collider);
                    mutable_collider.deinit();
                    self.allocator.destroy(mutable_collider);
                }
            }
        }

        self.allocator.free(self.bodies);
        self.free_list.deinit(self.allocator);
        self.broad_phase.deinit();
        self.narrow_phase.deinit();
        self.solver.deinit();
    }

    /// Create a dynamic body with automatic mass computation
    pub fn createDynamicBody(
        self: *VoxelPhysicsSystem,
        position: Vec3,
        rotation: Quat,
        collider: *VoxelCollider,
        density: f32,
    ) !BodyHandle {
        const handle = try self.allocateBody();
        const slot = &self.bodies[handle.index];

        const mass_props = computeMassProperties(collider, density);

        slot.body = VoxelBody.initDynamic(self.next_body_id, position, rotation);
        slot.body.collider = collider;
        slot.body.properties.inv_mass = mass_props.inv_mass;
        slot.body.properties.inv_inertia_local = mass_props.inv_inertia;
        slot.body.properties.friction = self.config.default_friction;
        slot.body.properties.restitution = self.config.default_restitution;

        self.next_body_id += 1;

        const aabb = slot.body.getWorldAABB();
        try self.broad_phase.addDynamicBody(slot.body.id, aabb);

        self.stats.total_bodies += 1;
        self.stats.dynamic_bodies += 1;

        return handle;
    }

    /// Create a dynamic body with explicit mass properties
    pub fn createDynamicBodyWithMass(
        self: *VoxelPhysicsSystem,
        position: Vec3,
        rotation: Quat,
        collider: *VoxelCollider,
        mass_props: MassProperties,
    ) !BodyHandle {
        const handle = try self.allocateBody();
        const slot = &self.bodies[handle.index];

        slot.body = VoxelBody.initDynamic(self.next_body_id, position, rotation);
        slot.body.collider = collider;
        slot.body.properties.inv_mass = mass_props.inv_mass;
        slot.body.properties.inv_inertia_local = mass_props.inv_inertia;
        slot.body.properties.friction = self.config.default_friction;
        slot.body.properties.restitution = self.config.default_restitution;

        self.next_body_id += 1;

        const aabb = slot.body.getWorldAABB();
        try self.broad_phase.addDynamicBody(slot.body.id, aabb);

        self.stats.total_bodies += 1;
        self.stats.dynamic_bodies += 1;

        return handle;
    }

    /// Create a static body
    pub fn createStaticBody(
        self: *VoxelPhysicsSystem,
        position: Vec3,
        rotation: Quat,
        collider: *VoxelCollider,
    ) !BodyHandle {
        const handle = try self.allocateBody();
        const slot = &self.bodies[handle.index];

        slot.body = VoxelBody.initStatic(self.next_body_id, position, rotation);
        slot.body.collider = collider;
        slot.body.properties.friction = self.config.default_friction;
        slot.body.properties.restitution = self.config.default_restitution;

        self.next_body_id += 1;

        const aabb = slot.body.getWorldAABB();
        try self.broad_phase.addStaticBody(slot.body.id, aabb);

        self.stats.total_bodies += 1;
        self.stats.static_bodies += 1;

        return handle;
    }

    /// Create a kinematic body
    pub fn createKinematicBody(
        self: *VoxelPhysicsSystem,
        position: Vec3,
        rotation: Quat,
        collider: *VoxelCollider,
    ) !BodyHandle {
        const handle = try self.allocateBody();
        const slot = &self.bodies[handle.index];

        slot.body = VoxelBody.initKinematic(self.next_body_id, position, rotation);
        slot.body.collider = collider;
        slot.body.properties.friction = self.config.default_friction;
        slot.body.properties.restitution = self.config.default_restitution;

        self.next_body_id += 1;

        const aabb = slot.body.getWorldAABB();
        try self.broad_phase.addDynamicBody(slot.body.id, aabb);

        self.stats.total_bodies += 1;

        return handle;
    }

    /// Destroy a body and free its resources
    pub fn destroyBody(self: *VoxelPhysicsSystem, handle: BodyHandle) void {
        if (!self.isValidHandle(handle)) return;

        const slot = &self.bodies[handle.index];

        self.stats.total_bodies -= 1;
        switch (slot.body.body_type) {
            .dynamic => self.stats.dynamic_bodies -= 1,
            .static => self.stats.static_bodies -= 1,
            .kinematic => {},
        }

        self.broad_phase.removeBody(slot.body.id);

        if (slot.owns_collider) {
            if (slot.body.collider) |collider| {
                const mutable_collider = @constCast(collider);
                mutable_collider.deinit();
                self.allocator.destroy(mutable_collider);
            }
        }

        slot.active = false;
        slot.generation += 1;
        slot.owns_collider = false;
        self.body_count -= 1;

        self.free_list.append(self.allocator, handle.index) catch {};
    }

    /// Get a body by handle
    pub fn getBody(self: *VoxelPhysicsSystem, handle: BodyHandle) ?*VoxelBody {
        if (!self.isValidHandle(handle)) return null;
        return &self.bodies[handle.index].body;
    }

    /// Get a body by handle
    pub fn getBodyConst(self: *const VoxelPhysicsSystem, handle: BodyHandle) ?*const VoxelBody {
        if (!self.isValidHandleConst(handle)) return null;
        return &self.bodies[handle.index].body;
    }

    /// Check if a handle is valid
    pub fn isValidHandle(self: *VoxelPhysicsSystem, handle: BodyHandle) bool {
        if (handle.index >= self.bodies.len) return false;
        const slot = &self.bodies[handle.index];
        return slot.active and slot.generation == handle.generation;
    }

    /// Check if a handle is valid
    fn isValidHandleConst(self: *const VoxelPhysicsSystem, handle: BodyHandle) bool {
        if (handle.index >= self.bodies.len) return false;
        const slot = &self.bodies[handle.index];
        return slot.active and slot.generation == handle.generation;
    }

    /// Find body handle by body ID
    pub fn findBodyById(self: *VoxelPhysicsSystem, body_id: u32) ?BodyHandle {
        for (self.bodies, 0..) |*slot, i| {
            if (slot.active and slot.body.id == body_id) {
                return BodyHandle{
                    .index = @intCast(i),
                    .generation = slot.generation,
                };
            }
        }
        return null;
    }

    /// Allocate a body slot
    fn allocateBody(self: *VoxelPhysicsSystem) !BodyHandle {
        var index: u32 = undefined;

        if (self.free_list.items.len > 0) {
            index = self.free_list.pop() orelse unreachable;
        } else {
            for (self.bodies, 0..) |*slot, i| {
                if (!slot.active) {
                    index = @intCast(i);
                    break;
                }
            } else {
                return error.MaxBodiesReached;
            }
        }

        const slot = &self.bodies[index];
        slot.active = true;
        self.body_count += 1;

        return BodyHandle{
            .index = index,
            .generation = slot.generation,
        };
    }

    /// Mark a body as owning its collider
    pub fn setColliderOwnership(self: *VoxelPhysicsSystem, handle: BodyHandle, owns: bool) void {
        if (!self.isValidHandle(handle)) return;
        self.bodies[handle.index].owns_collider = owns;
    }

    /// Update the physics simulation
    pub fn update(self: *VoxelPhysicsSystem, dt: f32) void {
        const frame_start = std.time.nanoTimestamp();
        self.stats.reset();

        self.accumulated_time += dt;
        self.stats.accumulated_time = self.accumulated_time;

        var substeps: u32 = 0;
        while (self.accumulated_time >= self.config.fixed_timestep and
            substeps < self.config.max_substeps)
        {
            self.step(self.config.fixed_timestep);
            self.accumulated_time -= self.config.fixed_timestep;
            substeps += 1;
        }

        if (self.accumulated_time > self.config.fixed_timestep * @as(f32, @floatFromInt(self.config.max_substeps))) {
            self.accumulated_time = self.config.fixed_timestep;
        }

        self.stats.substeps_this_frame = substeps;

        const frame_end = std.time.nanoTimestamp();
        self.stats.total_time_ms = @as(f32, @floatFromInt(frame_end - frame_start)) / 1_000_000.0;

        self.updateBodyCounts();
    }

    /// Perform a single physics step at fixed timestep
    pub fn step(self: *VoxelPhysicsSystem, dt: f32) void {
        if (self.config.enable_sleeping and self.allDynamicBodiesSleeping()) {
            self.stats.broad_phase_time_ms = 0;
            self.stats.narrow_phase_time_ms = 0;
            self.stats.island_build_time_ms = 0;
            self.stats.solver_time_ms = 0;
            return;
        }

        self.broadPhase();

        self.narrowPhase();

        self.solve(dt);

        if (self.config.enable_sleeping) {
            self.updateSleepStates(dt);
        }
    }

    /// Check if all dynamic bodies are currently sleeping
    fn allDynamicBodiesSleeping(self: *VoxelPhysicsSystem) bool {
        for (self.bodies) |*slot| {
            if (slot.active and slot.body.body_type == .dynamic) {
                if (!slot.body.is_sleeping) {
                    return false;
                }
            }
        }
        return true;
    }

    /// Broad phase collision detection
    fn broadPhase(self: *VoxelPhysicsSystem) void {
        const start = std.time.nanoTimestamp();

        for (self.bodies) |*slot| {
            if (slot.active and slot.body.body_type == .dynamic and !slot.body.is_sleeping) {
                const aabb = slot.body.getWorldAABB();
                const velocity = slot.body.getLinearVelocity();
                self.broad_phase.updateBody(slot.body.id, aabb, velocity) catch {};
            }
        }

        const end = std.time.nanoTimestamp();
        self.stats.broad_phase_time_ms += @as(f32, @floatFromInt(end - start)) / 1_000_000.0;
    }

    /// Narrow phase collision detection
    fn narrowPhase(self: *VoxelPhysicsSystem) void {
        const start = std.time.nanoTimestamp();

        self.narrow_phase.beginFrame();

        const all_pairs = self.broad_phase.findOverlappingPairs() catch &[_]BodyPair{};
        self.stats.broad_phase_pairs = @intCast(all_pairs.len);

        var filtered_pairs: [512]BodyPair = undefined;
        var filtered_count: usize = 0;
        var pairs_skipped_early: u32 = 0;

        for (all_pairs) |pair| {
            const body_a = self.findBodyPtrById(pair.body_a);
            const body_b = self.findBodyPtrById(pair.body_b);

            const a_sleeping = if (body_a) |b| b.is_sleeping else false;
            const b_sleeping = if (body_b) |b| b.is_sleeping else false;

            if (a_sleeping and b_sleeping) {
                pairs_skipped_early += 1;

                const key = pairKey(pair.body_a, pair.body_b);
                if (self.narrow_phase.contact_manager.previous_manifolds.get(key)) |prev_manifold| {
                    self.narrow_phase.contact_manager.manifolds.put(key, prev_manifold) catch {};
                }
            } else if (filtered_count < filtered_pairs.len) {
                filtered_pairs[filtered_count] = pair;
                filtered_count += 1;
            }
        }

        const pairs = filtered_pairs[0..filtered_count];
        self.stats.pairs_skipped_sleeping = pairs_skipped_early;

        if (self.thread_pool) |pool| {
            var body_ptrs: [256]*const VoxelBody = undefined;
            var body_count: usize = 0;

            for (self.bodies) |*slot| {
                if (slot.active and body_count < body_ptrs.len) {
                    body_ptrs[body_count] = &slot.body;
                    body_count += 1;
                }
            }

            self.narrow_phase.processPairsParallel(
                pool,
                pairs,
                body_ptrs[0..body_count],
            ) catch {
                self.processNarrowPhaseSequential(pairs);
            };
        } else {
            self.processNarrowPhaseSequential(pairs);
        }

        self.stats.narrow_phase_tested = self.narrow_phase.stats.pairs_tested;
        self.stats.narrow_phase_colliding = self.narrow_phase.stats.pairs_colliding;
        self.stats.contacts_generated = self.narrow_phase.stats.contacts_generated;
        self.stats.voxels_checked = self.narrow_phase.stats.voxels_checked;
        self.stats.pairs_parallel = self.narrow_phase.stats.pairs_parallel;
        self.stats.pairs_sequential = self.narrow_phase.stats.pairs_sequential;
        self.stats.pairs_skipped_sleeping += self.narrow_phase.stats.pairs_skipped_sleeping;
        self.stats.cache_hits = self.narrow_phase.stats.cache_hits;
        self.stats.cache_misses = self.narrow_phase.stats.cache_misses;

        const end = std.time.nanoTimestamp();
        self.stats.narrow_phase_time_ms += @as(f32, @floatFromInt(end - start)) / 1_000_000.0;
    }

    /// Sequential narrow phase processing
    fn processNarrowPhaseSequential(self: *VoxelPhysicsSystem, pairs: []const BodyPair) void {
        for (pairs) |pair| {
            const body_a = self.findBodyPtrById(pair.body_a);
            const body_b = self.findBodyPtrById(pair.body_b);

            if (body_a != null and body_b != null) {
                self.narrow_phase.processPair(body_a.?, body_b.?) catch {};
            }
        }
    }

    /// Constraint solving and integration
    fn solve(self: *VoxelPhysicsSystem, dt: f32) void {
        const start = std.time.nanoTimestamp();

        self.solver.clear();

        self.solver.config.gravity = self.config.gravity;

        for (self.bodies) |*slot| {
            if (slot.active) {
                self.solver.addBody(&slot.body) catch {};
            }
        }

        if (self.config.enable_warm_starting) {
            self.solver.setContactManager(self.narrow_phase.getContactManagerMut());
        }

        var contact_iter = self.narrow_phase.getContactManager().iterator();
        var manifolds_skipped: u32 = 0;
        while (contact_iter.next()) |manifold| {
            const body_a = self.findBodyPtrById(manifold.body_a_id);
            const body_b = self.findBodyPtrById(manifold.body_b_id);

            const a_sleeping = if (body_a) |b| b.is_sleeping else true;
            const b_sleeping = if (body_b) |b| b.is_sleeping else true;

            if (a_sleeping and b_sleeping) {
                manifolds_skipped += 1;
                continue;
            }

            self.solver.addManifold(manifold) catch {};
        }
        self.stats.manifolds_skipped_sleeping = manifolds_skipped;

        if (self.island_manager) |*manager| {
            const island_start = std.time.nanoTimestamp();

            const contact_manager = self.narrow_phase.getContactManager();
            var manifold_list = std.ArrayListUnmanaged(ContactManifold){};
            defer manifold_list.deinit(self.allocator);

            var manifold_iter = contact_manager.manifolds.valueIterator();
            while (manifold_iter.next()) |manifold| {
                manifold_list.append(self.allocator, manifold.*) catch {};
            }

            var body_types = std.ArrayListUnmanaged(BodyType){};
            defer body_types.deinit(self.allocator);

            var max_id: u32 = 0;
            for (self.bodies) |*slot| {
                if (slot.active and slot.body.id > max_id) {
                    max_id = slot.body.id;
                }
            }

            body_types.resize(self.allocator, max_id + 1) catch {};
            for (body_types.items) |*t| {
                t.* = .static;
            }
            for (self.bodies) |*slot| {
                if (slot.active and slot.body.id < body_types.items.len) {
                    body_types.items[slot.body.id] = slot.body.body_type;
                }
            }

            manager.buildIslandsWithBodyTypes(manifold_list.items, self.body_count, body_types.items) catch {};

            const island_stats = manager.getStats();
            self.stats.island_count = @intCast(island_stats.total_islands);
            self.stats.active_islands = @intCast(island_stats.active_islands);
            self.stats.sleeping_islands = @intCast(island_stats.sleeping_islands);
            self.stats.largest_island_size = @intCast(island_stats.largest_island_size);

            const island_end = std.time.nanoTimestamp();
            self.stats.island_build_time_ms = @as(f32, @floatFromInt(island_end - island_start)) / 1_000_000.0;

            var body_slice: []VoxelBody = undefined;
            var temp_bodies = std.ArrayListUnmanaged(VoxelBody){};
            defer temp_bodies.deinit(self.allocator);

            for (self.bodies) |*slot| {
                if (slot.active) {
                    temp_bodies.append(self.allocator, slot.body) catch {};
                }
            }
            body_slice = temp_bodies.items;
            manager.updateSleepStates(body_slice, dt);
        }

        if (self.graph_coloring) |*coloring| {
            var manifold_list_for_coloring = std.ArrayListUnmanaged(ContactManifold){};
            defer manifold_list_for_coloring.deinit(self.allocator);

            const contact_manager = self.narrow_phase.getContactManager();
            var color_manifold_iter = contact_manager.manifolds.valueIterator();
            while (color_manifold_iter.next()) |manifold| {
                manifold_list_for_coloring.append(self.allocator, manifold.*) catch {};
            }
            coloring.build(manifold_list_for_coloring.items) catch {};
        }

        if (self.thread_pool) |pool| {
            self.stats.threads_used = @intCast(pool.threadCount());
        }

        self.solver.solve(dt);

        self.stats.max_penetration = self.solver.stats.max_penetration;

        const end = std.time.nanoTimestamp();
        self.stats.solver_time_ms += @as(f32, @floatFromInt(end - start)) / 1_000_000.0;
    }

    /// Update sleep states for all bodies
    fn updateSleepStates(self: *VoxelPhysicsSystem, dt: f32) void {
        for (self.bodies) |*slot| {
            if (slot.active and slot.body.body_type == .dynamic and !slot.body.is_sleeping) {
                slot.body.updateSleepState(dt);
            }
        }
    }

    /// Update body count statistics
    fn updateBodyCounts(self: *VoxelPhysicsSystem) void {
        var sleeping: u32 = 0;
        var active: u32 = 0;

        for (self.bodies) |*slot| {
            if (slot.active and slot.body.body_type == .dynamic) {
                if (slot.body.is_sleeping) {
                    sleeping += 1;
                } else {
                    active += 1;
                }
            }
        }

        self.stats.sleeping_bodies = sleeping;
        self.stats.active_bodies = active;
    }

    /// Find body pointer by ID
    fn findBodyPtrById(self: *VoxelPhysicsSystem, body_id: u32) ?*const VoxelBody {
        for (self.bodies) |*slot| {
            if (slot.active and slot.body.id == body_id) {
                return &slot.body;
            }
        }
        return null;
    }

    /// Get the thread pool
    pub fn getThreadPool(self: *VoxelPhysicsSystem) ?*ThreadPool {
        return self.thread_pool;
    }

    /// Get the island manager
    pub fn getIslandManager(self: *VoxelPhysicsSystem) ?*IslandManager {
        if (self.island_manager) |*manager| {
            return manager;
        }
        return null;
    }

    /// Get the graph coloring
    pub fn getGraphColoring(self: *VoxelPhysicsSystem) ?*GraphColoring {
        if (self.graph_coloring) |*coloring| {
            return coloring;
        }
        return null;
    }

    /// Check if a body is in a sleeping island
    pub fn isBodyInSleepingIsland(self: *VoxelPhysicsSystem, handle: BodyHandle) bool {
        if (!self.isValidHandle(handle)) return false;
        if (self.island_manager) |*manager| {
            const body = &self.bodies[handle.index].body;
            return manager.isBodyInSleepingIsland(body.id);
        }
        return false;
    }

    /// Wake up all bodies in the island containing the specified body
    pub fn wakeBodyIsland(self: *VoxelPhysicsSystem, handle: BodyHandle) void {
        if (!self.isValidHandle(handle)) return;
        if (self.island_manager) |*manager| {
            const body = &self.bodies[handle.index].body;
            manager.wakeBodyIsland(body.id);
        }

        if (self.getBody(handle)) |body| {
            body.wakeUp();
        }
    }

    /// Get island statistics
    pub fn getIslandStats(self: *const VoxelPhysicsSystem) ?islands_mod.IslandManager.IslandStats {
        if (self.island_manager) |manager| {
            return manager.stats;
        }
        return null;
    }

    /// Check if threading is enabled and active
    pub fn isThreadingEnabled(self: *const VoxelPhysicsSystem) bool {
        return self.thread_pool != null;
    }

    /// Check if island-based solving is enabled
    pub fn isIslandSolvingEnabled(self: *const VoxelPhysicsSystem) bool {
        return self.island_manager != null;
    }

    /// Apply a force to a body at its center of mass
    pub fn applyForce(self: *VoxelPhysicsSystem, handle: BodyHandle, force: Vec3) void {
        if (self.getBody(handle)) |body| {
            body.applyForce(force);
        }
    }

    /// Apply a force at a world point
    pub fn applyForceAtPoint(self: *VoxelPhysicsSystem, handle: BodyHandle, force: Vec3, point: Vec3) void {
        if (self.getBody(handle)) |body| {
            body.applyForceAtPoint(force, point);
        }
    }

    /// Apply an impulse at center of mass
    pub fn applyImpulse(self: *VoxelPhysicsSystem, handle: BodyHandle, impulse: Vec3) void {
        if (self.getBody(handle)) |body| {
            body.applyImpulse(impulse);
        }
    }

    /// Apply an impulse at a world point
    pub fn applyImpulseAtPoint(self: *VoxelPhysicsSystem, handle: BodyHandle, impulse: Vec3, point: Vec3) void {
        if (self.getBody(handle)) |body| {
            body.applyImpulseAtPoint(impulse, point);
        }
    }

    /// Apply torque
    pub fn applyTorque(self: *VoxelPhysicsSystem, handle: BodyHandle, torque: Vec3) void {
        if (self.getBody(handle)) |body| {
            body.applyTorque(torque);
        }
    }

    /// Set linear velocity
    pub fn setLinearVelocity(self: *VoxelPhysicsSystem, handle: BodyHandle, velocity: Vec3) void {
        if (self.getBody(handle)) |body| {
            body.setLinearVelocity(velocity);
        }
    }

    /// Set angular velocity
    pub fn setAngularVelocity(self: *VoxelPhysicsSystem, handle: BodyHandle, velocity: Vec3) void {
        if (self.getBody(handle)) |body| {
            body.setAngularVelocity(velocity);
        }
    }

    /// Set position
    pub fn setPosition(self: *VoxelPhysicsSystem, handle: BodyHandle, position: Vec3) void {
        if (self.getBody(handle)) |body| {
            body.setPosition(position);
        }
    }

    /// Set rotation
    pub fn setRotation(self: *VoxelPhysicsSystem, handle: BodyHandle, rotation: Quat) void {
        if (self.getBody(handle)) |body| {
            body.setRotation(rotation);
        }
    }

    /// Wake up a body
    pub fn wakeUp(self: *VoxelPhysicsSystem, handle: BodyHandle) void {
        if (self.getBody(handle)) |body| {
            body.wakeUp();
        }
    }

    /// Put a body to sleep
    pub fn sleep(self: *VoxelPhysicsSystem, handle: BodyHandle) void {
        if (self.getBody(handle)) |body| {
            body.sleep();
        }
    }

    /// Get position of a body
    pub fn getPosition(self: *VoxelPhysicsSystem, handle: BodyHandle) ?Vec3 {
        if (self.getBodyConst(handle)) |body| {
            return body.getPosition();
        }
        return null;
    }

    /// Get rotation of a body
    pub fn getRotation(self: *VoxelPhysicsSystem, handle: BodyHandle) ?Quat {
        if (self.getBodyConst(handle)) |body| {
            return body.getRotation();
        }
        return null;
    }

    /// Get transform of a body
    pub fn getTransform(self: *VoxelPhysicsSystem, handle: BodyHandle) ?Transform {
        if (self.getBodyConst(handle)) |body| {
            return body.getTransform();
        }
        return null;
    }

    /// Get linear velocity
    pub fn getLinearVelocity(self: *VoxelPhysicsSystem, handle: BodyHandle) ?Vec3 {
        if (self.getBodyConst(handle)) |body| {
            return body.getLinearVelocity();
        }
        return null;
    }

    /// Get angular velocity
    pub fn getAngularVelocity(self: *VoxelPhysicsSystem, handle: BodyHandle) ?Vec3 {
        if (self.getBodyConst(handle)) |body| {
            return body.getAngularVelocity();
        }
        return null;
    }

    /// Get world AABB of a body
    pub fn getWorldAABB(self: *VoxelPhysicsSystem, handle: BodyHandle) ?AABB {
        if (self.getBodyConst(handle)) |body| {
            return body.getWorldAABB();
        }
        return null;
    }

    /// Check if body is sleeping
    pub fn isSleeping(self: *VoxelPhysicsSystem, handle: BodyHandle) bool {
        if (self.getBodyConst(handle)) |body| {
            return body.is_sleeping;
        }
        return false;
    }

    /// Get body count
    pub fn getBodyCount(self: *const VoxelPhysicsSystem) u32 {
        return self.body_count;
    }

    /// Get statistics
    pub fn getStats(self: *const VoxelPhysicsSystem) PhysicsStats {
        return self.stats;
    }

    /// Get contact manager for debug visualization
    pub fn getContactManager(self: *VoxelPhysicsSystem) *ContactManager {
        return self.narrow_phase.getContactManagerMut();
    }

    /// Set gravity
    pub fn setGravity(self: *VoxelPhysicsSystem, gravity: Vec3) void {
        self.config.gravity = gravity;
    }

    /// Get gravity
    pub fn getGravity(self: *const VoxelPhysicsSystem) Vec3 {
        return self.config.gravity;
    }

    /// Set solver parameters
    pub fn setSolverConfig(self: *VoxelPhysicsSystem, config: SolverConfig) void {
        self.solver.config = config;
        self.solver.config.gravity = self.config.gravity;
    }

    /// Get solver config
    pub fn getSolverConfigMut(self: *VoxelPhysicsSystem) *SolverConfig {
        return &self.solver.config;
    }

    /// Iterator over all active bodies
    pub const BodyIterator = struct {
        system: *VoxelPhysicsSystem,
        index: u32,

        pub fn next(self: *BodyIterator) ?struct { handle: BodyHandle, body: *VoxelBody } {
            while (self.index < self.system.bodies.len) {
                const i = self.index;
                self.index += 1;

                const slot = &self.system.bodies[i];
                if (slot.active) {
                    return .{
                        .handle = BodyHandle{
                            .index = i,
                            .generation = slot.generation,
                        },
                        .body = &slot.body,
                    };
                }
            }
            return null;
        }
    };

    /// Get iterator over all active bodies
    pub fn bodyIterator(self: *VoxelPhysicsSystem) BodyIterator {
        return BodyIterator{
            .system = self,
            .index = 0,
        };
    }
};

test "VoxelPhysicsSystem init and deinit" {
    const allocator = std.testing.allocator;
    var system = try VoxelPhysicsSystem.init(allocator, .{});
    defer system.deinit();

    try std.testing.expectEqual(@as(u32, 0), system.getBodyCount());
}

test "VoxelPhysicsSystem create and destroy body" {
    const allocator = std.testing.allocator;
    var system = try VoxelPhysicsSystem.init(allocator, .{});
    defer system.deinit();

    var collider = VoxelCollider.init(allocator, 0.1);
    defer collider.deinit();

    const handle = try system.createDynamicBody(
        Vec3.init(0, 5, 0),
        Quat.identity,
        &collider,
        1.0,
    );

    try std.testing.expect(system.isValidHandle(handle));
    try std.testing.expectEqual(@as(u32, 1), system.getBodyCount());

    system.destroyBody(handle);
    try std.testing.expect(!system.isValidHandle(handle));
    try std.testing.expectEqual(@as(u32, 0), system.getBodyCount());
}

test "VoxelPhysicsSystem body handle invalidation" {
    const allocator = std.testing.allocator;
    var system = try VoxelPhysicsSystem.init(allocator, .{});
    defer system.deinit();

    var collider = VoxelCollider.init(allocator, 0.1);
    defer collider.deinit();

    const handle1 = try system.createDynamicBody(Vec3.zero, Quat.identity, &collider, 1.0);
    system.destroyBody(handle1);

    try std.testing.expect(!system.isValidHandle(handle1));

    const handle2 = try system.createDynamicBody(Vec3.zero, Quat.identity, &collider, 1.0);

    try std.testing.expectEqual(handle1.index, handle2.index);
    try std.testing.expect(handle1.generation != handle2.generation);
    try std.testing.expect(system.isValidHandle(handle2));
}

test "VoxelPhysicsSystem step" {
    const allocator = std.testing.allocator;
    var system = try VoxelPhysicsSystem.init(allocator, .{});
    defer system.deinit();

    var collider = VoxelCollider.init(allocator, 0.1);
    defer collider.deinit();

    const handle = try system.createDynamicBody(
        Vec3.init(0, 5, 0),
        Quat.identity,
        &collider,
        1.0,
    );

    system.step(1.0 / 60.0);

    const pos = system.getPosition(handle).?;
    try std.testing.expect(pos.y < 5.0);
}

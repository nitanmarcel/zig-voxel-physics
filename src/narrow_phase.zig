const std = @import("std");
const math = @import("math.zig");
const collision_voxel = @import("collision_voxel.zig");
const collider_mod = @import("collider.zig");
const body_mod = @import("body.zig");
const thread_pool_mod = @import("thread_pool.zig");
const broad_phase_mod = @import("broad_phase.zig");

const Vec3 = math.Vec3;
const IVec3 = math.IVec3;
const Quat = math.Quat;
const Mat3 = math.Mat3;
const Transform = math.Transform;
const AABB = math.AABB;
const OBB = math.OBB;
const EPSILON = math.EPSILON;

const CollisionVoxel = collision_voxel.CollisionVoxel;
const VoxelType = collision_voxel.VoxelType;
const NORMAL_LUT = collision_voxel.NORMAL_LUT;
const getNormalVec3 = collision_voxel.getNormalVec3;

const CollisionBrick = collider_mod.CollisionBrick;
const VoxelCollider = collider_mod.VoxelCollider;
const BRICK_SIZE = collider_mod.BRICK_SIZE;

const VoxelBody = body_mod.VoxelBody;
const ThreadPool = thread_pool_mod.ThreadPool;
const BodyPair = broad_phase_mod.BodyPair;

/// Maximum number of contacts per manifold
pub const MAX_CONTACTS_PER_MANIFOLD = 4;

/// Maximum number of manifolds per frame
pub const MAX_MANIFOLDS = 256;

/// Maximum number of worker threads for parallel processing
pub const MAX_WORKER_THREADS = 16;

/// Minimum pairs required to use parallel processing
pub const MIN_PARALLEL_PAIRS = 8;

/// Maximum number of cached brick-pair results
pub const MAX_BRICK_PAIR_CACHE_SIZE = 1024;

/// Transform change threshold for cache invalidation
pub const CACHE_TRANSFORM_THRESHOLD_SQ: f32 = 0.0001;

/// Rotation change threshold for cache invalidation
pub const CACHE_ROTATION_THRESHOLD: f32 = 0.9999;

/// Distance threshold for voxel collision
pub const VOXEL_COLLISION_THRESHOLD: f32 = 1.0;

/// Minimum separation axis length to consider valid
pub const MIN_AXIS_LENGTH: f32 = 0.001;

/// Slop for penetration
pub const PENETRATION_SLOP: f32 = 0.005;

/// Speculative contact margin - generate contacts for voxels that are close but not touching
pub const SPECULATIVE_MARGIN: f32 = 0.02;

/// Cached contact data for a brick pair
pub const CachedBrickContacts = struct {
    /// Contact points
    contacts: [MAX_CONTACTS_PER_MANIFOLD]ContactPoint,
    contact_count: u8,

    /// Whether collision was detected
    has_collision: bool,

    /// Number of voxels checked to generate this result
    voxels_checked: u32,

    pub fn init() CachedBrickContacts {
        return .{
            .contacts = undefined,
            .contact_count = 0,
            .has_collision = false,
            .voxels_checked = 0,
        };
    }
};

/// Key for brick-pair cache lookup
pub const BrickPairKey = struct {
    /// Body IDs
    body_a_id: u32,
    body_b_id: u32,

    /// Brick positions within each body's collider
    brick_a_pos: IVec3,
    brick_b_pos: IVec3,

    /// Hash of relative transform for invalidation
    transform_hash: u64,

    pub fn init(
        body_a: u32,
        body_b: u32,
        brick_a: IVec3,
        brick_b: IVec3,
        rel_pos: Vec3,
        rel_rot: Quat,
    ) BrickPairKey {
        const a_first = body_a < body_b;
        return .{
            .body_a_id = if (a_first) body_a else body_b,
            .body_b_id = if (a_first) body_b else body_a,
            .brick_a_pos = if (a_first) brick_a else brick_b,
            .brick_b_pos = if (a_first) brick_b else brick_a,
            .transform_hash = computeTransformHash(rel_pos, rel_rot),
        };
    }

    /// Compute a hash of the key for hashmap lookup
    pub fn hash(self: BrickPairKey) u64 {
        var h: u64 = 0;
        h = h *% 31 +% @as(u64, self.body_a_id);
        h = h *% 31 +% @as(u64, self.body_b_id);

        h = h *% 31 +% @as(u64, @as(u32, @bitCast(self.brick_a_pos.x)));
        h = h *% 31 +% @as(u64, @as(u32, @bitCast(self.brick_a_pos.y)));
        h = h *% 31 +% @as(u64, @as(u32, @bitCast(self.brick_a_pos.z)));
        h = h *% 31 +% @as(u64, @as(u32, @bitCast(self.brick_b_pos.x)));
        h = h *% 31 +% @as(u64, @as(u32, @bitCast(self.brick_b_pos.y)));
        h = h *% 31 +% @as(u64, @as(u32, @bitCast(self.brick_b_pos.z)));
        h = h *% 31 +% self.transform_hash;
        return h;
    }

    /// Check equality for hashmap
    pub fn eql(self: BrickPairKey, other: BrickPairKey) bool {
        return self.body_a_id == other.body_a_id and
            self.body_b_id == other.body_b_id and
            self.brick_a_pos.x == other.brick_a_pos.x and
            self.brick_a_pos.y == other.brick_a_pos.y and
            self.brick_a_pos.z == other.brick_a_pos.z and
            self.brick_b_pos.x == other.brick_b_pos.x and
            self.brick_b_pos.y == other.brick_b_pos.y and
            self.brick_b_pos.z == other.brick_b_pos.z and
            self.transform_hash == other.transform_hash;
    }
};

/// Compute a hash from relative position and rotation
fn computeTransformHash(rel_pos: Vec3, rel_rot: Quat) u64 {
    const grid_scale: f32 = 100.0;
    const qx: i32 = @intFromFloat(rel_pos.x * grid_scale);
    const qy: i32 = @intFromFloat(rel_pos.y * grid_scale);
    const qz: i32 = @intFromFloat(rel_pos.z * grid_scale);

    const rot_scale: f32 = 1000.0;
    const rw: i16 = @intFromFloat(std.math.clamp(rel_rot.w * rot_scale, -32767.0, 32767.0));
    const rx: i16 = @intFromFloat(std.math.clamp(rel_rot.x * rot_scale, -32767.0, 32767.0));
    const ry: i16 = @intFromFloat(std.math.clamp(rel_rot.y * rot_scale, -32767.0, 32767.0));
    const rz: i16 = @intFromFloat(std.math.clamp(rel_rot.z * rot_scale, -32767.0, 32767.0));

    var h: u64 = 0;
    h = h *% 31 +% @as(u64, @as(u32, @bitCast(qx)));
    h = h *% 31 +% @as(u64, @as(u32, @bitCast(qy)));
    h = h *% 31 +% @as(u64, @as(u32, @bitCast(qz)));
    h = h *% 31 +% @as(u64, @as(u16, @bitCast(rw)));
    h = h *% 31 +% @as(u64, @as(u16, @bitCast(rx)));
    h = h *% 31 +% @as(u64, @as(u16, @bitCast(ry)));
    h = h *% 31 +% @as(u64, @as(u16, @bitCast(rz)));
    return h;
}

/// Cache entry with LRU tracking
const BrickPairCacheEntry = struct {
    key: BrickPairKey,
    value: CachedBrickContacts,
    last_used_frame: u64,
    valid: bool,
};

/// Brick-pair collision cache
pub const BrickPairCache = struct {
    allocator: std.mem.Allocator,

    /// Cache storage using open addressing
    entries: []BrickPairCacheEntry,
    capacity: usize,

    /// Current frame for LRU tracking
    current_frame: u64,

    /// Statistics
    stats: struct {
        hits: u64,
        misses: u64,
        evictions: u64,
        invalidations: u64,
    },

    pub fn init(allocator: std.mem.Allocator, capacity: usize) !BrickPairCache {
        const entries = try allocator.alloc(BrickPairCacheEntry, capacity);
        for (entries) |*e| {
            e.valid = false;
        }

        return .{
            .allocator = allocator,
            .entries = entries,
            .capacity = capacity,
            .current_frame = 0,
            .stats = .{
                .hits = 0,
                .misses = 0,
                .evictions = 0,
                .invalidations = 0,
            },
        };
    }

    pub fn deinit(self: *BrickPairCache) void {
        self.allocator.free(self.entries);
    }

    /// Begin a new frame
    pub fn beginFrame(self: *BrickPairCache) void {
        self.current_frame += 1;
    }

    /// Look up cached result for a brick pair
    pub fn get(self: *BrickPairCache, key: BrickPairKey) ?*const CachedBrickContacts {
        const hash_val = key.hash();
        const start_idx = hash_val % self.capacity;

        var idx = start_idx;
        var probes: usize = 0;
        const max_probes = @min(self.capacity, 16);

        while (probes < max_probes) {
            const entry = &self.entries[idx];
            if (!entry.valid) {
                self.stats.misses += 1;
                return null;
            }
            if (entry.key.eql(key)) {
                entry.last_used_frame = self.current_frame;
                self.stats.hits += 1;
                return &entry.value;
            }
            idx = (idx + 1) % self.capacity;
            probes += 1;
        }

        self.stats.misses += 1;
        return null;
    }

    /// Insert or update cached result
    pub fn put(self: *BrickPairCache, key: BrickPairKey, value: CachedBrickContacts) void {
        const hash_val = key.hash();
        const start_idx = hash_val % self.capacity;

        var idx = start_idx;
        var probes: usize = 0;
        const max_probes = @min(self.capacity, 16);

        var oldest_idx: usize = start_idx;
        var oldest_frame: u64 = std.math.maxInt(u64);

        while (probes < max_probes) {
            const entry = &self.entries[idx];

            if (!entry.valid) {
                entry.* = .{
                    .key = key,
                    .value = value,
                    .last_used_frame = self.current_frame,
                    .valid = true,
                };
                return;
            }

            if (entry.key.eql(key)) {
                entry.value = value;
                entry.last_used_frame = self.current_frame;
                return;
            }

            if (entry.last_used_frame < oldest_frame) {
                oldest_frame = entry.last_used_frame;
                oldest_idx = idx;
            }

            idx = (idx + 1) % self.capacity;
            probes += 1;
        }

        self.stats.evictions += 1;
        self.entries[oldest_idx] = .{
            .key = key,
            .value = value,
            .last_used_frame = self.current_frame,
            .valid = true,
        };
    }

    /// Invalidate all entries for a specific body
    pub fn invalidateBody(self: *BrickPairCache, body_id: u32) void {
        for (self.entries) |*entry| {
            if (entry.valid and (entry.key.body_a_id == body_id or entry.key.body_b_id == body_id)) {
                entry.valid = false;
                self.stats.invalidations += 1;
            }
        }
    }

    /// Clear the entire cache
    pub fn clear(self: *BrickPairCache) void {
        for (self.entries) |*e| {
            e.valid = false;
        }
        self.stats = .{
            .hits = 0,
            .misses = 0,
            .evictions = 0,
            .invalidations = 0,
        };
    }

    /// Get cache hit rate
    pub fn getHitRate(self: *const BrickPairCache) f32 {
        const total = self.stats.hits + self.stats.misses;
        if (total == 0) return 0.0;
        return @as(f32, @floatFromInt(self.stats.hits)) / @as(f32, @floatFromInt(total));
    }
};

/// A single contact point between two bodies
pub const ContactPoint = struct {
    /// World-space contact point
    point: Vec3,

    /// Contact normal
    normal: Vec3,

    /// Penetration depth
    penetration: f32,

    /// Local anchor on body A
    local_anchor_a: Vec3,

    /// Local anchor on body B
    local_anchor_b: Vec3,

    /// Accumulated normal impulse
    normal_impulse: f32 = 0,

    /// Accumulated tangent impulse
    tangent_impulse: [2]f32 = .{ 0, 0 },

    /// Cached effective mass for normal direction
    normal_mass: f32 = 0,

    /// Cached effective mass for tangent directions
    tangent_mass: [2]f32 = .{ 0, 0 },

    /// Feature ID for contact matching
    feature_id: u64 = 0,

    pub fn init(
        point: Vec3,
        normal: Vec3,
        penetration: f32,
        local_anchor_a: Vec3,
        local_anchor_b: Vec3,
    ) ContactPoint {
        return .{
            .point = point,
            .normal = normal,
            .penetration = penetration,
            .local_anchor_a = local_anchor_a,
            .local_anchor_b = local_anchor_b,
        };
    }

    /// Compute a feature ID from voxel coordinates for contact matching
    pub fn computeFeatureId(voxel_a: IVec3, voxel_b: IVec3) u64 {
        const ax: u64 = @bitCast(@as(i64, voxel_a.x) & 0xFFFF);
        const ay: u64 = @bitCast((@as(i64, voxel_a.y) & 0xFFFF) << 16);
        const az: u64 = @bitCast((@as(i64, voxel_a.z) & 0xFFFF) << 32);
        const bx: u64 = @bitCast((@as(i64, voxel_b.x) & 0xF) << 48);
        const by: u64 = @bitCast((@as(i64, voxel_b.y) & 0xF) << 52);
        const bz: u64 = @bitCast((@as(i64, voxel_b.z) & 0xF) << 56);
        return ax | ay | az | bx | by | bz;
    }
};

/// Fixed-size contact array
pub const ContactArray = struct {
    buffer: [MAX_CONTACTS_PER_MANIFOLD]ContactPoint = undefined,
    len: usize = 0,

    pub fn slice(self: *ContactArray) []ContactPoint {
        return self.buffer[0..self.len];
    }

    pub fn constSlice(self: *const ContactArray) []const ContactPoint {
        return self.buffer[0..self.len];
    }

    pub fn append(self: *ContactArray, item: ContactPoint) error{Overflow}!void {
        if (self.len >= MAX_CONTACTS_PER_MANIFOLD) return error.Overflow;
        self.buffer[self.len] = item;
        self.len += 1;
    }
};

/// A collection of contact points between two bodies
pub const ContactManifold = struct {
    /// Body A (owner)
    body_a_id: u32,

    /// Body B (other)
    body_b_id: u32,

    /// Contact points
    contacts: ContactArray,

    /// Friction anchor for sticky friction
    friction_anchor: ?Vec3,

    /// Combined friction coefficient
    friction: f32,

    /// Combined restitution coefficient
    restitution: f32,

    /// Whether this manifold is still active
    is_active: bool,

    pub fn init(body_a_id: u32, body_b_id: u32, friction: f32, restitution: f32) ContactManifold {
        return .{
            .body_a_id = body_a_id,
            .body_b_id = body_b_id,
            .contacts = .{},
            .friction_anchor = null,
            .friction = friction,
            .restitution = restitution,
            .is_active = true,
        };
    }

    /// Add a contact point, potentially replacing the worst one if full
    pub fn addContact(self: *ContactManifold, contact: ContactPoint) void {
        for (self.contacts.slice()) |*existing| {
            if (existing.feature_id == contact.feature_id) {
                const old_normal_impulse = existing.normal_impulse;
                const old_tangent_impulse = existing.tangent_impulse;
                existing.* = contact;
                existing.normal_impulse = old_normal_impulse;
                existing.tangent_impulse = old_tangent_impulse;
                return;
            }
        }

        if (self.contacts.len < MAX_CONTACTS_PER_MANIFOLD) {
            self.contacts.append(contact) catch {};
            return;
        }

        var min_penetration = contact.penetration;
        var min_index: ?usize = null;

        for (self.contacts.slice(), 0..) |existing, i| {
            if (existing.penetration < min_penetration) {
                min_penetration = existing.penetration;
                min_index = i;
            }
        }

        if (min_index) |idx| {
            self.contacts.buffer[idx] = contact;
        }
    }

    /// Clear all contacts
    pub fn clear(self: *ContactManifold) void {
        self.contacts.len = 0;
        self.friction_anchor = null;
        self.is_active = false;
    }

    /// Get the deepest contact
    pub fn getDeepestContact(self: *const ContactManifold) ?*const ContactPoint {
        if (self.contacts.len == 0) return null;

        var deepest: *const ContactPoint = &self.contacts.buffer[0];
        for (self.contacts.constSlice()[1..]) |*contact| {
            if (contact.penetration > deepest.penetration) {
                deepest = contact;
            }
        }
        return deepest;
    }
};

/// Project an OBB onto an axis and return min/max values
fn projectOBBOnAxis(obb: OBB, axis: Vec3) struct { min: f32, max: f32 } {
    const vertices = obb.getVertices();
    var min_proj: f32 = std.math.inf(f32);
    var max_proj: f32 = -std.math.inf(f32);

    for (vertices) |v| {
        const proj = v.dot(axis);
        min_proj = @min(min_proj, proj);
        max_proj = @max(max_proj, proj);
    }

    return .{ .min = min_proj, .max = max_proj };
}

/// Check if two projections overlap on an axis
fn projectionsOverlap(a_min: f32, a_max: f32, b_min: f32, b_max: f32) bool {
    return !(a_max < b_min or b_max < a_min);
}

/// Compute the overlap amount between two projections
fn projectionOverlapAmount(a_min: f32, a_max: f32, b_min: f32, b_max: f32) f32 {
    if (a_max < b_min) return a_max - b_min;
    if (b_max < a_min) return b_max - a_min;
    return @min(a_max - b_min, b_max - a_min);
}

/// SAT test between two OBBs. Returns true if they are separated.
pub fn satEarlyOut(obb_a: OBB, obb_b: OBB) bool {
    const axes_a = obb_a.getAxes();
    const axes_b = obb_b.getAxes();

    for (axes_a) |axis| {
        const proj_a = projectOBBOnAxis(obb_a, axis);
        const proj_b = projectOBBOnAxis(obb_b, axis);
        if (!projectionsOverlap(proj_a.min, proj_a.max, proj_b.min, proj_b.max)) {
            return true;
        }
    }

    for (axes_b) |axis| {
        const proj_a = projectOBBOnAxis(obb_a, axis);
        const proj_b = projectOBBOnAxis(obb_b, axis);
        if (!projectionsOverlap(proj_a.min, proj_a.max, proj_b.min, proj_b.max)) {
            return true;
        }
    }

    for (axes_a) |axis_a| {
        for (axes_b) |axis_b| {
            const cross = axis_a.cross(axis_b);
            const len = cross.length();

            if (len < MIN_AXIS_LENGTH) continue;

            const axis = cross.scale(1.0 / len);
            const proj_a = projectOBBOnAxis(obb_a, axis);
            const proj_b = projectOBBOnAxis(obb_b, axis);
            if (!projectionsOverlap(proj_a.min, proj_a.max, proj_b.min, proj_b.max)) {
                return true;
            }
        }
    }

    return false;
}

/// SAT test with penetration depth calculation. Returns null if separated.
pub fn satWithPenetration(obb_a: OBB, obb_b: OBB) ?struct { axis: Vec3, depth: f32 } {
    var min_overlap: f32 = std.math.inf(f32);
    var min_axis: Vec3 = Vec3.unit_x;

    const axes_a = obb_a.getAxes();
    const axes_b = obb_b.getAxes();

    const testAxis = struct {
        fn call(axis: Vec3, a: OBB, b: OBB, current_min_overlap: *f32, current_min_axis: *Vec3) bool {
            const len = axis.length();
            if (len < MIN_AXIS_LENGTH) return true;

            const normalized = axis.scale(1.0 / len);
            const proj_a = projectOBBOnAxis(a, normalized);
            const proj_b = projectOBBOnAxis(b, normalized);

            if (!projectionsOverlap(proj_a.min, proj_a.max, proj_b.min, proj_b.max)) {
                return false;
            }

            const overlap = projectionOverlapAmount(proj_a.min, proj_a.max, proj_b.min, proj_b.max);
            if (overlap < current_min_overlap.*) {
                current_min_overlap.* = overlap;

                const center_diff = b.center.sub(a.center);
                if (center_diff.dot(normalized) < 0) {
                    current_min_axis.* = normalized.negate();
                } else {
                    current_min_axis.* = normalized;
                }
            }
            return true;
        }
    }.call;

    for (axes_a) |axis| {
        if (!testAxis(axis, obb_a, obb_b, &min_overlap, &min_axis)) return null;
    }

    for (axes_b) |axis| {
        if (!testAxis(axis, obb_a, obb_b, &min_overlap, &min_axis)) return null;
    }

    for (axes_a) |axis_a| {
        for (axes_b) |axis_b| {
            const cross = axis_a.cross(axis_b);
            if (!testAxis(cross, obb_a, obb_b, &min_overlap, &min_axis)) return null;
        }
    }

    return .{ .axis = min_axis, .depth = min_overlap };
}

/// Result of overlap region calculation
pub const OverlapRegion = struct {
    /// Minimum voxel coordinate in collider A's local space
    min: IVec3,
    /// Maximum voxel coordinate in collider A's local space
    max: IVec3,
    /// Whether any overlap exists
    valid: bool,

    pub fn empty() OverlapRegion {
        return .{
            .min = IVec3.zero,
            .max = IVec3.zero,
            .valid = false,
        };
    }
};

/// Calculate the overlap region between two colliders.
pub fn getOverlapRegion(
    transform_a: Transform,
    collider_a: *const VoxelCollider,
    transform_b: Transform,
    collider_b: *const VoxelCollider,
) OverlapRegion {
    const obb_b = collider_b.getWorldOBB(transform_b);
    const vertices_b = obb_b.getVertices();

    const inv_rot_a = transform_a.rotation.conjugate();
    var t_min = Vec3.splat(std.math.inf(f32));
    var t_max = Vec3.splat(-std.math.inf(f32));

    for (vertices_b) |vert| {
        const relative = inv_rot_a.rotate(vert.sub(transform_a.position));
        t_min = t_min.minComponents(relative);
        t_max = t_max.maxComponents(relative);
    }

    const half_dim_a = collider_a.getDimensions().scale(0.5);
    const voxel_size = collider_a.voxel_size;

    t_min = t_min.sub(Vec3.splat(voxel_size));
    t_max = t_max.add(Vec3.splat(voxel_size));

    const min_clamped = t_min.clamp(half_dim_a.negate(), half_dim_a);
    const max_clamped = t_max.clamp(half_dim_a.negate(), half_dim_a);

    if (min_clamped.x >= max_clamped.x or
        min_clamped.y >= max_clamped.y or
        min_clamped.z >= max_clamped.z)
    {
        return OverlapRegion.empty();
    }

    const min_voxel = collider_a.localToVoxel(min_clamped);
    const max_voxel = collider_a.localToVoxel(max_clamped);

    const bounds_min = collider_a.min_bound;
    const bounds_max = collider_a.max_bound;

    return .{
        .min = IVec3.init(
            @max(min_voxel.x, bounds_min.x),
            @max(min_voxel.y, bounds_min.y),
            @max(min_voxel.z, bounds_min.z),
        ),
        .max = IVec3.init(
            @min(max_voxel.x, bounds_max.x),
            @min(max_voxel.y, bounds_max.y),
            @min(max_voxel.z, bounds_max.z),
        ),
        .valid = true,
    };
}

/// Check if two voxel types should be tested for collision
pub fn shouldCheckCollision(type_a: VoxelType, type_b: VoxelType) bool {
    if (type_a == .empty or type_b == .empty) return false;

    if (type_a == .corner or type_b == .corner) return true;

    if (type_a == .edge and type_b == .edge) return true;

    return false;
}

/// Result of narrow phase collision detection
pub const NarrowPhaseResult = struct {
    /// The contact manifold
    manifold: ContactManifold,
    /// Number of voxel pairs checked
    voxels_checked: u32,
    /// Whether collision was detected
    has_collision: bool,
};

/// Perform narrow-phase collision detection between two voxel bodies.
pub fn detectVoxelCollisions(
    body_a: *const VoxelBody,
    body_b: *const VoxelBody,
    allocator: std.mem.Allocator,
) NarrowPhaseResult {
    _ = allocator;

    const collider_a = body_a.collider orelse return .{
        .manifold = ContactManifold.init(body_a.id, body_b.id, 0.5, 0.0),
        .voxels_checked = 0,
        .has_collision = false,
    };

    const collider_b = body_b.collider orelse return .{
        .manifold = ContactManifold.init(body_a.id, body_b.id, 0.5, 0.0),
        .voxels_checked = 0,
        .has_collision = false,
    };

    const transform_a = body_a.getTransform();
    const transform_b = body_b.getTransform();

    const friction = @sqrt(body_a.properties.friction * body_b.properties.friction);
    const restitution = @max(body_a.properties.restitution, body_b.properties.restitution);

    var result = NarrowPhaseResult{
        .manifold = ContactManifold.init(body_a.id, body_b.id, friction, restitution),
        .voxels_checked = 0,
        .has_collision = false,
    };

    const overlap = getOverlapRegion(transform_a, collider_a, transform_b, collider_b);
    if (!overlap.valid) return result;

    const voxel_size = collider_a.voxel_size;
    const collision_dist_sq = voxel_size * voxel_size * VOXEL_COLLISION_THRESHOLD * VOXEL_COLLISION_THRESHOLD;

    var z: i32 = overlap.min.z;
    while (z <= overlap.max.z) : (z += 1) {
        var y: i32 = overlap.min.y;
        while (y <= overlap.max.y) : (y += 1) {
            var x: i32 = overlap.min.x;
            while (x <= overlap.max.x) : (x += 1) {
                const voxel_a = collider_a.getCollisionVoxel(IVec3.init(x, y, z));
                if (voxel_a.voxel_type == .empty) continue;

                const local_pos_a = collider_a.voxelToLocal(IVec3.init(x, y, z));
                const world_pos_a = transform_a.transformPoint(local_pos_a);

                const local_pos_in_b = transform_b.inverseTransformPoint(world_pos_a);
                const voxel_coord_b = collider_b.localToVoxel(local_pos_in_b);

                checkNeighborhood(
                    voxel_a,
                    IVec3.init(x, y, z),
                    world_pos_a,
                    transform_a,
                    voxel_coord_b,
                    collider_b,
                    transform_b,
                    collision_dist_sq,
                    voxel_size,
                    &result.manifold,
                    &result.voxels_checked,
                );
            }
        }
    }

    result.has_collision = result.manifold.contacts.len > 0;
    return result;
}

/// Check a 3x3x3 neighborhood around a voxel in collider B
fn checkNeighborhood(
    voxel_a: CollisionVoxel,
    voxel_coord_a: IVec3,
    world_pos_a: Vec3,
    transform_a: Transform,
    center_b: IVec3,
    collider_b: *const VoxelCollider,
    transform_b: Transform,
    collision_dist_sq: f32,
    voxel_size: f32,
    manifold: *ContactManifold,
    voxels_checked: *u32,
) void {
    const offsets: [3]i32 = .{ -1, 0, 1 };

    for (offsets) |dz| {
        for (offsets) |dy| {
            for (offsets) |dx| {
                const coord_b = IVec3.init(
                    center_b.x + dx,
                    center_b.y + dy,
                    center_b.z + dz,
                );

                if (!collider_b.isValidCoord(coord_b)) continue;

                const voxel_b = collider_b.getCollisionVoxel(coord_b);
                if (voxel_b.voxel_type == .empty) continue;

                voxels_checked.* += 1;

                if (!shouldCheckCollision(voxel_a.voxel_type, voxel_b.voxel_type)) continue;

                const local_pos_b = collider_b.voxelToLocal(coord_b);
                const world_pos_b = transform_b.transformPoint(local_pos_b);

                const diff = world_pos_b.sub(world_pos_a);
                const dist_sq = diff.lengthSquared();

                const speculative_dist_sq = collision_dist_sq + SPECULATIVE_MARGIN * SPECULATIVE_MARGIN +
                    2.0 * @sqrt(collision_dist_sq) * SPECULATIVE_MARGIN;

                if (dist_sq < speculative_dist_sq) {
                    const contact = generateContact(
                        voxel_a,
                        voxel_coord_a,
                        world_pos_a,
                        transform_a,
                        voxel_b,
                        coord_b,
                        world_pos_b,
                        transform_b,
                        diff,
                        dist_sq,
                        voxel_size,
                    );
                    manifold.addContact(contact);
                }
            }
        }
    }
}

/// Generate a contact point between two colliding voxels
fn generateContact(
    voxel_a: CollisionVoxel,
    voxel_coord_a: IVec3,
    world_pos_a: Vec3,
    transform_a: Transform,
    voxel_b: CollisionVoxel,
    voxel_coord_b: IVec3,
    world_pos_b: Vec3,
    transform_b: Transform,
    diff: Vec3,
    dist_sq: f32,
    voxel_size: f32,
) ContactPoint {
    const dist = @sqrt(dist_sq);

    const contact_point = world_pos_a.add(world_pos_b).scale(0.5);

    var penetration = voxel_size - dist;

    var normal: Vec3 = undefined;

    const lut_normal_a = getNormalVec3(voxel_a.normal_index);
    const world_normal_a = transform_a.rotation.rotate(lut_normal_a);
    const lut_normal_b = getNormalVec3(voxel_b.normal_index);
    const world_normal_b = transform_b.rotation.rotate(lut_normal_b);

    if (voxel_a.voxel_type == .face) {
        normal = world_normal_a;
    } else if (voxel_b.voxel_type == .face) {
        normal = world_normal_b.negate();
    } else if (dist > MIN_AXIS_LENGTH) {
        const dir_normal = diff.scale(1.0 / dist);

        if (dir_normal.dot(world_normal_a) > 0.3) {
            normal = world_normal_a;
        } else if (dir_normal.dot(world_normal_b.negate()) > 0.3) {
            normal = world_normal_b.negate();
        } else {
            normal = dir_normal;
        }
    } else {
        normal = world_normal_a;
    }

    if (voxel_a.voxel_type == .inside or voxel_b.voxel_type == .inside) {
        penetration = @max(penetration, voxel_size);
    }

    const local_anchor_a = transform_a.inverseTransformPoint(contact_point);
    const local_anchor_b = transform_b.inverseTransformPoint(contact_point);

    var contact = ContactPoint.init(
        contact_point,
        normal,
        penetration,
        local_anchor_a,
        local_anchor_b,
    );

    contact.feature_id = ContactPoint.computeFeatureId(voxel_coord_a, voxel_coord_b);

    return contact;
}

/// Result of brick-level collision test
pub const BrickCollisionResult = struct {
    /// Whether the bricks potentially collide
    overlaps: bool,
    /// Minimum penetration depth
    min_penetration: f32,
    /// Penetration axis
    penetration_axis: Vec3,
};

/// Test collision between two bricks using SAT.
pub fn testBrickCollision(
    brick_a: *const CollisionBrick,
    transform_a: Transform,
    voxel_size_a: f32,
    brick_b: *const CollisionBrick,
    transform_b: Transform,
    voxel_size_b: f32,
) BrickCollisionResult {
    if (brick_a.is_empty or brick_b.is_empty) {
        return .{
            .overlaps = false,
            .min_penetration = 0,
            .penetration_axis = Vec3.zero,
        };
    }

    const obb_a = OBB.fromAABB(brick_a.local_aabb.transform(transform_a), Transform.identity);
    const obb_b = OBB.fromAABB(brick_b.local_aabb.transform(transform_b), Transform.identity);

    _ = voxel_size_a;
    _ = voxel_size_b;

    if (satWithPenetration(obb_a, obb_b)) |result| {
        return .{
            .overlaps = true,
            .min_penetration = result.depth,
            .penetration_axis = result.axis,
        };
    }

    return .{
        .overlaps = false,
        .min_penetration = 0,
        .penetration_axis = Vec3.zero,
    };
}

/// Generate a unique key for a body pair
fn pairKey(body_a: u32, body_b: u32) u64 {
    const a = @min(body_a, body_b);
    const b = @max(body_a, body_b);
    return (@as(u64, a) << 32) | @as(u64, b);
}

/// Manages contact manifolds across simulation frames
pub const ContactManager = struct {
    allocator: std.mem.Allocator,

    /// Active manifolds indexed by body pair
    manifolds: std.AutoHashMap(u64, ContactManifold),

    /// Manifolds from previous frame for warm starting
    previous_manifolds: std.AutoHashMap(u64, ContactManifold),

    pub fn init(allocator: std.mem.Allocator) ContactManager {
        return .{
            .allocator = allocator,
            .manifolds = std.AutoHashMap(u64, ContactManifold).init(allocator),
            .previous_manifolds = std.AutoHashMap(u64, ContactManifold).init(allocator),
        };
    }

    pub fn deinit(self: *ContactManager) void {
        self.manifolds.deinit();
        self.previous_manifolds.deinit();
    }

    /// Clear all manifolds for a new frame
    pub fn beginFrame(self: *ContactManager) void {
        const temp = self.previous_manifolds;
        self.previous_manifolds = self.manifolds;
        self.manifolds = temp;
        self.manifolds.clearRetainingCapacity();
    }

    /// Add or update a manifold
    pub fn addManifold(self: *ContactManager, manifold: ContactManifold) !void {
        const key = pairKey(manifold.body_a_id, manifold.body_b_id);

        if (self.previous_manifolds.get(key)) |prev| {
            var new_manifold = manifold;
            warmStartManifold(&new_manifold, &prev);
            try self.manifolds.put(key, new_manifold);
        } else {
            try self.manifolds.put(key, manifold);
        }
    }

    /// Get a manifold for a body pair
    pub fn getManifold(self: *const ContactManager, body_a_id: u32, body_b_id: u32) ?*const ContactManifold {
        const key = pairKey(body_a_id, body_b_id);
        if (self.manifolds.getPtr(key)) |ptr| {
            return ptr;
        }
        return null;
    }

    /// Get mutable manifold for a body pair
    pub fn getManifoldMut(self: *ContactManager, body_a_id: u32, body_b_id: u32) ?*ContactManifold {
        const key = pairKey(body_a_id, body_b_id);
        return self.manifolds.getPtr(key);
    }

    /// Iterator over all active manifolds
    pub fn iterator(self: *const ContactManager) std.AutoHashMap(u64, ContactManifold).ValueIterator {
        return self.manifolds.valueIterator();
    }

    /// Get number of active manifolds
    pub fn count(self: *const ContactManager) usize {
        return self.manifolds.count();
    }

    /// Warm start a manifold from a previous frame's manifold
    fn warmStartManifold(new_manifold: *ContactManifold, old_manifold: *const ContactManifold) void {
        for (new_manifold.contacts.slice()) |*new_contact| {
            for (old_manifold.contacts.constSlice()) |*old_contact| {
                if (new_contact.feature_id == old_contact.feature_id) {
                    new_contact.normal_impulse = old_contact.normal_impulse;
                    new_contact.tangent_impulse = old_contact.tangent_impulse;
                    break;
                }
            }
        }
    }

    /// Merge manifolds from a thread-local buffer
    pub fn mergeThreadLocalManifolds(self: *ContactManager, thread_manifolds: []const ContactManifold) !void {
        for (thread_manifolds) |manifold| {
            if (manifold.contacts.len > 0) {
                try self.addManifold(manifold);
            }
        }
    }
};

/// Thread-local buffer for collecting manifolds during parallel processing
pub const ThreadLocalManifoldBuffer = struct {
    manifolds: [MAX_MANIFOLDS]ContactManifold,
    count: usize,

    /// Statistics for this thread's work
    stats: struct {
        pairs_tested: u32,
        pairs_colliding: u32,
        voxels_checked: u32,
        contacts_generated: u32,
        pairs_skipped_sleeping: u32,
        manifolds_preserved: u32,
        cache_hits: u32,
        cache_misses: u32,
    },

    pub fn init() ThreadLocalManifoldBuffer {
        return .{
            .manifolds = undefined,
            .count = 0,
            .stats = .{
                .pairs_tested = 0,
                .pairs_colliding = 0,
                .voxels_checked = 0,
                .contacts_generated = 0,
                .pairs_skipped_sleeping = 0,
                .manifolds_preserved = 0,
                .cache_hits = 0,
                .cache_misses = 0,
            },
        };
    }

    pub fn addManifold(self: *ThreadLocalManifoldBuffer, manifold: ContactManifold) void {
        if (self.count < MAX_MANIFOLDS) {
            self.manifolds[self.count] = manifold;
            self.count += 1;
        }
    }

    pub fn getManifolds(self: *const ThreadLocalManifoldBuffer) []const ContactManifold {
        return self.manifolds[0..self.count];
    }

    pub fn clear(self: *ThreadLocalManifoldBuffer) void {
        self.count = 0;
        self.stats = .{
            .pairs_tested = 0,
            .pairs_colliding = 0,
            .voxels_checked = 0,
            .contacts_generated = 0,
            .pairs_skipped_sleeping = 0,
            .manifolds_preserved = 0,
            .cache_hits = 0,
            .cache_misses = 0,
        };
    }
};

/// Context for parallel narrow-phase processing
pub const ParallelNarrowPhaseContext = struct {
    /// Thread-local buffers
    thread_buffers: [MAX_WORKER_THREADS]ThreadLocalManifoldBuffer,

    /// Body lookup function data
    bodies: []const *const VoxelBody,

    /// Pairs to process
    pairs: []const BodyPair,

    /// Previous manifolds for warm starting sleeping pairs
    previous_manifolds: *const std.AutoHashMap(u64, ContactManifold),

    /// Read-only cache for lookups
    cache: ?*BrickPairCache,

    /// Config
    config: NarrowPhaseConfig,

    /// Allocator for collision detection
    allocator: std.mem.Allocator,

    pub fn init(
        bodies: []const *const VoxelBody,
        pairs: []const BodyPair,
        previous_manifolds: *const std.AutoHashMap(u64, ContactManifold),
        cache: ?*BrickPairCache,
        config: NarrowPhaseConfig,
        allocator: std.mem.Allocator,
    ) ParallelNarrowPhaseContext {
        var ctx = ParallelNarrowPhaseContext{
            .thread_buffers = undefined,
            .bodies = bodies,
            .pairs = pairs,
            .previous_manifolds = previous_manifolds,
            .cache = cache,
            .config = config,
            .allocator = allocator,
        };
        for (&ctx.thread_buffers) |*buf| {
            buf.* = ThreadLocalManifoldBuffer.init();
        }
        return ctx;
    }

    /// Process a range of pairs for a specific thread
    pub fn processRange(self: *ParallelNarrowPhaseContext, thread_id: usize, start: usize, end: usize) void {
        if (thread_id >= MAX_WORKER_THREADS) return;

        const buffer = &self.thread_buffers[thread_id];

        for (self.pairs[start..end]) |pair| {
            self.processPairIntoBuffer(pair, buffer);
        }
    }

    fn processPairIntoBuffer(self: *ParallelNarrowPhaseContext, pair: BodyPair, buffer: *ThreadLocalManifoldBuffer) void {
        buffer.stats.pairs_tested += 1;

        var body_a: ?*const VoxelBody = null;
        var body_b: ?*const VoxelBody = null;

        for (self.bodies) |body| {
            if (body.id == pair.body_a) body_a = body;
            if (body.id == pair.body_b) body_b = body;
            if (body_a != null and body_b != null) break;
        }

        if (body_a == null or body_b == null) return;

        const both_sleeping = body_a.?.is_sleeping and body_b.?.is_sleeping;
        if (both_sleeping) {
            const key = pairKey(body_a.?.id, body_b.?.id);
            if (self.previous_manifolds.get(key)) |prev_manifold| {
                buffer.stats.pairs_skipped_sleeping += 1;
                buffer.stats.manifolds_preserved += 1;
                buffer.stats.contacts_generated += @intCast(prev_manifold.contacts.len);
                buffer.addManifold(prev_manifold);
                return;
            }
            buffer.stats.pairs_skipped_sleeping += 1;
            return;
        }

        const collider_a = body_a.?.collider orelse return;
        const collider_b = body_b.?.collider orelse return;

        const transform_a = body_a.?.getTransform();
        const transform_b = body_b.?.getTransform();

        if (self.config.use_brick_sat) {
            const obb_a = collider_a.getWorldOBB(transform_a);
            const obb_b = collider_b.getWorldOBB(transform_b);

            if (satEarlyOut(obb_a, obb_b)) {
                return;
            }
        }

        const use_cache = self.config.use_brick_pair_cache and
            self.cache != null and
            (body_a.?.body_type == .static or body_b.?.body_type == .static or
                body_a.?.is_sleeping or body_b.?.is_sleeping);

        if (use_cache) {
            const rel_pos = transform_b.position.sub(transform_a.position);
            const rel_rot = transform_a.rotation.conjugate().mul(transform_b.rotation);

            const cache_key = BrickPairKey.init(
                body_a.?.id,
                body_b.?.id,
                IVec3.zero,
                IVec3.zero,
                rel_pos,
                rel_rot,
            );

            if (self.cache.?.get(cache_key)) |cached| {
                buffer.stats.cache_hits += 1;

                if (cached.has_collision) {
                    const friction = @sqrt(body_a.?.properties.friction * body_b.?.properties.friction);
                    const restitution = @max(body_a.?.properties.restitution, body_b.?.properties.restitution);
                    var manifold = ContactManifold.init(body_a.?.id, body_b.?.id, friction, restitution);

                    for (0..cached.contact_count) |i| {
                        _ = manifold.addContact(cached.contacts[i]);
                    }

                    buffer.stats.pairs_colliding += 1;
                    buffer.stats.contacts_generated += cached.contact_count;
                    buffer.stats.voxels_checked += cached.voxels_checked;
                    buffer.addManifold(manifold);
                }
                return;
            }

            buffer.stats.cache_misses += 1;
        }

        const result = detectVoxelCollisions(body_a.?, body_b.?, self.allocator);

        buffer.stats.voxels_checked += result.voxels_checked;

        if (result.has_collision) {
            buffer.stats.pairs_colliding += 1;
            buffer.stats.contacts_generated += @intCast(result.manifold.contacts.len);
            buffer.addManifold(result.manifold);
        }
    }

    /// Aggregate stats result type
    pub const AggregatedStats = struct {
        pairs_tested: u32,
        pairs_colliding: u32,
        voxels_checked: u32,
        contacts_generated: u32,
        pairs_skipped_sleeping: u32,
        manifolds_preserved: u32,
        cache_hits: u32,
        cache_misses: u32,
    };

    /// Aggregate stats from all thread buffers
    pub fn aggregateStats(self: *const ParallelNarrowPhaseContext, num_threads: usize) AggregatedStats {
        var result = AggregatedStats{
            .pairs_tested = 0,
            .pairs_colliding = 0,
            .voxels_checked = 0,
            .contacts_generated = 0,
            .pairs_skipped_sleeping = 0,
            .manifolds_preserved = 0,
            .cache_hits = 0,
            .cache_misses = 0,
        };

        const count = @min(num_threads, MAX_WORKER_THREADS);
        for (self.thread_buffers[0..count]) |*buf| {
            result.pairs_tested += buf.stats.pairs_tested;
            result.pairs_colliding += buf.stats.pairs_colliding;
            result.voxels_checked += buf.stats.voxels_checked;
            result.contacts_generated += buf.stats.contacts_generated;
            result.pairs_skipped_sleeping += buf.stats.pairs_skipped_sleeping;
            result.manifolds_preserved += buf.stats.manifolds_preserved;
            result.cache_hits += buf.stats.cache_hits;
            result.cache_misses += buf.stats.cache_misses;
        }

        return result;
    }
};

/// Configuration for narrow phase collision detection
pub const NarrowPhaseConfig = struct {
    /// Maximum contacts to generate per pair
    max_contacts_per_pair: usize = MAX_CONTACTS_PER_MANIFOLD,

    /// Collision distance threshold multiplier
    distance_threshold: f32 = VOXEL_COLLISION_THRESHOLD,

    /// Whether to use brick-level early-out
    use_brick_sat: bool = true,

    /// Whether to use voxel type filtering
    use_type_filtering: bool = true,

    /// Whether to use brick-pair caching for static/sleeping pairs
    use_brick_pair_cache: bool = true,

    /// Brick-pair cache capacity
    brick_pair_cache_size: usize = MAX_BRICK_PAIR_CACHE_SIZE,
};

/// Narrow phase collision detection dispatcher
pub const NarrowPhase = struct {
    allocator: std.mem.Allocator,
    config: NarrowPhaseConfig,
    contact_manager: ContactManager,

    /// Brick-pair cache for static/sleeping pairs
    brick_pair_cache: ?BrickPairCache,

    /// Statistics
    stats: struct {
        pairs_tested: u32 = 0,
        pairs_colliding: u32 = 0,
        voxels_checked: u32 = 0,
        contacts_generated: u32 = 0,
        pairs_skipped_sleeping: u32 = 0,
        manifolds_preserved: u32 = 0,
        pairs_parallel: u32 = 0,
        pairs_sequential: u32 = 0,
        cache_hits: u32 = 0,
        cache_misses: u32 = 0,
    } = .{},

    pub fn init(allocator: std.mem.Allocator, config: NarrowPhaseConfig) NarrowPhase {
        var cache: ?BrickPairCache = null;
        if (config.use_brick_pair_cache) {
            cache = BrickPairCache.init(allocator, config.brick_pair_cache_size) catch null;
        }

        return .{
            .allocator = allocator,
            .config = config,
            .contact_manager = ContactManager.init(allocator),
            .brick_pair_cache = cache,
        };
    }

    pub fn deinit(self: *NarrowPhase) void {
        if (self.brick_pair_cache) |*cache| {
            cache.deinit();
        }
        self.contact_manager.deinit();
    }

    /// Begin a new collision detection frame
    pub fn beginFrame(self: *NarrowPhase) void {
        self.contact_manager.beginFrame();
        if (self.brick_pair_cache) |*cache| {
            cache.beginFrame();
        }
        self.stats = .{};
    }

    /// Invalidate cache for a body that has moved or changed
    pub fn invalidateCacheForBody(self: *NarrowPhase, body_id: u32) void {
        if (self.brick_pair_cache) |*cache| {
            cache.invalidateBody(body_id);
        }
    }

    /// Get cache statistics
    pub fn getCacheHitRate(self: *const NarrowPhase) f32 {
        if (self.brick_pair_cache) |*cache| {
            return cache.getHitRate();
        }
        return 0.0;
    }

    /// Process a collision pair
    pub fn processPair(
        self: *NarrowPhase,
        body_a: *const VoxelBody,
        body_b: *const VoxelBody,
    ) !void {
        self.stats.pairs_tested += 1;

        const both_sleeping = body_a.is_sleeping and body_b.is_sleeping;
        if (both_sleeping) {
            const key = pairKey(body_a.id, body_b.id);
            if (self.contact_manager.previous_manifolds.get(key)) |prev_manifold| {
                self.stats.pairs_skipped_sleeping += 1;
                self.stats.manifolds_preserved += 1;
                self.stats.contacts_generated += @intCast(prev_manifold.contacts.len);
                try self.contact_manager.manifolds.put(key, prev_manifold);
                return;
            }

            self.stats.pairs_skipped_sleeping += 1;
            return;
        }

        const collider_a = body_a.collider orelse return;
        const collider_b = body_b.collider orelse return;

        const transform_a = body_a.getTransform();
        const transform_b = body_b.getTransform();

        if (self.config.use_brick_sat) {
            const obb_a = collider_a.getWorldOBB(transform_a);
            const obb_b = collider_b.getWorldOBB(transform_b);

            if (satEarlyOut(obb_a, obb_b)) {
                return;
            }
        }

        const use_cache = self.config.use_brick_pair_cache and
            self.brick_pair_cache != null and
            (body_a.body_type == .static or body_b.body_type == .static or
                body_a.is_sleeping or body_b.is_sleeping);

        if (use_cache) {
            const rel_pos = transform_b.position.sub(transform_a.position);
            const rel_rot = transform_a.rotation.conjugate().mul(transform_b.rotation);

            const cache_key = BrickPairKey.init(
                body_a.id,
                body_b.id,
                IVec3.zero,
                IVec3.zero,
                rel_pos,
                rel_rot,
            );

            if (self.brick_pair_cache.?.get(cache_key)) |cached| {
                self.stats.cache_hits += 1;

                if (cached.has_collision) {
                    const friction = @sqrt(body_a.properties.friction * body_b.properties.friction);
                    const restitution = @max(body_a.properties.restitution, body_b.properties.restitution);
                    var manifold = ContactManifold.init(body_a.id, body_b.id, friction, restitution);

                    for (0..cached.contact_count) |i| {
                        _ = manifold.addContact(cached.contacts[i]);
                    }

                    self.stats.pairs_colliding += 1;
                    self.stats.contacts_generated += cached.contact_count;
                    self.stats.voxels_checked += cached.voxels_checked;
                    try self.contact_manager.addManifold(manifold);
                }
                return;
            }

            self.stats.cache_misses += 1;

            const result = detectVoxelCollisions(body_a, body_b, self.allocator);

            var cached_contacts = CachedBrickContacts.init();
            cached_contacts.has_collision = result.has_collision;
            cached_contacts.voxels_checked = result.voxels_checked;

            if (result.has_collision) {
                const contact_count = @min(result.manifold.contacts.len, MAX_CONTACTS_PER_MANIFOLD);
                cached_contacts.contact_count = @intCast(contact_count);
                for (0..contact_count) |i| {
                    cached_contacts.contacts[i] = result.manifold.contacts.buffer[i];
                }
            }

            self.brick_pair_cache.?.put(cache_key, cached_contacts);

            self.stats.voxels_checked += result.voxels_checked;

            if (result.has_collision) {
                self.stats.pairs_colliding += 1;
                self.stats.contacts_generated += @intCast(result.manifold.contacts.len);
                try self.contact_manager.addManifold(result.manifold);
            }
        } else {
            const result = detectVoxelCollisions(body_a, body_b, self.allocator);

            self.stats.voxels_checked += result.voxels_checked;

            if (result.has_collision) {
                self.stats.pairs_colliding += 1;
                self.stats.contacts_generated += @intCast(result.manifold.contacts.len);
                try self.contact_manager.addManifold(result.manifold);
            }
        }
    }

    /// Get the contact manager
    pub fn getContactManager(self: *const NarrowPhase) *const ContactManager {
        return &self.contact_manager;
    }

    /// Get mutable contact manager
    pub fn getContactManagerMut(self: *NarrowPhase) *ContactManager {
        return &self.contact_manager;
    }

    /// Process collision pairs in parallel using thread pool
    pub fn processPairsParallel(
        self: *NarrowPhase,
        thread_pool: *ThreadPool,
        pairs: []const BodyPair,
        bodies: []const *const VoxelBody,
    ) !void {
        const num_pairs = pairs.len;

        if (num_pairs < MIN_PARALLEL_PAIRS or thread_pool.threadCount() <= 1) {
            self.stats.pairs_sequential += @intCast(num_pairs);
            for (pairs) |pair| {
                var body_a: ?*const VoxelBody = null;
                var body_b: ?*const VoxelBody = null;
                for (bodies) |body| {
                    if (body.id == pair.body_a) body_a = body;
                    if (body.id == pair.body_b) body_b = body;
                    if (body_a != null and body_b != null) break;
                }
                if (body_a != null and body_b != null) {
                    try self.processPair(body_a.?, body_b.?);
                }
            }
            return;
        }

        self.stats.pairs_parallel += @intCast(num_pairs);

        var ctx = ParallelNarrowPhaseContext.init(
            bodies,
            pairs,
            &self.contact_manager.previous_manifolds,
            if (self.brick_pair_cache) |*c| c else null,
            self.config,
            self.allocator,
        );

        const num_threads = @min(thread_pool.threadCount(), MAX_WORKER_THREADS);
        const pairs_per_thread = (num_pairs + num_threads - 1) / num_threads;

        var task_contexts: [MAX_WORKER_THREADS]struct {
            ctx: *ParallelNarrowPhaseContext,
            thread_id: usize,
            start: usize,
            end: usize,
        } = undefined;

        var submitted: usize = 0;
        var start: usize = 0;

        while (start < num_pairs and submitted < num_threads) {
            const end = @min(start + pairs_per_thread, num_pairs);
            task_contexts[submitted] = .{
                .ctx = &ctx,
                .thread_id = submitted,
                .start = start,
                .end = end,
            };

            const task_ptr = &task_contexts[submitted];
            const success = thread_pool.submit(.{
                .func = struct {
                    fn process(ptr: *anyopaque) void {
                        const tc: *@TypeOf(task_ptr.*) = @ptrCast(@alignCast(ptr));
                        tc.ctx.processRange(tc.thread_id, tc.start, tc.end);
                    }
                }.process,
                .data = @ptrCast(task_ptr),
            });

            if (!success) {
                ctx.processRange(submitted, start, num_pairs);
                submitted += 1;
                break;
            }

            submitted += 1;
            start = end;
        }

        thread_pool.waitAll();

        for (ctx.thread_buffers[0..submitted]) |*buf| {
            try self.contact_manager.mergeThreadLocalManifolds(buf.getManifolds());
        }

        const aggregated = ctx.aggregateStats(submitted);
        self.stats.pairs_tested += aggregated.pairs_tested;
        self.stats.pairs_colliding += aggregated.pairs_colliding;
        self.stats.voxels_checked += aggregated.voxels_checked;
        self.stats.contacts_generated += aggregated.contacts_generated;
        self.stats.pairs_skipped_sleeping += aggregated.pairs_skipped_sleeping;
        self.stats.manifolds_preserved += aggregated.manifolds_preserved;
        self.stats.cache_hits += aggregated.cache_hits;
        self.stats.cache_misses += aggregated.cache_misses;
    }
};

test "ContactPoint feature ID" {
    const id1 = ContactPoint.computeFeatureId(
        IVec3.init(1, 2, 3),
        IVec3.init(4, 5, 6),
    );
    const id2 = ContactPoint.computeFeatureId(
        IVec3.init(1, 2, 3),
        IVec3.init(4, 5, 6),
    );
    const id3 = ContactPoint.computeFeatureId(
        IVec3.init(1, 2, 4),
        IVec3.init(4, 5, 6),
    );

    try std.testing.expectEqual(id1, id2);
    try std.testing.expect(id1 != id3);
}

test "SAT early out - separated OBBs" {
    const obb_a = OBB.init(
        Vec3.init(0, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.identity,
    );
    const obb_b = OBB.init(
        Vec3.init(5, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.identity,
    );

    try std.testing.expect(satEarlyOut(obb_a, obb_b));
}

test "SAT early out - overlapping OBBs" {
    const obb_a = OBB.init(
        Vec3.init(0, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.identity,
    );
    const obb_b = OBB.init(
        Vec3.init(1.5, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.identity,
    );

    try std.testing.expect(!satEarlyOut(obb_a, obb_b));
}

test "SAT with penetration" {
    const obb_a = OBB.init(
        Vec3.init(0, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.identity,
    );
    const obb_b = OBB.init(
        Vec3.init(1.5, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.identity,
    );

    const result = satWithPenetration(obb_a, obb_b);
    try std.testing.expect(result != null);

    if (result) |r| {
        try std.testing.expect(r.depth > 0);
        try std.testing.expectApproxEqAbs(@as(f32, 0.5), r.depth, 0.01);
    }
}

test "SAT with penetration - separated" {
    const obb_a = OBB.init(
        Vec3.init(0, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.identity,
    );
    const obb_b = OBB.init(
        Vec3.init(5, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.identity,
    );

    const result = satWithPenetration(obb_a, obb_b);
    try std.testing.expect(result == null);
}

test "SAT with rotated OBBs" {
    const obb_a = OBB.init(
        Vec3.init(0, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.identity,
    );

    const obb_b = OBB.init(
        Vec3.init(2, 0, 0),
        Vec3.init(1, 1, 1),
        Quat.fromAxisAngle(Vec3.unit_y, std.math.pi / 4.0),
    );

    try std.testing.expect(!satEarlyOut(obb_a, obb_b));
}

test "shouldCheckCollision" {
    try std.testing.expect(shouldCheckCollision(.corner, .corner));
    try std.testing.expect(shouldCheckCollision(.corner, .edge));
    try std.testing.expect(shouldCheckCollision(.corner, .face));
    try std.testing.expect(shouldCheckCollision(.corner, .inside));

    try std.testing.expect(shouldCheckCollision(.edge, .corner));
    try std.testing.expect(shouldCheckCollision(.edge, .edge));
    try std.testing.expect(!shouldCheckCollision(.edge, .face));
    try std.testing.expect(!shouldCheckCollision(.edge, .inside));

    try std.testing.expect(shouldCheckCollision(.face, .corner));
    try std.testing.expect(!shouldCheckCollision(.face, .edge));
    try std.testing.expect(!shouldCheckCollision(.face, .face));

    try std.testing.expect(!shouldCheckCollision(.empty, .corner));
    try std.testing.expect(!shouldCheckCollision(.corner, .empty));
}

test "ContactManifold add and replace" {
    var manifold = ContactManifold.init(1, 2, 0.5, 0.3);

    for (0..MAX_CONTACTS_PER_MANIFOLD) |i| {
        var contact = ContactPoint.init(
            Vec3.init(@floatFromInt(i), 0, 0),
            Vec3.unit_y,
            0.1 * @as(f32, @floatFromInt(i)),
            Vec3.zero,
            Vec3.zero,
        );

        contact.feature_id = @intCast(i + 1);
        manifold.addContact(contact);
    }

    try std.testing.expectEqual(MAX_CONTACTS_PER_MANIFOLD, manifold.contacts.len);

    var deep_contact = ContactPoint.init(
        Vec3.init(100, 0, 0),
        Vec3.unit_y,
        10.0,
        Vec3.zero,
        Vec3.zero,
    );

    deep_contact.feature_id = 999;
    manifold.addContact(deep_contact);

    try std.testing.expectEqual(MAX_CONTACTS_PER_MANIFOLD, manifold.contacts.len);

    var found = false;
    for (manifold.contacts.slice()) |c| {
        if (c.penetration == 10.0) {
            found = true;
            break;
        }
    }
    try std.testing.expect(found);
}

test "ContactManager warm starting" {
    const allocator = std.testing.allocator;
    var manager = ContactManager.init(allocator);
    defer manager.deinit();

    var manifold1 = ContactManifold.init(1, 2, 0.5, 0.3);
    var contact1 = ContactPoint.init(Vec3.zero, Vec3.unit_y, 0.1, Vec3.zero, Vec3.zero);
    contact1.feature_id = 12345;
    contact1.normal_impulse = 50.0;
    manifold1.addContact(contact1);

    try manager.addManifold(manifold1);

    manager.beginFrame();

    var manifold2 = ContactManifold.init(1, 2, 0.5, 0.3);
    var contact2 = ContactPoint.init(Vec3.init(0.01, 0, 0), Vec3.unit_y, 0.11, Vec3.zero, Vec3.zero);
    contact2.feature_id = 12345;
    contact2.normal_impulse = 0.0;
    manifold2.addContact(contact2);

    try manager.addManifold(manifold2);

    const retrieved = manager.getManifold(1, 2);
    try std.testing.expect(retrieved != null);
    if (retrieved) |m| {
        try std.testing.expectEqual(@as(usize, 1), m.contacts.len);
        try std.testing.expectApproxEqAbs(@as(f32, 50.0), m.contacts.buffer[0].normal_impulse, 0.001);
    }
}

test "NarrowPhase init and deinit" {
    const allocator = std.testing.allocator;
    var narrow = NarrowPhase.init(allocator, .{});
    defer narrow.deinit();

    narrow.beginFrame();
    try std.testing.expectEqual(@as(u32, 0), narrow.stats.pairs_tested);
}

test "ThreadLocalManifoldBuffer basic operations" {
    var buffer = ThreadLocalManifoldBuffer.init();

    try std.testing.expectEqual(@as(usize, 0), buffer.count);
    try std.testing.expectEqual(@as(usize, 0), buffer.getManifolds().len);

    const manifold = ContactManifold.init(1, 2, 0.5, 0.3);
    buffer.addManifold(manifold);

    try std.testing.expectEqual(@as(usize, 1), buffer.count);
    try std.testing.expectEqual(@as(usize, 1), buffer.getManifolds().len);

    buffer.clear();
    try std.testing.expectEqual(@as(usize, 0), buffer.count);
}

test "ParallelNarrowPhaseContext stats aggregation" {
    const allocator = std.testing.allocator;
    var prev_manifolds = std.AutoHashMap(u64, ContactManifold).init(allocator);
    defer prev_manifolds.deinit();

    const bodies: []const *const VoxelBody = &.{};
    const pairs: []const BodyPair = &.{};

    var ctx = ParallelNarrowPhaseContext.init(
        bodies,
        pairs,
        &prev_manifolds,
        null,
        .{},
        allocator,
    );

    ctx.thread_buffers[0].stats.pairs_tested = 10;
    ctx.thread_buffers[0].stats.pairs_colliding = 3;
    ctx.thread_buffers[1].stats.pairs_tested = 15;
    ctx.thread_buffers[1].stats.pairs_colliding = 5;

    const aggregated = ctx.aggregateStats(2);
    try std.testing.expectEqual(@as(u32, 25), aggregated.pairs_tested);
    try std.testing.expectEqual(@as(u32, 8), aggregated.pairs_colliding);
}

test "OverlapRegion empty" {
    const region = OverlapRegion.empty();
    try std.testing.expect(!region.valid);
}

test "BrickPairCache basic operations" {
    const allocator = std.testing.allocator;
    var cache = try BrickPairCache.init(allocator, 64);
    defer cache.deinit();

    const key = BrickPairKey.init(
        1,
        2,
        IVec3.init(0, 0, 0),
        IVec3.init(1, 0, 0),
        Vec3.init(1.0, 0.0, 0.0),
        Quat.identity,
    );

    try std.testing.expect(cache.get(key) == null);
    try std.testing.expectEqual(@as(u64, 1), cache.stats.misses);
    try std.testing.expectEqual(@as(u64, 0), cache.stats.hits);

    var contacts = CachedBrickContacts.init();
    contacts.has_collision = true;
    contacts.contact_count = 1;
    cache.put(key, contacts);

    const cached = cache.get(key);
    try std.testing.expect(cached != null);
    try std.testing.expect(cached.?.has_collision);
    try std.testing.expectEqual(@as(u8, 1), cached.?.contact_count);
    try std.testing.expectEqual(@as(u64, 1), cache.stats.hits);
}

test "BrickPairCache invalidation" {
    const allocator = std.testing.allocator;
    var cache = try BrickPairCache.init(allocator, 64);
    defer cache.deinit();

    const key1 = BrickPairKey.init(1, 2, IVec3.zero, IVec3.zero, Vec3.zero, Quat.identity);
    const key2 = BrickPairKey.init(1, 3, IVec3.zero, IVec3.zero, Vec3.zero, Quat.identity);
    const key3 = BrickPairKey.init(2, 3, IVec3.zero, IVec3.zero, Vec3.zero, Quat.identity);

    cache.put(key1, CachedBrickContacts.init());
    cache.put(key2, CachedBrickContacts.init());
    cache.put(key3, CachedBrickContacts.init());

    try std.testing.expect(cache.get(key1) != null);
    try std.testing.expect(cache.get(key2) != null);
    try std.testing.expect(cache.get(key3) != null);

    cache.invalidateBody(1);

    try std.testing.expect(cache.get(key1) == null);
    try std.testing.expect(cache.get(key2) == null);

    try std.testing.expect(cache.get(key3) != null);
}

test "BrickPairKey transform sensitivity" {
    const key1 = BrickPairKey.init(
        1,
        2,
        IVec3.zero,
        IVec3.zero,
        Vec3.init(0.0, 0.0, 0.0),
        Quat.identity,
    );

    const key2 = BrickPairKey.init(
        1,
        2,
        IVec3.zero,
        IVec3.zero,
        Vec3.init(0.1, 0.0, 0.0),
        Quat.identity,
    );

    try std.testing.expect(!key1.eql(key2));

    const key3 = BrickPairKey.init(
        1,
        2,
        IVec3.zero,
        IVec3.zero,
        Vec3.init(0.0, 0.0, 0.0),
        Quat.identity,
    );
    try std.testing.expect(key1.eql(key3));
}

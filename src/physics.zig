const std = @import("std");

pub const math = @import("math.zig");

/// 3D vector for positions, velocities, forces
pub const Vec3 = math.Vec3;

/// Integer 3D vector for voxel coordinates
pub const IVec3 = math.IVec3;

/// Quaternion for rotations
pub const Quat = math.Quat;

/// 3x3 matrix for inertia tensors
pub const Mat3 = math.Mat3;

/// Position + rotation transform
pub const Transform = math.Transform;

/// Axis-aligned bounding box
pub const AABB = math.AABB;

/// Oriented bounding box
pub const OBB = math.OBB;

/// Math constants
pub const PI = math.PI;
pub const EPSILON = math.EPSILON;

/// Utility functions
pub const integrateRotation = math.integrateRotation;

pub const collision_voxel = @import("collision_voxel.zig");

/// Voxel classification
pub const VoxelType = collision_voxel.VoxelType;

/// Collision data for a single voxel
pub const CollisionVoxel = collision_voxel.CollisionVoxel;

/// Normal direction lookup table
pub const NORMAL_LUT = collision_voxel.NORMAL_LUT;
pub const NUM_NORMALS = collision_voxel.NUM_NORMALS;

/// Normal lookup functions
pub const getNormalVec3 = collision_voxel.getNormalVec3;
pub const findClosestNormalIndex = collision_voxel.findClosestNormalIndex;
pub const computeNormalIndex = collision_voxel.computeNormalIndex;

/// Voxel classification functions
pub const classifyVoxel = collision_voxel.classifyVoxel;
pub const classifyFromNeighbors = collision_voxel.classifyFromNeighbors;

pub const collider = @import("collider.zig");

/// Collision brick
pub const CollisionBrick = collider.CollisionBrick;

/// Voxel collider
pub const VoxelCollider = collider.VoxelCollider;

/// Collider constants
pub const BRICK_SIZE = collider.BRICK_SIZE;
pub const BRICK_VOLUME = collider.BRICK_VOLUME;
pub const DEFAULT_VOXEL_SIZE = collider.DEFAULT_VOXEL_SIZE;

pub const body = @import("body.zig");

/// Body type
pub const BodyType = body.BodyType;

/// Hot motion data
pub const BodyMotionState = body.BodyMotionState;

/// Cold property data
pub const BodyProperties = body.BodyProperties;

/// Voxel physics body
pub const VoxelBody = body.VoxelBody;

/// Mass property computation result
pub const MassProperties = body.MassProperties;

/// Compute mass properties from collider
pub const computeMassProperties = body.computeMassProperties;

/// Body constants
pub const DEFAULT_LINEAR_DAMPING = body.DEFAULT_LINEAR_DAMPING;
pub const DEFAULT_ANGULAR_DAMPING = body.DEFAULT_ANGULAR_DAMPING;
pub const SLEEP_LINEAR_THRESHOLD = body.SLEEP_LINEAR_THRESHOLD;
pub const SLEEP_ANGULAR_THRESHOLD = body.SLEEP_ANGULAR_THRESHOLD;
pub const SLEEP_TIME_THRESHOLD = body.SLEEP_TIME_THRESHOLD;

pub const broad_phase = @import("broad_phase.zig");

/// Potential collision pair
pub const BodyPair = broad_phase.BodyPair;

/// AABB tree node
pub const TreeNode = broad_phase.TreeNode;

/// Dynamic AABB tree
pub const AABBTree = broad_phase.AABBTree;

/// Broad phase collision detection
pub const BroadPhase = broad_phase.BroadPhase;

/// Broad phase constants
pub const NULL_NODE = broad_phase.NULL_NODE;
pub const AABB_EXTENSION = broad_phase.AABB_EXTENSION;
pub const AABB_VELOCITY_MULTIPLIER = broad_phase.AABB_VELOCITY_MULTIPLIER;

pub const narrow_phase = @import("narrow_phase.zig");

/// Contact point between two bodies
pub const ContactPoint = narrow_phase.ContactPoint;

/// Contact manifold
pub const ContactManifold = narrow_phase.ContactManifold;

/// Overlap region calculation result
pub const OverlapRegion = narrow_phase.OverlapRegion;

/// Narrow phase collision result
pub const NarrowPhaseResult = narrow_phase.NarrowPhaseResult;

/// Brick collision test result
pub const BrickCollisionResult = narrow_phase.BrickCollisionResult;

/// Contact manager for manifold persistence
pub const ContactManager = narrow_phase.ContactManager;

/// Narrow phase configuration
pub const NarrowPhaseConfig = narrow_phase.NarrowPhaseConfig;

/// Narrow phase collision detection
pub const NarrowPhase = narrow_phase.NarrowPhase;

/// SAT early-out test for OBBs
pub const satEarlyOut = narrow_phase.satEarlyOut;

/// SAT test with penetration depth
pub const satWithPenetration = narrow_phase.satWithPenetration;

/// Get overlap region between two colliders
pub const getOverlapRegion = narrow_phase.getOverlapRegion;

/// Detect voxel collisions between two bodies
pub const detectVoxelCollisions = narrow_phase.detectVoxelCollisions;

/// Test brick collision
pub const testBrickCollision = narrow_phase.testBrickCollision;

/// Check if two voxel types should collide
pub const shouldCheckCollision = narrow_phase.shouldCheckCollision;

/// Narrow phase constants
pub const MAX_CONTACTS_PER_MANIFOLD = narrow_phase.MAX_CONTACTS_PER_MANIFOLD;
pub const MAX_MANIFOLDS = narrow_phase.MAX_MANIFOLDS;
pub const VOXEL_COLLISION_THRESHOLD = narrow_phase.VOXEL_COLLISION_THRESHOLD;
pub const PENETRATION_SLOP = narrow_phase.PENETRATION_SLOP;

pub const solver = @import("solver.zig");

/// Contact solver
pub const ContactSolver = solver.ContactSolver;

/// Solver configuration
pub const SolverConfig = solver.SolverConfig;

/// Soft constraint parameters
pub const SoftParams = solver.SoftParams;

/// Compute soft constraint parameters from spring parameters
pub const computeSoftParams = solver.computeSoftParams;

/// Solver statistics
pub const SolverStats = solver.ContactSolver.SolverStats;

/// Solver constants
pub const MAX_VELOCITY_ITERATIONS = solver.MAX_VELOCITY_ITERATIONS;
pub const MAX_POSITION_ITERATIONS = solver.MAX_POSITION_ITERATIONS;
pub const DEFAULT_SUBSTEPS = solver.DEFAULT_SUBSTEPS;
pub const MIN_SEPARATION = solver.MIN_SEPARATION;
pub const RESTITUTION_THRESHOLD = solver.RESTITUTION_THRESHOLD;

pub const system = @import("system.zig");

/// Unified physics system
pub const VoxelPhysicsSystem = system.VoxelPhysicsSystem;

/// Physics configuration
pub const PhysicsConfig = system.PhysicsConfig;

/// Physics statistics
pub const PhysicsStats = system.PhysicsStats;

/// Body handle for stable references
pub const BodyHandle = system.BodyHandle;

pub const simd = @import("simd.zig");

/// 4-wide float vector for SIMD operations
pub const Vec4f = simd.Vec4f;

/// 4-wide integer vector
pub const Vec4i = simd.Vec4i;

/// 4-wide boolean mask
pub const Vec4b = simd.Vec4b;

/// SIMD AABB overlap test
pub const simdAABBTest = simd.simdAABBTest;

/// SIMD distance check
pub const simdDistanceCheck = simd.simdDistanceCheck;

/// SOA layout for AABB bounds
pub const AABBBoundsSOA = simd.AABBBoundsSOA;

/// SOA layout for voxel positions
pub const VoxelPositionsSOA = simd.VoxelPositionsSOA;

/// SIMD width constant
pub const SIMD_WIDTH = simd.SIMD_WIDTH;

pub const thread_pool = @import("thread_pool.zig");

/// Thread pool for parallel physics processing
pub const ThreadPool = thread_pool.ThreadPool;

/// Task for thread pool execution
pub const Task = thread_pool.Task;

/// Index range for parallel for operations
pub const IndexRange = thread_pool.IndexRange;

/// Parallel for-each execution
pub const parallelFor = thread_pool.parallelFor;

/// Parallel for-range execution
pub const parallelForRange = thread_pool.parallelForRange;

/// Parallel accumulator for reduce operations
pub const ParallelAccumulator = thread_pool.ParallelAccumulator;

pub const islands = @import("islands.zig");

/// Union-Find data structure for efficient grouping
pub const UnionFind = islands.UnionFind;

/// Island of connected bodies
pub const Island = islands.Island;

/// Island manager for detection and tracking
pub const IslandManager = islands.IslandManager;

/// Graph coloring for parallel contact solving
pub const GraphColoring = islands.GraphColoring;

/// Solve islands in parallel
pub const solveIslandsParallel = islands.solveIslandsParallel;

/// Island configuration constants
pub const MIN_PARALLEL_ISLAND_SIZE = islands.MIN_PARALLEL_ISLAND_SIZE;
pub const MAX_ISLANDS = islands.MAX_ISLANDS;

pub const version = struct {
    pub const major = 0;
    pub const minor = 5;
    pub const patch = 0;
    pub const string = "0.5.0-dev";
};

test {
    std.testing.refAllDecls(@This());
    _ = math;
    _ = collision_voxel;
    _ = collider;
    _ = body;
    _ = broad_phase;
    _ = narrow_phase;
    _ = solver;
    _ = system;
    _ = simd;
    _ = thread_pool;
    _ = islands;
}

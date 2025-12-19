const std = @import("std");

pub const math = @import("math.zig");
pub const collider = @import("collider.zig");
pub const collision_voxel = @import("collision_voxel.zig");
pub const body = @import("body.zig");
pub const solver = @import("solver.zig");
pub const system = @import("system.zig");
pub const broad_phase = @import("broad_phase.zig");
pub const narrow_phase = @import("narrow_phase.zig");
pub const islands = @import("islands.zig");
pub const simd = @import("simd.zig");
pub const thread_pool = @import("thread_pool.zig");

pub const VoxelPhysicsSystem = system.VoxelPhysicsSystem;
pub const BodyHandle = system.BodyHandle;
pub const PhysicsConfig = system.PhysicsConfig;
pub const PhysicsStats = system.PhysicsStats;
pub const VoxelCollider = collider.VoxelCollider;
pub const SolverConfig = solver.SolverConfig;

test {
    std.testing.refAllDecls(@This());
}

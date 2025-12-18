const std = @import("std");
const physics = @import("zig-voxel-physics");

const Vec3 = physics.Vec3;
const Quat = physics.Quat;
const IVec3 = physics.IVec3;
const VoxelCollider = physics.VoxelCollider;
const CollisionVoxel = physics.CollisionVoxel;
const VoxelType = physics.VoxelType;
const VoxelPhysicsSystem = physics.VoxelPhysicsSystem;
const PhysicsConfig = physics.PhysicsConfig;
const SolverConfig = physics.SolverConfig;

const VOXEL_SIZE: f32 = 0.2;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const config = PhysicsConfig{
        .gravity = Vec3.init(0, -9.81, 0),
        .fixed_timestep = 1.0 / 60.0,
        .solver = SolverConfig{
            .substeps = 4,
            .velocity_iterations = 2,
            .position_iterations = 4,
            .warm_starting = true,
        },
        .enable_sleeping = true,
    };

    var system = try VoxelPhysicsSystem.init(allocator, config);
    defer system.deinit();

    const ground_collider = try allocator.create(VoxelCollider);
    ground_collider.* = VoxelCollider.init(allocator, VOXEL_SIZE);
    try createGroundVoxels(ground_collider);
    ground_collider.recomputeMetadata();

    const ground_handle = try system.createStaticBody(
        Vec3.init(0, -VOXEL_SIZE * 0.5, 0),
        Quat.identity,
        ground_collider,
    );
    system.setColliderOwnership(ground_handle, true);

    const box_collider = try allocator.create(VoxelCollider);
    box_collider.* = VoxelCollider.init(allocator, VOXEL_SIZE);
    try createBoxVoxels(box_collider);
    box_collider.recomputeMetadata();

    const box_handle = try system.createDynamicBody(
        Vec3.init(0, 3, 0),
        Quat.identity,
        box_collider,
        1.0,
    );
    system.setColliderOwnership(box_handle, true);

    const dt: f32 = 1.0 / 60.0;
    var frame: u32 = 0;
    const max_frames: u32 = 600;

    std.debug.print("\nSimulating...\n", .{});

    while (frame < max_frames) : (frame += 1) {
        system.update(dt);

        if (system.getBodyConst(box_handle)) |body| {
            if (body.is_sleeping) {
                std.debug.print("\nBox went to sleep after {d} frames\n", .{frame});
                break;
            }
        }
    }

    if (system.getBodyConst(box_handle)) |body| {
        const pos = body.getPosition();
        const vel = body.getLinearVelocity();

        std.debug.print("\n=== Results ===\n", .{});
        std.debug.print("Position: ({d:.3}, {d:.3}, {d:.3})\n", .{ pos.x, pos.y, pos.z });
        std.debug.print("Velocity: ({d:.3}, {d:.3}, {d:.3})\n", .{ vel.x, vel.y, vel.z });
        std.debug.print("Sleeping: {}\n", .{body.is_sleeping});
    }

    std.debug.print("\nSimulation complete!\n", .{});
}

fn createGroundVoxels(collider: *VoxelCollider) !void {
    const size: i32 = 16;
    const half = size / 2;

    var z: i32 = -half;
    while (z < half) : (z += 1) {
        var x: i32 = -half;
        while (x < half) : (x += 1) {
            const on_x_edge = (x == -half or x == half - 1);
            const on_z_edge = (z == -half or z == half - 1);

            var vtype: VoxelType = .face;
            var normal: u5 = 2;

            if (on_x_edge and on_z_edge) {
                vtype = .corner;
                if (x < 0 and z < 0) normal = 23;
                if (x > 0 and z < 0) normal = 19;
                if (x < 0 and z > 0) normal = 22;
                if (x > 0 and z > 0) normal = 18;
            } else if (on_x_edge) {
                vtype = .edge;
                normal = if (x < 0) 8 else 6;
            } else if (on_z_edge) {
                vtype = .edge;
                normal = if (z < 0) 15 else 14;
            }

            try collider.setCollisionVoxel(IVec3.init(x, 0, z), CollisionVoxel.init(vtype, normal));
        }
    }
}

fn createBoxVoxels(collider: *VoxelCollider) !void {
    var z: i32 = -1;
    while (z <= 1) : (z += 1) {
        var y: i32 = -1;
        while (y <= 1) : (y += 1) {
            var x: i32 = -1;
            while (x <= 1) : (x += 1) {
                const on_x = (x == -1 or x == 1);
                const on_y = (y == -1 or y == 1);
                const on_z = (z == -1 or z == 1);

                const edges = @as(u8, @intFromBool(on_x)) + @as(u8, @intFromBool(on_y)) + @as(u8, @intFromBool(on_z));

                const vtype: VoxelType = switch (edges) {
                    3 => .corner,
                    2 => .edge,
                    1 => .face,
                    else => .inside,
                };

                var normal: u5 = 0;
                if (on_x and x > 0) normal = 0;
                if (on_x and x < 0) normal = 1;
                if (on_y and y > 0) normal = 2;
                if (on_y and y < 0) normal = 3;
                if (on_z and z > 0) normal = 4;
                if (on_z and z < 0) normal = 5;

                try collider.setCollisionVoxel(IVec3.init(x, y, z), CollisionVoxel.init(vtype, normal));
            }
        }
    }
}

# zig-voxel-physics

A high-performance voxel-to-voxel physics engine written in Zig. Designed for my custom voxel rendering engine.

## Features

- **Voxel-Based Colliders** - Sparse brick structure (8×8×8 voxel bricks) with per-voxel type classification
- **TGS Soft Solver** - Temporal Gauss-Seidel with soft constraints for stable stacking
- **Speculative Contacts** - Prevents contact loss and micro-bouncing at rest
- **Island-Based Sleeping** - Near-zero CPU cost when bodies are at rest (~0.003ms)
- **Warm Starting** - Contact impulse caching for improved solver convergence
- **Parallel Ready** - Thread pool support for narrow phase collision detection

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    VoxelPhysicsSystem                           │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │  BroadPhase │→ │ NarrowPhase │→ │   Solver    │             │
│  │  (AABB Tree)│  │(Voxel-Voxel)│  │  (TGS Soft) │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│         ↓                ↓                ↓                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │   Islands   │  │  Manifolds  │  │   Bodies    │             │
│  │   Manager   │  │   Cache     │  │   Array     │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
└─────────────────────────────────────────────────────────────────┘
```

### Core Components

| Component | Description |
|-----------|-------------|
| `VoxelPhysicsSystem` | Main coordinator, manages bodies and runs the simulation loop |
| `VoxelBody` | Rigid body with position, velocity, and mass properties |
| `VoxelCollider` | Sparse voxel structure defining collision shape |
| `BroadPhase` | Dynamic AABB tree for fast pair culling |
| `NarrowPhase` | Voxel-to-voxel contact generation |
| `ContactSolver` | TGS Soft constraint solver |
| `IslandManager` | Groups connected bodies for parallel solving |

---

## Installation

Add to your `build.zig.zon`:

```zig
.dependencies = .{
    .@"zig-voxel-physics" = .{
        .url = "https://github.com/yourusername/zig-voxel-physics/archive/refs/tags/v0.1.0.tar.gz",
        .hash = "...",
    },
},
```

Then in your `build.zig`:

```zig
const voxel_physics = b.dependency("zig-voxel-physics", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("zig-voxel-physics", voxel_physics.module("zig-voxel-physics"));
```

## Quick Start

```zig
const physics = @import("zig-voxel-physics");

// Create physics system
var system = try physics.VoxelPhysicsSystem.init(allocator, .{
    .gravity = physics.Vec3.init(0, -9.81, 0),
});
defer system.deinit();

// Create a collider
var collider = try allocator.create(physics.VoxelCollider);
collider.* = physics.VoxelCollider.init(allocator, 0.2); // 0.2m voxel size

// Add voxels (3x3x3 cube with proper classification)
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

            const vtype: physics.VoxelType = switch (edges) {
                3 => .corner,
                2 => .edge,
                1 => .face,
                else => .inside,
            };

            var normal: u5 = 2;
            if (on_x and x > 0) normal = 0;
            if (on_x and x < 0) normal = 1;
            if (on_y and y > 0) normal = 2;
            if (on_y and y < 0) normal = 3;
            if (on_z and z > 0) normal = 4;
            if (on_z and z < 0) normal = 5;

            const voxel = physics.CollisionVoxel.init(vtype, normal);
            try collider.setCollisionVoxel(physics.IVec3.init(x, y, z), voxel);
        }
    }
}
collider.recomputeMetadata();

// Create dynamic body
const handle = try system.createDynamicBody(
    physics.Vec3.init(0, 5, 0),
    physics.Quat.identity,
    collider,
    1.0,
);
system.setColliderOwnership(handle, true);

// Game loop
while (running) {
    system.update(dt);

    if (system.getBodyConst(handle)) |body| {
        const pos = body.getPosition();
        // Use position for rendering...
    }
}
```

## Voxel Types

Voxels must be classified correctly for collision detection to work:

| Type | Description | Collides With |
|------|-------------|---------------|
| `.corner` | Corner voxel (3 exposed faces) | Everything |
| `.edge` | Edge voxel (2 exposed faces) | Corners, edges |
| `.face` | Surface voxel (1 exposed face) | Corners |
| `.inside` | Interior voxel | Corners |
| `.empty` | No collision | Nothing |

**Important:** Face-to-face collisions are filtered out. Ensure shapes have proper corner/edge voxels for collision detection.

## Configuration

### PhysicsConfig

```zig
const config = physics.PhysicsConfig{
    // Core simulation
    .gravity = physics.Vec3.init(0, -9.81, 0), // Gravity (m/s²)
    .fixed_timestep = 1.0 / 60.0,              // Physics timestep (seconds)
    .max_substeps = 8,                          // Max substeps per frame

    // Solver
    .solver = physics.SolverConfig{ ... },      // See SolverConfig below

    // Capacity
    .max_bodies = 1024,                         // Maximum number of bodies

    // Material defaults
    .default_friction = 0.6,                    // Friction coefficient
    .default_restitution = 0.2,                 // Bounciness (0-1)
    .default_density = 1.0,                     // kg per voxel

    // Optimizations
    .enable_sleeping = true,                    // Sleep idle bodies
    .enable_warm_starting = true,               // Cache contact impulses
    .enable_threading = false,                  // Multithreaded narrow phase
    .thread_count = 0,                          // Worker threads (0 = auto)
    .enable_islands = true,                     // Island-based solving
    .enable_simd_broad_phase = true,            // SIMD optimizations
    .min_parallel_batch_size = 16,              // Min pairs for parallel processing
};
```

### SolverConfig

```zig
const solver = physics.SolverConfig{
    // Iterations
    .substeps = 4,                              // Sub-steps per physics step
    .velocity_iterations = 1,                   // Velocity iterations per substep
    .position_iterations = 2,                   // Position iterations (after substeps)

    // Soft constraints (Box2D 3.0 style)
    .hertz = 30.0,                              // Spring frequency (Hz)
    .damping_ratio = 1.0,                       // Damping (1.0 = critical)

    // Position correction
    .position_correction_rate = 0.2,            // Baumgarte stabilization rate
    .max_position_correction = 0.04,            // Max correction per iteration (m)
    .slop = 0.005,                              // Allowed penetration (m)

    // Warm starting
    .warm_starting = true,                      // Enable warm starting
    .warm_start_scale = 0.8,                    // Impulse scale factor (0-1)

    // Features
    .gravity = physics.Vec3.init(0, -9.81, 0),  // Gravity for integration
    .enable_friction = true,                    // Friction solving
    .enable_restitution = true,                 // Bounciness
    .disable_rotation = false,                  // Debug: disable angular velocity
};
```

## Performance

| Scenario | Typical Time |
|----------|--------------|
| All bodies sleeping | ~0.003ms |
| 10 active bodies | ~2-5ms |
| 50 active bodies | ~10-20ms |

## Building

```bash
# Build library
zig build

# Run tests
zig build test

# Run example
zig build example
```

## References

- **Box2D** by Erin Catto ([box2d.org](https://box2d.org/))
  - Used for the constraint solver, contact management, and warm starting
- **Bullet Physics** ([bulletphysics.org](https://bulletphysics.org/))
  - Broad phase culling, sleeping system, and threading approach
- **Catto, E.** "Soft Constraints" (GDC 2011)
  - Soft constraint math (the spring-damper model)
- **Catto, E.** "Solver2D" (GDC 2024)
  - TGS solver iteration structure
- **Coumans, E.** "Collision Detection" (GDC)
  - SAT algorithm for OBB collision testing
- **Lengyel, E.** "Voxel-Based Terrain for Real-Time Virtual Simulations"
  - Sparse voxel storage (changed from 16³ blocks to 8³ bricks for collision)

## License

MIT License - see [LICENSE](LICENSE) for details.

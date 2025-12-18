const std = @import("std");
const math = @import("math.zig");

const Vec3 = math.Vec3;
const AABB = math.AABB;

/// Null node index
pub const NULL_NODE: u32 = std.math.maxInt(u32);

/// AABB expansion margin for dynamic bodies
pub const AABB_EXTENSION: f32 = 0.1;

/// AABB expansion multiplier based on velocity
pub const AABB_VELOCITY_MULTIPLIER: f32 = 2.0;

/// A pair of potentially colliding bodies
pub const BodyPair = struct {
    body_a: u32,
    body_b: u32,

    /// Create a canonical pair
    pub fn init(a: u32, b: u32) BodyPair {
        if (a < b) {
            return .{ .body_a = a, .body_b = b };
        } else {
            return .{ .body_a = b, .body_b = a };
        }
    }

    /// Hash for use in hash sets
    pub fn hash(self: BodyPair) u64 {
        return @as(u64, self.body_a) | (@as(u64, self.body_b) << 32);
    }

    /// Equality check
    pub fn eql(self: BodyPair, other: BodyPair) bool {
        return self.body_a == other.body_a and self.body_b == other.body_b;
    }
};

/// Node in the dynamic AABB tree
pub const TreeNode = struct {
    /// Fat AABB
    aabb: AABB,

    /// User data
    user_data: u32,

    /// Parent node index
    parent: u32,

    /// Child node indices
    child1: u32,
    child2: u32,

    /// Height of subtree
    height: i32,

    /// Whether this node is a leaf
    pub fn isLeaf(self: TreeNode) bool {
        return self.child1 == NULL_NODE;
    }
};

/// Dynamic AABB tree for broad phase collision detection
pub const AABBTree = struct {
    allocator: std.mem.Allocator,

    /// All nodes in the tree
    nodes: std.ArrayListUnmanaged(TreeNode),

    /// Root node index
    root: u32,

    /// Free list for node recycling
    free_list: u32,

    /// Number of nodes in use
    node_count: u32,

    /// Map from body ID to node index
    body_to_node: std.AutoHashMapUnmanaged(u32, u32),

    /// Create a new AABB tree
    pub fn init(allocator: std.mem.Allocator) AABBTree {
        return .{
            .allocator = allocator,
            .nodes = .{},
            .root = NULL_NODE,
            .free_list = NULL_NODE,
            .node_count = 0,
            .body_to_node = .{},
        };
    }

    /// Clean up resources
    pub fn deinit(self: *AABBTree) void {
        self.nodes.deinit(self.allocator);
        self.body_to_node.deinit(self.allocator);
    }

    /// Clear all nodes from the tree
    pub fn clear(self: *AABBTree) void {
        self.nodes.clearRetainingCapacity();
        self.root = NULL_NODE;
        self.free_list = NULL_NODE;
        self.node_count = 0;
        self.body_to_node.clearRetainingCapacity();
    }

    /// Allocate a node from the pool
    fn allocateNode(self: *AABBTree) !u32 {
        if (self.free_list != NULL_NODE) {
            const node_id = self.free_list;
            self.free_list = self.nodes.items[node_id].parent;
            self.nodes.items[node_id] = .{
                .aabb = AABB.empty,
                .user_data = 0,
                .parent = NULL_NODE,
                .child1 = NULL_NODE,
                .child2 = NULL_NODE,
                .height = 0,
            };
            self.node_count += 1;
            return node_id;
        }

        const node_id: u32 = @intCast(self.nodes.items.len);
        try self.nodes.append(self.allocator, .{
            .aabb = AABB.empty,
            .user_data = 0,
            .parent = NULL_NODE,
            .child1 = NULL_NODE,
            .child2 = NULL_NODE,
            .height = 0,
        });
        self.node_count += 1;
        return node_id;
    }

    /// Free a node back to the pool
    fn freeNode(self: *AABBTree, node_id: u32) void {
        self.nodes.items[node_id].parent = self.free_list;
        self.nodes.items[node_id].height = -1;
        self.free_list = node_id;
        self.node_count -= 1;
    }

    /// Insert a body into the tree
    pub fn insertBody(self: *AABBTree, body_id: u32, aabb: AABB) !u32 {
        const node_id = try self.allocateNode();

        const fat_aabb = aabb.inflate(AABB_EXTENSION);

        self.nodes.items[node_id].aabb = fat_aabb;
        self.nodes.items[node_id].user_data = body_id;
        self.nodes.items[node_id].height = 0;

        try self.body_to_node.put(self.allocator, body_id, node_id);
        self.insertLeaf(node_id);

        return node_id;
    }

    /// Remove a body from the tree
    pub fn removeBody(self: *AABBTree, body_id: u32) void {
        if (self.body_to_node.get(body_id)) |node_id| {
            self.removeLeaf(node_id);
            self.freeNode(node_id);
            _ = self.body_to_node.remove(body_id);
        }
    }

    /// Update a body's AABB in the tree
    pub fn updateBody(self: *AABBTree, body_id: u32, aabb: AABB, displacement: Vec3) !bool {
        const node_id = self.body_to_node.get(body_id) orelse return false;

        const fat_aabb = aabb.inflate(AABB_EXTENSION);

        if (self.nodes.items[node_id].aabb.containsAABB(aabb)) {
            const large_aabb = fat_aabb.inflate(AABB_EXTENSION * 4.0);
            if (large_aabb.containsAABB(self.nodes.items[node_id].aabb)) {
                return false;
            }
        }

        self.removeLeaf(node_id);

        var extended = fat_aabb;
        const d = displacement.scale(AABB_VELOCITY_MULTIPLIER);

        if (d.x < 0) {
            extended.min.x += d.x;
        } else {
            extended.max.x += d.x;
        }
        if (d.y < 0) {
            extended.min.y += d.y;
        } else {
            extended.max.y += d.y;
        }
        if (d.z < 0) {
            extended.min.z += d.z;
        } else {
            extended.max.z += d.z;
        }

        self.nodes.items[node_id].aabb = extended;
        self.insertLeaf(node_id);

        return true;
    }

    /// Insert a leaf node into the tree
    fn insertLeaf(self: *AABBTree, leaf: u32) void {
        if (self.root == NULL_NODE) {
            self.root = leaf;
            self.nodes.items[leaf].parent = NULL_NODE;
            return;
        }

        const leaf_aabb = self.nodes.items[leaf].aabb;
        var index = self.root;

        while (!self.nodes.items[index].isLeaf()) {
            const child1 = self.nodes.items[index].child1;
            const child2 = self.nodes.items[index].child2;

            const area = self.nodes.items[index].aabb.surfaceArea();

            const combined_aabb = self.nodes.items[index].aabb.merge(leaf_aabb);
            const combined_area = combined_aabb.surfaceArea();

            const cost = 2.0 * combined_area;

            const inheritance_cost = 2.0 * (combined_area - area);

            var cost1: f32 = undefined;
            if (self.nodes.items[child1].isLeaf()) {
                const aabb = leaf_aabb.merge(self.nodes.items[child1].aabb);
                cost1 = aabb.surfaceArea() + inheritance_cost;
            } else {
                const aabb = leaf_aabb.merge(self.nodes.items[child1].aabb);
                const old_area = self.nodes.items[child1].aabb.surfaceArea();
                const new_area = aabb.surfaceArea();
                cost1 = (new_area - old_area) + inheritance_cost;
            }

            var cost2: f32 = undefined;
            if (self.nodes.items[child2].isLeaf()) {
                const aabb = leaf_aabb.merge(self.nodes.items[child2].aabb);
                cost2 = aabb.surfaceArea() + inheritance_cost;
            } else {
                const aabb = leaf_aabb.merge(self.nodes.items[child2].aabb);
                const old_area = self.nodes.items[child2].aabb.surfaceArea();
                const new_area = aabb.surfaceArea();
                cost2 = (new_area - old_area) + inheritance_cost;
            }

            if (cost < cost1 and cost < cost2) {
                break;
            }

            if (cost1 < cost2) {
                index = child1;
            } else {
                index = child2;
            }
        }

        const sibling = index;

        const old_parent = self.nodes.items[sibling].parent;
        const new_parent = self.allocateNode() catch return;
        self.nodes.items[new_parent].parent = old_parent;
        self.nodes.items[new_parent].user_data = 0;
        self.nodes.items[new_parent].aabb = leaf_aabb.merge(self.nodes.items[sibling].aabb);
        self.nodes.items[new_parent].height = self.nodes.items[sibling].height + 1;

        if (old_parent != NULL_NODE) {
            if (self.nodes.items[old_parent].child1 == sibling) {
                self.nodes.items[old_parent].child1 = new_parent;
            } else {
                self.nodes.items[old_parent].child2 = new_parent;
            }

            self.nodes.items[new_parent].child1 = sibling;
            self.nodes.items[new_parent].child2 = leaf;
            self.nodes.items[sibling].parent = new_parent;
            self.nodes.items[leaf].parent = new_parent;
        } else {
            self.nodes.items[new_parent].child1 = sibling;
            self.nodes.items[new_parent].child2 = leaf;
            self.nodes.items[sibling].parent = new_parent;
            self.nodes.items[leaf].parent = new_parent;
            self.root = new_parent;
        }

        var walk_index = self.nodes.items[leaf].parent;
        while (walk_index != NULL_NODE) {
            walk_index = self.balance(walk_index);

            const c1 = self.nodes.items[walk_index].child1;
            const c2 = self.nodes.items[walk_index].child2;

            self.nodes.items[walk_index].height = 1 + @max(
                self.nodes.items[c1].height,
                self.nodes.items[c2].height,
            );
            self.nodes.items[walk_index].aabb = self.nodes.items[c1].aabb.merge(self.nodes.items[c2].aabb);

            walk_index = self.nodes.items[walk_index].parent;
        }
    }

    /// Remove a leaf node from the tree
    fn removeLeaf(self: *AABBTree, leaf: u32) void {
        if (leaf == self.root) {
            self.root = NULL_NODE;
            return;
        }

        const parent = self.nodes.items[leaf].parent;
        const grand_parent = self.nodes.items[parent].parent;
        var sibling: u32 = undefined;

        if (self.nodes.items[parent].child1 == leaf) {
            sibling = self.nodes.items[parent].child2;
        } else {
            sibling = self.nodes.items[parent].child1;
        }

        if (grand_parent != NULL_NODE) {
            if (self.nodes.items[grand_parent].child1 == parent) {
                self.nodes.items[grand_parent].child1 = sibling;
            } else {
                self.nodes.items[grand_parent].child2 = sibling;
            }
            self.nodes.items[sibling].parent = grand_parent;
            self.freeNode(parent);

            var index = grand_parent;
            while (index != NULL_NODE) {
                index = self.balance(index);

                const child1 = self.nodes.items[index].child1;
                const child2 = self.nodes.items[index].child2;

                self.nodes.items[index].aabb = self.nodes.items[child1].aabb.merge(self.nodes.items[child2].aabb);
                self.nodes.items[index].height = 1 + @max(
                    self.nodes.items[child1].height,
                    self.nodes.items[child2].height,
                );

                index = self.nodes.items[index].parent;
            }
        } else {
            self.root = sibling;
            self.nodes.items[sibling].parent = NULL_NODE;
            self.freeNode(parent);
        }
    }

    /// Balance the tree at a node
    fn balance(self: *AABBTree, index: u32) u32 {
        if (self.nodes.items[index].isLeaf() or self.nodes.items[index].height < 2) {
            return index;
        }

        const child1 = self.nodes.items[index].child1;
        const child2 = self.nodes.items[index].child2;

        const balance_val = self.nodes.items[child2].height - self.nodes.items[child1].height;

        if (balance_val > 1) {
            const child2_child1 = self.nodes.items[child2].child1;
            const child2_child2 = self.nodes.items[child2].child2;

            self.nodes.items[child2].child1 = index;
            self.nodes.items[child2].parent = self.nodes.items[index].parent;
            self.nodes.items[index].parent = child2;

            if (self.nodes.items[child2].parent != NULL_NODE) {
                if (self.nodes.items[self.nodes.items[child2].parent].child1 == index) {
                    self.nodes.items[self.nodes.items[child2].parent].child1 = child2;
                } else {
                    self.nodes.items[self.nodes.items[child2].parent].child2 = child2;
                }
            } else {
                self.root = child2;
            }

            if (self.nodes.items[child2_child1].height > self.nodes.items[child2_child2].height) {
                self.nodes.items[child2].child2 = child2_child1;
                self.nodes.items[index].child2 = child2_child2;
                self.nodes.items[child2_child2].parent = index;
                self.nodes.items[index].aabb = self.nodes.items[child1].aabb.merge(self.nodes.items[child2_child2].aabb);
                self.nodes.items[child2].aabb = self.nodes.items[index].aabb.merge(self.nodes.items[child2_child1].aabb);

                self.nodes.items[index].height = 1 + @max(self.nodes.items[child1].height, self.nodes.items[child2_child2].height);
                self.nodes.items[child2].height = 1 + @max(self.nodes.items[index].height, self.nodes.items[child2_child1].height);
            } else {
                self.nodes.items[child2].child2 = child2_child2;
                self.nodes.items[index].child2 = child2_child1;
                self.nodes.items[child2_child1].parent = index;
                self.nodes.items[index].aabb = self.nodes.items[child1].aabb.merge(self.nodes.items[child2_child1].aabb);
                self.nodes.items[child2].aabb = self.nodes.items[index].aabb.merge(self.nodes.items[child2_child2].aabb);

                self.nodes.items[index].height = 1 + @max(self.nodes.items[child1].height, self.nodes.items[child2_child1].height);
                self.nodes.items[child2].height = 1 + @max(self.nodes.items[index].height, self.nodes.items[child2_child2].height);
            }

            return child2;
        }

        if (balance_val < -1) {
            const child1_child1 = self.nodes.items[child1].child1;
            const child1_child2 = self.nodes.items[child1].child2;

            self.nodes.items[child1].child1 = index;
            self.nodes.items[child1].parent = self.nodes.items[index].parent;
            self.nodes.items[index].parent = child1;

            if (self.nodes.items[child1].parent != NULL_NODE) {
                if (self.nodes.items[self.nodes.items[child1].parent].child1 == index) {
                    self.nodes.items[self.nodes.items[child1].parent].child1 = child1;
                } else {
                    self.nodes.items[self.nodes.items[child1].parent].child2 = child1;
                }
            } else {
                self.root = child1;
            }

            if (self.nodes.items[child1_child1].height > self.nodes.items[child1_child2].height) {
                self.nodes.items[child1].child2 = child1_child1;
                self.nodes.items[index].child1 = child1_child2;
                self.nodes.items[child1_child2].parent = index;
                self.nodes.items[index].aabb = self.nodes.items[child2].aabb.merge(self.nodes.items[child1_child2].aabb);
                self.nodes.items[child1].aabb = self.nodes.items[index].aabb.merge(self.nodes.items[child1_child1].aabb);

                self.nodes.items[index].height = 1 + @max(self.nodes.items[child2].height, self.nodes.items[child1_child2].height);
                self.nodes.items[child1].height = 1 + @max(self.nodes.items[index].height, self.nodes.items[child1_child1].height);
            } else {
                self.nodes.items[child1].child2 = child1_child2;
                self.nodes.items[index].child1 = child1_child1;
                self.nodes.items[child1_child1].parent = index;
                self.nodes.items[index].aabb = self.nodes.items[child2].aabb.merge(self.nodes.items[child1_child1].aabb);
                self.nodes.items[child1].aabb = self.nodes.items[index].aabb.merge(self.nodes.items[child1_child2].aabb);

                self.nodes.items[index].height = 1 + @max(self.nodes.items[child2].height, self.nodes.items[child1_child1].height);
                self.nodes.items[child1].height = 1 + @max(self.nodes.items[index].height, self.nodes.items[child1_child2].height);
            }

            return child1;
        }

        return index;
    }

    /// Query the tree for all bodies overlapping an AABB
    pub fn query(self: *const AABBTree, aabb: AABB, results: *std.ArrayListUnmanaged(u32)) !void {
        var stack = std.ArrayListUnmanaged(u32){};
        defer stack.deinit(self.allocator);

        if (self.root == NULL_NODE) return;

        try stack.append(self.allocator, self.root);

        while (stack.items.len > 0) {
            const node_id = stack.pop() orelse continue;

            if (node_id == NULL_NODE) continue;

            const node = self.nodes.items[@intCast(node_id)];

            if (node.aabb.overlaps(aabb)) {
                if (node.isLeaf()) {
                    try results.append(self.allocator, node.user_data);
                } else {
                    try stack.append(self.allocator, node.child1);
                    try stack.append(self.allocator, node.child2);
                }
            }
        }
    }

    /// Get all pairs of potentially overlapping bodies within the tree
    pub fn findOverlappingPairs(self: *const AABBTree, pairs: *std.ArrayListUnmanaged(BodyPair)) !void {
        if (self.root == NULL_NODE) return;

        var results = std.ArrayListUnmanaged(u32){};
        defer results.deinit(self.allocator);

        for (self.nodes.items, 0..) |node, i| {
            if (node.height >= 0 and node.isLeaf()) {
                results.clearRetainingCapacity();
                try self.query(node.aabb, &results);

                for (results.items) |other_id| {
                    if (other_id > node.user_data) {
                        try pairs.append(self.allocator, BodyPair.init(node.user_data, other_id));
                    }
                }
                _ = i;
            }
        }
    }

    /// Query against another tree
    pub fn queryAgainstTree(self: *const AABBTree, other: *const AABBTree, pairs: *std.ArrayListUnmanaged(BodyPair)) !void {
        if (self.root == NULL_NODE or other.root == NULL_NODE) return;

        var results = std.ArrayListUnmanaged(u32){};
        defer results.deinit(self.allocator);

        for (self.nodes.items) |node| {
            if (node.height >= 0 and node.isLeaf()) {
                results.clearRetainingCapacity();
                try other.query(node.aabb, &results);

                for (results.items) |other_id| {
                    try pairs.append(self.allocator, BodyPair.init(node.user_data, other_id));
                }
            }
        }
    }

    /// Get the node for a body ID
    pub fn getNode(self: *const AABBTree, body_id: u32) ?*const TreeNode {
        if (self.body_to_node.get(body_id)) |node_id| {
            return &self.nodes.items[node_id];
        }
        return null;
    }
};

/// Complete broad phase collision detection system
pub const BroadPhase = struct {
    allocator: std.mem.Allocator,

    /// Tree for static bodies
    static_tree: AABBTree,

    /// Tree for dynamic bodies
    dynamic_tree: AABBTree,

    /// Pair cache for temporal coherence
    cached_pairs: std.ArrayListUnmanaged(BodyPair),

    /// Whether the cache needs rebuilding
    cache_dirty: bool,

    /// Create a new broad phase
    pub fn init(allocator: std.mem.Allocator) BroadPhase {
        return .{
            .allocator = allocator,
            .static_tree = AABBTree.init(allocator),
            .dynamic_tree = AABBTree.init(allocator),
            .cached_pairs = .{},
            .cache_dirty = true,
        };
    }

    /// Clean up resources
    pub fn deinit(self: *BroadPhase) void {
        self.static_tree.deinit();
        self.dynamic_tree.deinit();
        self.cached_pairs.deinit(self.allocator);
    }

    /// Clear all bodies from the broad phase
    pub fn clear(self: *BroadPhase) void {
        self.static_tree.clear();
        self.dynamic_tree.clear();
        self.cached_pairs.clearRetainingCapacity();
        self.cache_dirty = true;
    }

    /// Add a static body
    pub fn addStaticBody(self: *BroadPhase, body_id: u32, aabb: AABB) !void {
        _ = try self.static_tree.insertBody(body_id, aabb);
        self.cache_dirty = true;
    }

    /// Add a dynamic body
    pub fn addDynamicBody(self: *BroadPhase, body_id: u32, aabb: AABB) !void {
        _ = try self.dynamic_tree.insertBody(body_id, aabb);
        self.cache_dirty = true;
    }

    /// Remove a body
    pub fn removeBody(self: *BroadPhase, body_id: u32) void {
        self.static_tree.removeBody(body_id);
        self.dynamic_tree.removeBody(body_id);
        self.cache_dirty = true;
    }

    /// Update a dynamic body's AABB
    pub fn updateBody(self: *BroadPhase, body_id: u32, aabb: AABB, displacement: Vec3) !void {
        if (try self.dynamic_tree.updateBody(body_id, aabb, displacement)) {
            self.cache_dirty = true;
        }
    }

    /// Find all potentially overlapping body pairs
    pub fn findOverlappingPairs(self: *BroadPhase) ![]BodyPair {
        if (!self.cache_dirty) {
            return self.cached_pairs.items;
        }

        self.cached_pairs.clearRetainingCapacity();

        try self.dynamic_tree.findOverlappingPairs(&self.cached_pairs);

        try self.dynamic_tree.queryAgainstTree(&self.static_tree, &self.cached_pairs);

        self.cache_dirty = false;
        return self.cached_pairs.items;
    }

    /// Query for bodies overlapping an AABB
    pub fn queryAABB(self: *const BroadPhase, aabb: AABB, results: *std.ArrayListUnmanaged(u32)) !void {
        try self.static_tree.query(aabb, results);
        try self.dynamic_tree.query(aabb, results);
    }

    /// Mark the cache as dirty
    pub fn invalidateCache(self: *BroadPhase) void {
        self.cache_dirty = true;
    }
};

test "BodyPair canonical ordering" {
    const pair1 = BodyPair.init(5, 3);
    try std.testing.expectEqual(@as(u32, 3), pair1.body_a);
    try std.testing.expectEqual(@as(u32, 5), pair1.body_b);

    const pair2 = BodyPair.init(3, 5);
    try std.testing.expect(pair1.eql(pair2));
}

test "AABBTree insert and query" {
    var tree = AABBTree.init(std.testing.allocator);
    defer tree.deinit();

    const aabb1 = AABB.init(Vec3.init(0, 0, 0), Vec3.init(1, 1, 1));
    const aabb2 = AABB.init(Vec3.init(0.5, 0.5, 0.5), Vec3.init(1.5, 1.5, 1.5));
    const aabb3 = AABB.init(Vec3.init(10, 10, 10), Vec3.init(11, 11, 11));

    _ = try tree.insertBody(1, aabb1);
    _ = try tree.insertBody(2, aabb2);
    _ = try tree.insertBody(3, aabb3);

    var results = std.ArrayListUnmanaged(u32){};
    defer results.deinit(std.testing.allocator);

    const query_aabb = AABB.init(Vec3.init(0.25, 0.25, 0.25), Vec3.init(0.75, 0.75, 0.75));
    try tree.query(query_aabb, &results);

    try std.testing.expect(results.items.len >= 1);
}

test "AABBTree remove body" {
    var tree = AABBTree.init(std.testing.allocator);
    defer tree.deinit();

    const aabb = AABB.init(Vec3.init(0, 0, 0), Vec3.init(1, 1, 1));
    _ = try tree.insertBody(1, aabb);

    try std.testing.expect(tree.getNode(1) != null);

    tree.removeBody(1);

    try std.testing.expect(tree.getNode(1) == null);
}

test "BroadPhase find overlapping pairs" {
    var bp = BroadPhase.init(std.testing.allocator);
    defer bp.deinit();

    const aabb1 = AABB.init(Vec3.init(0, 0, 0), Vec3.init(1, 1, 1));
    const aabb2 = AABB.init(Vec3.init(0.5, 0.5, 0.5), Vec3.init(1.5, 1.5, 1.5));
    const aabb3 = AABB.init(Vec3.init(10, 10, 10), Vec3.init(11, 11, 11));

    try bp.addDynamicBody(1, aabb1);
    try bp.addDynamicBody(2, aabb2);
    try bp.addDynamicBody(3, aabb3);

    const pairs = try bp.findOverlappingPairs();

    try std.testing.expect(pairs.len >= 1);

    var found_pair = false;
    for (pairs) |pair| {
        if ((pair.body_a == 1 and pair.body_b == 2) or (pair.body_a == 2 and pair.body_b == 1)) {
            found_pair = true;
            break;
        }
    }
    try std.testing.expect(found_pair);
}

test "BroadPhase static vs dynamic" {
    var bp = BroadPhase.init(std.testing.allocator);
    defer bp.deinit();

    const aabb1 = AABB.init(Vec3.init(0, 0, 0), Vec3.init(1, 1, 1));
    const aabb2 = AABB.init(Vec3.init(0.5, 0.5, 0.5), Vec3.init(1.5, 1.5, 1.5));

    try bp.addStaticBody(1, aabb1);
    try bp.addDynamicBody(2, aabb2);

    const pairs = try bp.findOverlappingPairs();

    try std.testing.expect(pairs.len >= 1);
}

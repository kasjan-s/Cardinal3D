
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
#include <iostream>

namespace PT {

constexpr int kBvhBinCount = 8;
constexpr float kMaxFloat = std::numeric_limits<float>::max();

// construct BVH hierarchy given a vector of prims
template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.

    // Keep these two lines of code in your solution. They clear the list of nodes and
    // initialize member variable 'primitives' as a vector of the scene prims
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Modify the code ahead to construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.
    //
    // Please use the SAH as described in class.  We recomment the binned build from lecture.
    // In general, here is a rough sketch:
    //
    //  For each axis X,Y,Z:
    //     Try possible splits along axis, evaluate SAH for each
    //  Take minimum cost across all axes.
    //  Partition primitives into a left and right child group
    //  Compute left and right child bboxes
    //  Make the left and right child nodes.
    //
    //
    // While a BVH is conceptually a tree structure, the BVH class uses a single vector (nodes)
    // to store all the nodes. Therefore, BVH nodes don't contain pointers to child nodes,
    // but rather the indices of the
    // child nodes in this array. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.
    //
    // As an example of how to make nodes, the starter code below builds a BVH with a
    // root node that encloses all the primitives and its two descendants at Level 2.
    // For now, the split is hardcoded such that the first primitive is put in the left
    // child of the root, and all the other primitives are in the right child.
    // There are no further descendants.

    // edge case
    if(primitives.empty()) {
        return;
    }

    // compute bounding box for all primitives
    BBox bb;
    for(size_t i = 0; i < primitives.size(); ++i) {
        bb.enclose(primitives[i].bbox());
    }

    // set up root node (root BVH). Notice that it contains all primitives.
    size_t root_node_addr = new_node();
    Node& node = nodes[root_node_addr];
    node.bbox = bb;
    node.start = 0;
    node.size = primitives.size();
    root_idx = root_node_addr;

    std::function<void(size_t, std::vector<Node>&, std::vector<Primitive>&)> splitNode;

    splitNode = [this, &splitNode](size_t node_index, std::vector<Node>& nodes, std::vector<Primitive>& primitives) {
        Node& node = nodes[node_index];
        if (node.size == 1) {
            return;
        }

        if (node.size == 2) {
            // Just split into two nodes directly to save work and to not have to deal with edge cases where
            // one of the nodes might be exactly on the bin boundary (or where we run into floating point errors).
            BBox split_leftBox;
            split_leftBox.enclose(primitives[node.start].bbox());
            BBox split_rightBox;
            split_rightBox.enclose(primitives[node.start + 1].bbox());

            // Same boundary, just return, it makes no difference if we split.
            if (split_leftBox.min == split_rightBox.min && split_leftBox.max == split_rightBox.max) {
                return;
            }

            size_t startl = node.start;
            size_t node_addr_l = new_node();
            size_t node_addr_r = new_node();
            nodes[node_index].l = node_addr_l;
            nodes[node_index].r = node_addr_r;

            nodes[node_addr_l].bbox = split_leftBox;
            nodes[node_addr_l].start = startl;
            nodes[node_addr_l].size = 1;

            nodes[node_addr_r].bbox = split_rightBox;
            nodes[node_addr_r].start = startl + 1;
            nodes[node_addr_r].size = 1;

            return;
        }

        //  For each axis X,Y,Z:
        //     Try possible splits along axis, evaluate SAH for each
        //  Take minimum cost across all axes.
        //  Partition primitives into a left and right child group
        //  Compute left and right child bboxes
        //  Make the left and right child nodes.

        enum Axis {
            X, Y, Z
        };

        struct SplitCandidate {
            float cost;
            size_t left_bin_count;
            size_t right_bin_count;
            float coord_min;
            float coord_max;
            float boundary;
            Axis axis;
            bool operator<(const SplitCandidate& other) const {
                return cost < other.cost;
            }
        };

        auto calculateBinCosts = [](const std::vector<Primitive>& primitives, size_t begin, size_t end, float coord_min, float coord_max, Axis axis) {
            float bin_size = (coord_max - coord_min) / kBvhBinCount;
            std::vector<std::pair<BBox, int>> split(kBvhBinCount, std::make_pair(BBox(), 0));
            for (size_t idx = begin; idx < end; ++idx) {
                auto& obj = primitives[idx];
                float coord;
                switch (axis) {
                    case Axis::X:
                        coord = obj.bbox().center().x;
                        break;
                    case Axis::Y:
                        coord = obj.bbox().center().y;
                        break;
                    case Axis::Z:
                        coord = obj.bbox().center().z;
                        break;
                }
                int bin = std::floor((coord - coord_min) / bin_size);
                bin = std::clamp(bin, 0, kBvhBinCount - 1);
                split[bin].first.enclose(obj.bbox());
                split[bin].second++;
            }

            std::vector<SplitCandidate> costs;
            for (int i = 0; i < kBvhBinCount - 1; ++i) {
                int count_left = 0;
                int count_right = 0;
                BBox left;
                BBox right;
                for (int j = 0; j < kBvhBinCount; ++j) {
                    if (j < i + 1) {
                        left.enclose(split[j].first);
                        count_left += split[j].second;
                    } else {
                        right.enclose(split[j].first);
                        count_right += split[j].second;
                    }
                }

                SplitCandidate bin_cost;
                bin_cost.cost = left.surface_area() * count_left + right.surface_area() * count_right;
                bin_cost.left_bin_count = count_left;
                bin_cost.right_bin_count = count_right;
                bin_cost.axis = axis;
                bin_cost.coord_min = coord_min;
                bin_cost.coord_max = coord_max;
                bin_cost.boundary = coord_min + (i + 1) * bin_size;
                costs.push_back(bin_cost);
            }
            return costs;
        };
        BBox& bb = node.bbox;
        std::vector<SplitCandidate> x_bin_costs = calculateBinCosts(primitives, node.start, node.start + node.size, bb.min.x, bb.max.x, Axis::X);
        std::vector<SplitCandidate> y_bin_costs = calculateBinCosts(primitives, node.start, node.start + node.size, bb.min.y, bb.max.y, Axis::Y);
        std::vector<SplitCandidate> z_bin_costs = calculateBinCosts(primitives, node.start, node.start + node.size, bb.min.z, bb.max.z, Axis::Z);

        auto x_min_cost_iter = std::min_element(x_bin_costs.begin(), x_bin_costs.end());
        auto y_min_cost_iter = std::min_element(y_bin_costs.begin(), y_bin_costs.end());
        auto z_min_cost_iter = std::min_element(z_bin_costs.begin(), z_bin_costs.end());

        std::vector<SplitCandidate> candidates = {*x_min_cost_iter, *y_min_cost_iter, *z_min_cost_iter};
        for (auto iter = candidates.begin(); iter != candidates.end();) {
            if (iter->left_bin_count == 0 || iter->right_bin_count == 0) {
                iter = candidates.erase(iter);
            }  else {
                ++iter;
            }
        }
        
        if (candidates.empty())
            return;

        auto best_candidate = candidates[0];
        for (auto candidate : candidates) {
            if (candidate.cost < best_candidate.cost)
                best_candidate = candidate;
        }

        std::function<float(const Primitive&)> axis_coord;
        switch (best_candidate.axis) {
            case Axis::X: {
                axis_coord = [](const Primitive& obj) { return obj.bbox().center().x; };
                break;
            } 
            case Axis::Y: {
                axis_coord = [](const Primitive& obj) { return obj.bbox().center().y; };
                break;
            } 
            case Axis::Z: {
                axis_coord = [](const Primitive& obj) { return obj.bbox().center().z; };
                break;
            } 
        }

        // Create bounding boxes for children
        BBox split_leftBox;
        BBox split_rightBox;

        int left = node.start;
        int right = node.start + node.size - 1;
        size_t left_count = 0;
        while (left <= right) {
            if (axis_coord(primitives[left]) <= best_candidate.boundary) {
                std::swap(primitives[left], primitives[right]);
                split_rightBox.enclose(primitives[right].bbox());
                --right;
            } else {
                split_leftBox.enclose(primitives[left].bbox());
                ++left;
                ++left_count;
            }
        }
        
        // Edge case where end-objects were directly on the boundary.
        if (left_count == 0 || left_count == node.size) {
            return;
        }

        size_t startl = node.start;  // starting prim index of left child
        size_t rangel = left_count; // number of prims in left child
        size_t startr = node.start + rangel;  // starting prim index of right child
        size_t ranger = node.size - rangel; // number of prims in right child

        // create child nodes
        size_t node_addr_l = new_node();
        size_t node_addr_r = new_node();
        nodes[node_index].l = node_addr_l;
        nodes[node_index].r = node_addr_r;

        nodes[node_addr_l].bbox = split_leftBox;
        nodes[node_addr_l].start = startl;
        nodes[node_addr_l].size = rangel;

        nodes[node_addr_r].bbox = split_rightBox;
        nodes[node_addr_r].start = startr;
        nodes[node_addr_r].size = ranger;

        splitNode(node_addr_l, nodes, primitives);
        splitNode(node_addr_r, nodes, primitives);
    };
    splitNode(root_idx, nodes, primitives);
}

template<typename Primitive>
Trace BVH<Primitive>::hit(const Ray& ray) const {
    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    ret.distance = kMaxFloat;

    std::function<void(const Node&, const Ray&, Trace&)> visit_node;
    visit_node = [this, &visit_node] (const Node& node, const Ray& ray, Trace& ret) {
        Vec2 times(0.0f, ret.distance);
        if (!node.bbox.hit(ray, times)) {
            return;
        }

        if (node.is_leaf()) {
            for (size_t idx = node.start; idx < node.start + node.size; ++idx) {
                Trace hit = primitives[idx].hit(ray);
                if (hit.hit) {
                    ret = Trace::min(ret, hit);
                }
            }
            return;
        } else {
            Vec2 left_times(0.0f, times[1]);
            Vec2 right_times(0.0f, times[1]);
            bool hit_left = nodes[node.l].bbox.hit(ray, left_times);
            bool hit_right = nodes[node.r].bbox.hit(ray, right_times);

            if (!hit_left && !hit_right) {
                return;
            }

            if (hit_left && !hit_right) {
                return visit_node(nodes[node.l], ray, ret);
            }

            if (!hit_left && hit_right) {
                return visit_node(nodes[node.r], ray, ret);
            }

            if (left_times[0] < right_times[0]) {
                visit_node(nodes[node.l], ray, ret);
                if (right_times[0] < ret.distance)
                    visit_node(nodes[node.r], ray, ret);
            } else {
                visit_node(nodes[node.r], ray, ret);
                if (right_times[0] < ret.distance)
                    visit_node(nodes[node.l], ray, ret);
            }
        }
    };

    if (root_idx < nodes.size()) {
        visit_node(nodes[root_idx], ray, ret);
    }

    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive>
BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive>
bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive>
BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive>
std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive>
void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT

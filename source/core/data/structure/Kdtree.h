#ifndef KDTREE_H
#define KDTREE_H

#include "core/data/primitive/Point.h"
#include <algorithm>
#include <queue>
#include <vector>


namespace n2m {
class Kdtree {
private:
    struct node {
        node(const Point &pt) : point_(pt), left_(nullptr),
                                right_(nullptr) {
        }

        double get(size_t index) const {
            return point_.get(index);
        }

        double distance(const Point &pt) const {
            return point_.compute_distance(pt);
        }

        Point point_;
        node *left_;
        node *right_;
    };

    node *root_ = nullptr;
    size_t visited_ = 0;
    std::vector<node> nodes_;

    struct node_cmp {
        node_cmp(size_t index) : index_(index) {
        }

        bool operator()(const node &n1, const node &n2) const {
            return n1.point_.get(index_) < n2.point_.get(index_);
        }

        size_t index_;
    };

    node *make_tree(size_t begin, size_t end, size_t index) {
        if (end <= begin) return nullptr;
        size_t n = begin + (end - begin) / 2;
        auto i = nodes_.begin();
        std::nth_element(i + begin, i + n, i + end, node_cmp(index));
        index = (index + 1) % 3;
        nodes_[n].left_ = make_tree(begin, n, index);
        nodes_[n].right_ = make_tree(n + 1, end, index);
        return &nodes_[n];
    }

    void k_nearest(node *root,
                   const Point &point,
                   size_t index,
                   size_t k,
                   std::priority_queue<std::pair<double, const Point *> > &pq) {
        if (root == nullptr) return;

        ++visited_;
        double d = root->distance(point);

        if (d != 0 && pq.size() < k) {
            pq.emplace(d, &root->point_);
        } else if (d != 0 && d < pq.top().first) {
            pq.pop();
            pq.emplace(d, &root->point_);
        }

        double dx = root->get(index) - point.get(index);
        index = (index + 1) % 3;

        // Visit the subtree containing the query point first
        k_nearest(dx > 0 ? root->left_ : root->right_, point, index, k, pq);

        // Check the other subtree if there's a possibility of finding closer points
        if (dx * dx < pq.top().first || pq.size() < k) {
            k_nearest(dx > 0 ? root->right_ : root->left_, point, index, k,
                      pq);
        }
    }

public:
    Kdtree(const Kdtree &) = delete;

    Kdtree &operator=(const Kdtree &) = delete;


    template<typename iterator>
    Kdtree(iterator begin, iterator end) : nodes_(begin, end) {
        root_ = make_tree(0, nodes_.size(), 0);
    }


    template<typename func>
    Kdtree(func &&f, size_t n) {
        nodes_.reserve(n);
        for (size_t i = 0; i < n; ++i) nodes_.push_back(f());
        root_ = make_tree(0, nodes_.size(), 0);
    }

    /**
     * Returns true if the tree is empty, false otherwise.
     */
    bool empty() const { return nodes_.empty(); }

    /**
     * Returns the number of nodes visited by the last call
     * to nearest().
     */
    size_t visited() const { return visited_; }


    /**
         * Finds the K nearest points in the tree to the given point.
         * It is not valid to call this function if the tree is empty.
         *
         * @param pt a point
         * @param k number of nearest neighbors to find
         * @return a list of the K nearest points
         */
    std::vector<Point> k_nearest(const Point &pt, size_t k) {
        if (root_ == nullptr) throw;


        visited_ = 0;
        std::priority_queue<std::pair<double, const Point *> > pq;
        k_nearest(root_, pt, 0, k, pq);

        std::vector<Point> result;
        while (!pq.empty()) {
            result.push_back(*pq.top().second);
            pq.pop();
        }

        std::reverse(result.begin(), result.end()); // Closest points first
        return result;
    }
};
}
#endif //KDTREE_H

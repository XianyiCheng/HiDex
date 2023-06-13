#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#ifndef UTILS_H
#define UTILS_H
#include "../mechanics/utilities/utilities.h"
#endif

#ifndef SAMPLE_H
#define SAMPLE_H
#include "../mechanics/utilities/sample.h"
#endif


#ifndef CONTACTCONSTRAINTS_H
#define CONTACTCONSTRAINTS_H
    #include "../mechanics/contacts/contact_constraints.h"
#endif



// Resuable RRT
class ReusableRRT {
public:
  struct Node {
    Vector7d config; // x, y, z, qx, qy, qz, qw
    VectorXd manipulator_config; // optional: save and search the manipulator config during the rrt
    int parent = -1;
    int edge = -1; // index of edge from parent to this
    bool is_extended_to_goal = false;
    bool is_explored = false; // is this node being used in the MCTS
    bool has_been_root = false;
    std::vector<ContactPoint> envs;
    std::vector<VectorXi> modes;
    Node(Vector7d data) { config = data; }
  };

  struct Edge {
    VectorXi mode;
    std::vector<Vector7d> path;
    Edge(VectorXi m, std::vector<Vector7d> &p) : mode(m), path(p) {}
  };

  std::vector<Node> nodes;
  std::vector<Edge> edges;
  double angle_weight;
  double translation_weight;
  ReusableRRT(const double &a_weight, const double &p_weight)
      : angle_weight(a_weight), translation_weight(p_weight) {}
  double dist(const Vector7d &q1, const Vector7d &q2) {
    Vector3d p1(q1[0], q1[1], q1[2]);
    Vector3d p2(q2[0], q2[1], q2[2]);
    Quaterniond quat1(q1[6], q1[3], q1[4], q1[5]); // Quaterniond: w, x, y, z
    Quaterniond quat2(q2[6], q2[3], q2[4], q2[5]);
    Vector3d dp = p1 - p2;
    double dtrans = dp.norm();
    double angle = angBTquat(quat1, quat2);

    double d = this->translation_weight * dtrans + this->angle_weight * angle;
    return d;
  }

  std::vector<int> subtree_node_idxes(int root_idx, VectorXi mode) {
    // given the root node index of a subtree, find out all the node indices in
    // the subtree
    std::vector<int> all_idxes;
    all_idxes.push_back(root_idx);
    for (int k = root_idx + 1; k < this->nodes.size(); k++) {
      int kk = k;
      while (this->nodes[kk].parent > root_idx) {
        kk = this->nodes[kk].parent;
      }
      if (this->nodes[kk].parent == root_idx) {
        bool if_same_mode = ((this->edges[this->nodes[kk].edge].mode.head(
                                  this->nodes[root_idx].envs.size()) -
                              mode)
                                 .norm() < 1e-3);
        if (if_same_mode) {
          all_idxes.push_back(k);
        }
      }
    }
    return all_idxes;
  }

  int find_node(const Vector7d &q, VectorXi mode, int parent_idx,
                double thr = 1e-3) {
    // ss_mode: the mode that leads to this node
    int near_idx = 0;
    double min_d = this->dist(nodes[0].config, q);

    for (int i = 1; i < nodes.size(); i++) {
      if (nodes[i].parent != parent_idx) {
        continue;
      }
      if (edges[nodes[i].edge].mode.size() != mode.size()) {
        continue;
      }

      if ((edges[nodes[i].edge].mode - mode).norm() != 0) {
        continue;
      }

      double d = this->dist(nodes[i].config, q);
      if (d < min_d) {
        near_idx = i;
        min_d = d;
      }
    }

    return (min_d < thr) ? near_idx : -1;
  }

  int find_node(const Vector7d &q, int n_envs, int parent_idx,
                double thr = 1e-3){
      
    int near_idx = 0;
    double min_d = this->dist(nodes[0].config, q);

    for (int i = 1; i < nodes.size(); i++) {
      if (nodes[i].parent != parent_idx) {
        continue;
      }
      if (nodes[i].envs.size() != n_envs) {
        continue;
      }

      double d = this->dist(nodes[i].config, q);
      if (d < min_d) {
        near_idx = i;
        min_d = d;
      }
    }

    return (min_d < thr) ? near_idx : -1;
  }

  int find_node(const Vector7d &q, double thr = 1e-3) {
    int near_idx = 0;
    double min_d = this->dist(nodes[0].config, q);

    for (int i = 1; i < nodes.size(); i++) {
      double d = this->dist(nodes[i].config, q);
      if (d < min_d) {
        near_idx = i;
        min_d = d;
      }
    }
    return (min_d < thr) ? near_idx : -1;
  }

  int nearest_neighbor(const Vector7d &q) {
    int near_idx;
    double min_d = DBL_MAX;
    for (int i = 0; i < nodes.size(); i++) {
      double d = this->dist(nodes[i].config, q);
      if (d < min_d) {
        near_idx = i;
        min_d = d;
      }
    }
    return near_idx;
  }

  int nearest_neighbor_subtree(const Vector7d &q, int subtree_root_idx,
                               const std::vector<int> &subtree_idxes,
                               bool if_unextened_to_goal = false,
                               bool if_unexplored = false) {

    int near_idx = -1;
    double min_d = DBL_MAX;

    for (int i : subtree_idxes) {

      if (if_unextened_to_goal &&
          (this->nodes[i].is_extended_to_goal == true)) {
        continue;
      }

      double d = this->dist(nodes[i].config, q);
      if (d < min_d) {

        if (if_unexplored) {
          bool is_explored = false;
          int kk = i;
          while (kk > subtree_root_idx) {
            if (this->nodes[kk].is_explored) {
              is_explored = true;
              break;
            }
            kk = this->nodes[kk].parent;
          }
          if (is_explored) {
            continue;
          }
        }

        near_idx = i;
        min_d = d;
      }
    }
    return near_idx;
  }

  void backtrack(int last_node_idx, std::vector<int> *node_path,
                 int root_idx = 0) {
    int idx = last_node_idx;
    while (idx > root_idx) {
      node_path->push_back(idx);
      idx = nodes[idx].parent;
    }
    node_path->push_back(root_idx);
    return;
  }
  void add_node(Node *n, int parent_idx, Edge *e) {
    // int node_idx = nodes.size();
    int edge_idx = edges.size();
    n->parent = parent_idx;
    n->edge = edge_idx;

    nodes.push_back(*n);
    edges.push_back(*e);
    return;
  }

  void initial_node(Node *n) {
    n->parent = -1;
    nodes.push_back(*n);
    return;
  }
};
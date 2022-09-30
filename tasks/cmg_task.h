
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cfloat>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#ifndef UTILS_H
#define UTILS_H
#include "../mechanics/utilities/utilities.h"
#endif
#include "../mechanics/contacts/contact_mode_enumeration.h"
#include "../mechanics/utilities/sample.h"

#ifndef _WORLD_TEMPLATE
#define _WORLD_TEMPLATE
#include "../mechanics/worlds/WorldTemplate.h"
#endif

class RRTTree {
public:
  struct Node {
    Vector7d config; // x, y, z, qx, qy, qz, qw
    int parent = -1;
    int edge = -1; // index of edge from parent to this
    bool is_extended_to_goal = false;
    std::vector<ContactPoint> envs;
    std::vector<VectorXi> modes;
    Node(Vector7d data) { config = data; }
  };

  struct Edge {
    VectorXi mode;
    Edge(VectorXi m) : mode(m) {}
  };

  std::vector<Node> nodes;
  std::vector<Edge> edges;
  double angle_weight;
  double translation_weight;
  RRTTree(const double &a_weight, const double &p_weight)
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
  int nearest_unextended_to_goal(const Vector7d &q) {
    int near_idx;
    double min_d = DBL_MAX;
    for (int i = 0; i < nodes.size(); i++) {
      double d = this->dist(nodes[i].config, q);
      if ((d < min_d) && (!nodes[i].is_extended_to_goal)) {
        near_idx = i;
        min_d = d;
      }
    }
    if (min_d == DBL_MAX) {
      return -1;
    }

    return near_idx;
  }
  void backtrack(int last_node_idx, std::vector<int> *node_path) {
    int cur_node = last_node_idx;
    node_path->push_back(cur_node);
    bool is_start_node = nodes[cur_node].parent != -1;
    while (is_start_node) {
      cur_node = nodes[cur_node].parent;
      node_path->push_back(cur_node);
      is_start_node = nodes[cur_node].parent != -1;
    }
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

class CMGTASK {

public:
  struct State {
    Vector7d m_pose;
    std::vector<ContactPoint> envs;
    int m_mode_idx = -1; // the mode chosen for this state, to the next state
    std::vector<Eigen::VectorXi> modes;

    State() {}

    State(Vector7d pose, const std::vector<ContactPoint> &envs_, int mode_idx,
          const std::vector<Eigen::VectorXi> &modes_)
        : m_pose(pose), envs(envs_), m_mode_idx(mode_idx), modes(modes_) {}

    State(const State &state_) {
      // copy constructor
      m_pose = state_.m_pose;
      m_mode_idx = state_.m_mode_idx;
      modes = state_.modes;
      envs = state_.envs;
    }

    void do_action(int action) { m_mode_idx = action; }

    State &operator=(const State &state_) {
      this->m_pose = state_.m_pose;
      this->m_mode_idx = state_.m_mode_idx;
      this->modes = state_.modes;
      this->envs = state_.envs;
      return *this;
    }
  };

  struct SearchOptions {
    // the search options for search_a_new_path using RRT
    Eigen::Vector3d x_lb;
    Eigen::Vector3d x_ub;
    double eps_trans; // the maximum translation in a RRT extend
    double eps_angle; // the maximum rotation in a RRT extend
    int sampleSO3 = true;
    double goal_biased_prob = 0.8;
    int max_samples = 100;
    Vector3d sample_rotation_axis;

    SearchOptions() {}
  };

  CMGTASK() { this->cons = std::make_unique<ContactConstraints>(2); }

  void initialize(const Vector7d &start_object_pose,
                  const Vector7d &goal_object_pose, double goal_thr, double wa,
                  double wt, double charac_len, double mu_env, double mu_mnp,
                  Matrix6d object_inertia, Vector6d f_gravity,
                  std::shared_ptr<WorldTemplate> world,
                  const SearchOptions &options) {
    this->start_object_pose = start_object_pose;
    this->goal_object_pose = goal_object_pose;
    this->goal_thr = goal_thr;
    this->wa = wa;
    this->wt = wt;
    this->charac_len = charac_len;
    this->mu_env = mu_env;
    this->mu_mnp = mu_mnp;
    this->object_inertia = object_inertia;
    this->f_gravity = f_gravity;
    this->search_options = options;
    this->m_initialized = true;
    this->m_world = world;
  }

  State get_start_state() const { return generate_state(start_object_pose); }

  State generate_state(const Vector7d &object_pose) const;

  std::vector<State> search_a_new_path(const State &start_state);

  double evaluate_path(const std::vector<State> &path) const;

  bool forward_integration(const Vector7d &x_start, const Vector7d &x_goal,
                           const std::vector<ContactPoint> &envs_,
                           const VectorXi &env_mode_,
                           std::vector<Vector7d> *path);

private:
  bool m_initialized = false;
  Vector7d start_object_pose;
  Vector7d goal_object_pose;
  double goal_thr;
  double wa; // weigh the importance of angle
  double wt; // weigh the importance of translation

  double charac_len; // characteristic length

  double mu_mnp = 0.8;
  double mu_env = 0.5;

  Matrix6d object_inertia;
  Vector6d f_gravity;

  std::vector<ContactPoint> object_surface_pts;

  std::shared_ptr<WorldTemplate>
      m_world; // save the object, environment, do collision detections, ...

  SearchOptions search_options;

  std::unique_ptr<ContactConstraints> cons;
};
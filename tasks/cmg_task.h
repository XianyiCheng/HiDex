
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

#define CMG_QUASISTATIC 0
#define CMG_QUASIDYNAMIC 1
#define CMG_NODYNAMICS -1

class RRTTree
{
public:
  struct Node
  {
    Vector7d config; // x, y, z, qx, qy, qz, qw
    int parent = -1;
    int edge = -1; // index of edge from parent to this
    bool is_extended_to_goal = false;
    std::vector<ContactPoint> envs;
    std::vector<VectorXi> modes;
    Node(Vector7d data) { config = data; }
  };

  struct Edge
  {
    VectorXi mode;
    std::vector<Vector7d> path;
    Edge(VectorXi m, std::vector<Vector7d> &p) : mode(m), path(p) {}
  };

  std::vector<Node> nodes;
  std::vector<Edge> edges;
  double angle_weight;
  double translation_weight;
  RRTTree(const double &a_weight, const double &p_weight)
      : angle_weight(a_weight), translation_weight(p_weight) {}
  double dist(const Vector7d &q1, const Vector7d &q2)
  {
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

  int nearest_neighbor(const Vector7d &q)
  {
    int near_idx;
    double min_d = DBL_MAX;
    for (int i = 0; i < nodes.size(); i++)
    {
      double d = this->dist(nodes[i].config, q);
      if (d < min_d)
      {
        near_idx = i;
        min_d = d;
      }
    }
    return near_idx;
  }

  std::vector<int> nearest_neighbors(const Vector7d &q)
  {
    int near_idx = this->nearest_neighbor(q);
    double min_d = this->dist(nodes[near_idx].config, q);

    std::vector<int> near_idxes;
    for (int i = 0; i < nodes.size(); i++)
    {
      double d = this->dist(nodes[i].config, q);
      if ((d - min_d) * (d - min_d) < 1e-3)
      {
        near_idxes.push_back(i);
      }
    }
    return near_idxes;
  }

  int nearest_unextended_to_goal(const Vector7d &q)
  {
    int near_idx;
    double min_d = DBL_MAX;
    for (int i = 0; i < nodes.size(); i++)
    {
      double d = this->dist(nodes[i].config, q);
      if ((d < min_d) && (!nodes[i].is_extended_to_goal))
      {
        near_idx = i;
        min_d = d;
      }
    }
    if (min_d == DBL_MAX)
    {
      return -1;
    }

    return near_idx;
  }
  void backtrack(int last_node_idx, std::vector<int> *node_path)
  {
    int cur_node = last_node_idx;
    node_path->push_back(cur_node);
    bool is_start_node = nodes[cur_node].parent != -1;
    while (is_start_node)
    {
      cur_node = nodes[cur_node].parent;
      node_path->push_back(cur_node);
      is_start_node = nodes[cur_node].parent != -1;
    }
    return;
  }
  void add_node(Node *n, int parent_idx, Edge *e)
  {
    // int node_idx = nodes.size();
    int edge_idx = edges.size();
    n->parent = parent_idx;
    n->edge = edge_idx;

    nodes.push_back(*n);
    edges.push_back(*e);
    return;
  }

  void initial_node(Node *n)
  {
    n->parent = -1;
    nodes.push_back(*n);
    return;
  }
};

// Resuable RRT
class ReusableRRT
{
public:
  struct Node
  {
    Vector7d config; // x, y, z, qx, qy, qz, qw
    int parent = -1;
    int edge = -1; // index of edge from parent to this
    bool is_extended_to_goal = false;
    bool is_explored = false; // is this node being used in the MCTS
    bool has_been_root = false;
    std::vector<ContactPoint> envs;
    std::vector<VectorXi> modes;
    Node(Vector7d data) { config = data; }
  };

  struct Edge
  {
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
  double dist(const Vector7d &q1, const Vector7d &q2)
  {
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

  std::vector<int> subtree_node_idxes(int root_idx, VectorXi mode)
  {
    // given the root node index of a subtree, find out all the node indices in
    // the subtree
    std::vector<int> all_idxes;
    all_idxes.push_back(root_idx);
    for (int k = root_idx + 1; k < this->nodes.size(); k++)
    {
      int kk = k;
      while (this->nodes[kk].parent > root_idx)
      {
        kk = this->nodes[kk].parent;
      }
      if (this->nodes[kk].parent == root_idx)
      {
        bool if_same_mode = ((this->edges[this->nodes[kk].edge].mode.head(
                                  this->nodes[root_idx].envs.size()) -
                              mode)
                                 .norm() < 1e-3);
        if (if_same_mode)
        {
          all_idxes.push_back(k);
        }
      }
    }
    return all_idxes;
  }

  int find_node(const Vector7d &q, int parent_env_n, double thr = 1e-3)
  {
    int near_idx = 0;
    double min_d = this->dist(nodes[0].config, q);

    for (int i = 1; i < nodes.size(); i++)
    {
      if (nodes[nodes[i].parent].envs.size() != parent_env_n)
      {
        continue;
      }
      double d = this->dist(nodes[i].config, q);
      if (d < min_d)
      {
        near_idx = i;
        min_d = d;
      }
    }
    return (min_d < thr) ? near_idx : -1;
  }

  int find_node(const Vector7d &q, VectorXi ss_mode, int parent_idx, double thr = 1e-3)
  {
    // ss_mode: the mode that leads to this node
    int near_idx = 0;
    double min_d = this->dist(nodes[0].config, q);

    for (int i = 1; i < nodes.size(); i++)
    {
      if (nodes[i].parent != parent_idx)
      {
        continue;
      }
      if (edges[nodes[i].edge].mode.size() != ss_mode.size()){
        continue;
      }

      if ((edges[nodes[i].edge].mode - ss_mode).norm() != 0)
      {
        continue;
      }

      double d = this->dist(nodes[i].config, q);
      if (d < min_d)
      {
        near_idx = i;
        min_d = d;
      }
    }

    return (min_d < thr) ? near_idx : -1;
  }

  int nearest_neighbor(const Vector7d &q)
  {
    int near_idx;
    double min_d = DBL_MAX;
    for (int i = 0; i < nodes.size(); i++)
    {
      double d = this->dist(nodes[i].config, q);
      if (d < min_d)
      {
        near_idx = i;
        min_d = d;
      }
    }
    return near_idx;
  }

  int nearest_neighbor_subtree(const Vector7d &q, int subtree_root_idx,
                               const std::vector<int> &subtree_idxes,
                               bool if_unextened_to_goal = false,
                               bool if_unexplored = false)
  {

    int near_idx = -1;
    double min_d = DBL_MAX;

    for (int i : subtree_idxes)
    {

      if (if_unextened_to_goal &&
          (this->nodes[i].is_extended_to_goal == true))
      {
        continue;
      }

      double d = this->dist(nodes[i].config, q);
      if (d < min_d)
      {

        if (if_unexplored)
        {
          bool is_explored = false;
          int kk = i;
          while (kk > subtree_root_idx)
          {
            if (this->nodes[kk].is_explored)
            {
              is_explored = true;
              break;
            }
            kk = this->nodes[kk].parent;
          }
          if (is_explored)
          {
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
                 int root_idx = 0)
  {
    int idx = last_node_idx;
    while (idx > root_idx)
    {
      node_path->push_back(idx);
      idx = nodes[idx].parent;
    }
    node_path->push_back(root_idx);
    return;
  }
  void add_node(Node *n, int parent_idx, Edge *e)
  {
    // int node_idx = nodes.size();
    int edge_idx = edges.size();
    n->parent = parent_idx;
    n->edge = edge_idx;

    nodes.push_back(*n);
    edges.push_back(*e);
    return;
  }

  void initial_node(Node *n)
  {
    n->parent = -1;
    nodes.push_back(*n);
    return;
  }
};

class CMGTASK
{

public:
  struct State
  {
    Vector7d m_pose;
    std::vector<ContactPoint> envs;
    int m_mode_idx = -1; // the mode chosen for this state, to the next state
    std::vector<Eigen::VectorXi> modes;
    std::vector<Vector7d>
        m_path;            // the path to this state (TODO: only save this path when
                           // m_mode_idx = -1 (node type: "pose"))
    VectorXi path_ss_mode; // store the ss_mode of the path to this state

    State() {}

    State(Vector7d pose, const std::vector<ContactPoint> &envs_, int mode_idx,
          const std::vector<Eigen::VectorXi> &modes_)
        : m_pose(pose), envs(envs_), m_mode_idx(mode_idx), modes(modes_) {}

    State(const State &state_)
    {
      // copy constructor
      m_pose = state_.m_pose;
      m_mode_idx = state_.m_mode_idx;
      modes = state_.modes;
      envs = state_.envs;
      m_path = state_.m_path;
      path_ss_mode = state_.path_ss_mode;
    }

    void do_action(int action) { m_mode_idx = action; }

    State &operator=(const State &state_)
    {
      this->m_pose = state_.m_pose;
      this->m_mode_idx = state_.m_mode_idx;
      this->modes = state_.modes;
      this->envs = state_.envs;
      this->m_path = state_.m_path;
      this->path_ss_mode = state_.path_ss_mode;
      return *this;
    }
  };

  struct State2
  {
    int timestep = 0;
    int finger_index; // current finger index
    bool is_valid;
    State2() {}
    State2(int t, int idx) : timestep(t), finger_index(idx) {}
    void do_action(int action)
    {
      this->finger_index = action;
      this->timestep++;
    }
  };

  struct SearchOptions
  {
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
                  const SearchOptions &options);

  // --- Level 1 Tree functions ---
  State get_start_state() const { return generate_state(start_object_pose); }

  State generate_state(const Vector7d &object_pose) const;

  std::vector<State> search_a_new_path(const State &start_state);

  std::vector<State> search_a_new_path_old(const State &start_state);

  bool forward_integration(const Vector7d &x_start, const Vector7d &x_goal,
                           const std::vector<ContactPoint> &envs_,
                           const VectorXi &env_mode_,
                           std::vector<Vector7d> *path);

  double evaluate_path(const std::vector<State> &path) const
  {
    // return the REWARD of the path: larger reward -> better path

    // TODO: define reward
    // double reward = 1 / (double(path.size()-7)*double(path.size()-7)+1);

    // double avg_maintain_contact = 0;
    // for (auto s:path){
    //   if (s.m_mode_idx!=-1){
    //     VectorXi mode = s.modes[s.m_mode_idx];
    //     avg_maintain_contact += 1 -
    //     (double(mode.sum())+1)/(double(mode.size())+1);
    //   }
    // }
    // avg_maintain_contact = avg_maintain_contact/double(path.size());

    // double reward = 1 / double(path.size()) + avg_maintain_contact;
    double reward = 1 / double(path.size());

    return reward;
  }

  // --- Level 2 Tree functions for robot contact planning ----

  std::vector<int> get_finger_locations(int finger_location_index)
  {

    int N = this->object_surface_pts.size();
    int n = this->number_of_robot_contacts;
    int x = finger_location_index;

    std::vector<int> finger_locations;
    for (int k = 0; k < n; ++k)
    {
      int a = int(x / pow(N, (n - k - 1)));
      x -= a * (int)pow(N, (n - k - 1));

      finger_locations.push_back(a);
    }
    return finger_locations;
  }

  VectorXd get_robot_config_from_action_idx(int action_index)
  {

    std::vector<int> finger_locations =
        this->get_finger_locations(action_index);

    // for point fingers
    VectorXd mnp_config(6 * finger_locations.size());
    for (int k = 0; k < finger_locations.size(); ++k)
    {
      mnp_config.block(6 * k, 0, 3, 1) =
          this->object_surface_pts[finger_locations[k]].p;
      mnp_config.block(6 * k + 3, 0, 3, 1) =
          this->object_surface_pts[finger_locations[k]].n;
    }

    return mnp_config;
  }

  State2 get_start_state2() const
  {
    State2 state(0, -1);
    return state;
  }
  double evaluate_path(const std::vector<State2> &path) const
  {

    if (!path.back().is_valid)
    {
      return 0.0;
    }

    int number_of_finger_changes = 0;
    for (int k = 0; k < path.size() - 1; ++k)
    {
      if (path[k].finger_index != path[k + 1].finger_index)
      {
        number_of_finger_changes++;
      }
    }

    // double reward_finger_stay =
    //     1.0 - (double(number_of_finger_changes)) / double(path.size());
    double reward_finger_stay =
        1.0 - (double(number_of_finger_changes * number_of_finger_changes)) / (2 * double(path.size()));
    reward_finger_stay = std::max(reward_finger_stay, 0.0);

    double reward_path_size = 3.0 / double(path.size());

    // double reward_finger_1 = 1.0 - finger_number / double(path.size());

    return reward_finger_stay + reward_path_size;
  }

  double estimate_next_state_value(const State2 &state, int action)
  {
    return 0.0;
    // return 0.0 for now, can use neural networks to estimate values
  }

  int action_selection_policy_level2(const State2 &state,
                                     const State2 &pre_state)
  {
    // Manually assigned policy: Specific to the planning robot contact problem:
    // with 0.5 probability, we will select the current action
    if (randd() > 0.5)
    {
      return pre_state.finger_index;
    }
    else
    {
      return -1;
    }
  }

  int action_selection_policy_level1(const State &state,
                                     const State &pre_state)
  {
    // Manually assigned policy: only for select modes, select the same mode as
    // previous mode
    if (randd() > 0.5)
    {
      return pre_state.m_mode_idx;
    }
    else
    {
      return -1;
    }
  }

  int get_number_of_robot_actions(const State2 &state)
  {
    // return combination of fingers for now
    return pow(this->object_surface_pts.size(), this->number_of_robot_contacts);
  }

  bool is_terminal(const State2 &state)
  {
    // check if terminal, also check if valid,
    // if not valid it is also terminal
    if (!state.is_valid)
    {
      return true;
    }
    else
    {
      if (state.timestep >= this->saved_object_trajectory.size() - 1)
      {
        return true;
      }
    }
    return false;
  }

  bool is_valid(const State2 &state);

  std::vector<State> saved_object_trajectory;
  std::vector<ContactPoint> object_surface_pts;
  int number_of_robot_contacts;

  int task_dynamics_type = CMG_QUASISTATIC;

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

  std::shared_ptr<WorldTemplate>
      m_world; // save the object, environment, do collision detections, ...

  SearchOptions search_options;

  std::unique_ptr<ContactConstraints> cons;

  std::shared_ptr<ReusableRRT> shared_rrt;
};
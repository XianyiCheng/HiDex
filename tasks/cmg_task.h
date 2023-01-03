
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
#include "../mechanics/contacts/contact_mode_enumeration.h"

#ifndef SAMPLE_H
#define SAMPLE_H
#include "../mechanics/utilities/sample.h"
#endif

#ifndef _WORLD_TEMPLATE
#define _WORLD_TEMPLATE
#include "../mechanics/worlds/WorldTemplate.h"
#endif

#ifndef RRT_H
#define RRT_H
#include "rrt.h"
#endif

#define CMG_QUASISTATIC 0
#define CMG_QUASIDYNAMIC 1
#define CMG_NODYNAMICS -1

class CMGTASK
{

public:
  struct State
  {
    typedef int Action;
    static const Action no_action;
    Vector7d m_pose;
    std::vector<ContactPoint> envs;
    int m_mode_idx = -1; // the mode chosen for this state, to the next state
    std::vector<Eigen::VectorXi> modes;
    std::vector<Vector7d>
        m_path; // the path to this state (TODO: only save this path when
                // m_mode_idx = -1 (node type: "pose"))

    static Action action_index_to_action(int action_idx)
    {
      Action action = action_idx;
      return action;
    }

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
    }

    void do_action(Action action) { m_mode_idx = action; }

    State &operator=(const State &state_)
    {
      this->m_pose = state_.m_pose;
      this->m_mode_idx = state_.m_mode_idx;
      this->modes = state_.modes;
      this->envs = state_.envs;
      this->m_path = state_.m_path;
      return *this;
    }
  };

  struct State2
  {
    struct Action
    {
      long int finger_idx = -1;
      int timestep = -1;
      Action() {}
      Action(int timestep_, long int finger_idx_) : timestep(timestep_), finger_idx(finger_idx_) {}
      Action &operator=(const Action &action_)
      {
        this->finger_idx = action_.finger_idx;
        this->timestep = action_.timestep;
        return *this;
      }
      bool operator==(const Action &action_) const
      {
        return (timestep == action_.timestep && finger_idx == action_.finger_idx);
      }
    };
    static const Action no_action;

    int timestep = 0;
    long int finger_index; // current finger index
    bool is_valid;
    int t_max = -1; // maximum time step this can reach
    State2() {}
    State2(int t, long int idx) : timestep(t), finger_index(idx) {}
    void do_action(Action action)
    {
      this->finger_index = action.finger_idx;
      this->timestep = action.timestep;
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

    bool control_neighbors = false;

    SearchOptions() {}
  };

  CMGTASK() { this->cons = std::make_unique<ContactConstraints>(2); }

  void initialize(const Vector7d &start_object_pose,
                  const Vector7d &goal_object_pose, double goal_thr, double wa,
                  double wt, double charac_len, double mu_env, double mu_mnp,
                  Matrix6d object_inertia, Vector6d f_gravity,
                  std::shared_ptr<WorldTemplate> world, int n_robot_contacts,
                  int dynamic_type, std::vector<ContactPoint> surface_pts,
                  const SearchOptions &options, bool if_refine = false,
                  double refine_dist = 0.0);

  int neighbors_on_the_same_manifold(const Vector7d &q, std::vector<ContactPoint> envs, std::vector<VectorXi> env_modes,
                                     double dist_thr);

  // --- Level 1 Tree functions ---
  State get_start_state() const { return generate_state(start_object_pose); }

  State generate_state(const Vector7d &object_pose) const;

  std::vector<State> search_a_new_path(const State &start_state);

  // std::vector<State> search_a_new_path_old(const State &start_state);

  bool forward_integration(const Vector7d &x_start, const Vector7d &x_goal,
                           const std::vector<ContactPoint> &envs_,
                           const VectorXi &env_mode_,
                           std::vector<Vector7d> *path);

  double travel_distance(const std::vector<State> &path) const
  {
    double dist = 0.0;

    for (int i = 0; i < path.size() - 1; ++i)
    {
      dist += this->shared_rrt->dist(path[i].m_pose, path[i + 1].m_pose);
    }
    return dist;
  }

  double evaluate_path(const std::vector<State> &path) const
  {
    double reward = 1 / double(path.size());

    return reward;
  }

  double evaluate_path(const std::vector<State> &path, const std::vector<State2> &path2)
  {
    //

    double dist = this->travel_distance(path);
    double best_dist = this->shared_rrt->dist(this->start_object_pose, this->goal_object_pose);
    double x_dist = dist / best_dist;
    double y_dist = 3.39617221 * x_dist - 7.59164285;

    double x_path = double(path.size());
    double y_path = 0.64872688 * x_path - 4.52948518;

    double total_finger_changes = this->total_finger_change_ratio(path2);

    double x_finger = total_finger_changes / double(path2.back().t_max);
    double y_finger = 11.14845406 * x_finger - 4.59804752;

    double r_dist = 1.0 / (1.0 + std::exp(y_dist));
    double r_path = 1.0 / (1.0 + std::exp(y_path));
    double r_finger = 1.0 / (1.0 + std::exp(y_finger));

    double reward = 0.4 * r_dist + 0.4 * r_path + 0.2 * r_finger;

    return reward;
  }

  // --- Level 2 Tree functions for robot contact planning ----

  std::vector<int> get_finger_locations(long int finger_location_index);

  VectorXd get_robot_config_from_action_idx(long int action_index)
  {

    std::vector<int> finger_locations =
        this->get_finger_locations(action_index);

    // for point fingers
    VectorXd mnp_config(6 * finger_locations.size());
    for (int k = 0; k < finger_locations.size(); ++k)
    {
      if (finger_locations[k] == -1)
      {
        // temporary solution:
        // when not in contact, set the finger location to a very far away
        // point, only works for point fingers
        // TODO: for other robots, need to consider IK, collision, etc.
        mnp_config.block(6 * k, 0, 3, 1) = Vector3d(100, 100, 100);
        mnp_config.block(6 * k + 3, 0, 3, 1) = Vector3d::Zero();
        continue;
      }
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

  double total_finger_change_ratio(const std::vector<State2> &path);
  double evaluate_path(const std::vector<State2> &path);

  double estimate_next_state_value(const State2 &state, State2::Action action)
  {
    // return 0.0 for now, can use neural networks to estimate values
    return 0.0;
  }

  double action_heuristics_level2(State2::Action action, const State2 &state,
                                  const State2 &pre_state)
  {
    // return the heuristics of an action in level2, this can be hand designed
    // or learned todo: improve this heuristics
    std::vector<int> finger_locations_1 =
        this->get_finger_locations(pre_state.finger_index);
    std::vector<int> finger_locations_2 =
        this->get_finger_locations(action.finger_idx);

    double heu = 1.0;
    for (int i = 0; i < finger_locations_1.size(); ++i)
    {
      if (finger_locations_1[i] == finger_locations_2[i])
      {
        heu *= 2.0;
      }
    }

    return heu;
  }

  long int get_number_of_robot_actions(const State2 &state)
  {
    return this->n_finger_combinations;
  }

  long int get_number_of_actions(const State2 &state)
  {
    return this->n_finger_combinations * this->saved_object_trajectory.size();
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
    if (state.t_max == (this->saved_object_trajectory.size() - 1))
    {
      return true;
    }
    return false;
  }

  int total_rrt_nodes() { return shared_rrt->nodes.size(); }

  bool is_valid(const State2 &state, const State2 &prev_state);

  bool is_finger_valid(long int finger_idx, int timestep);

  bool is_valid_transition(const State2 &state, const State2 &prev_state);

  void save_trajectory(const std::vector<CMGTASK::State> &path);

  std::vector<State>
  generate_a_finer_object_trajectory(std::vector<State> &object_traj,
                                     double dist);

  bool robot_contact_feasibile_check(long int finger_idx, const Vector7d &x, const VectorXi &cs_mode, const Vector6d &v,
                                     const std::vector<ContactPoint> &envs);

  void sample_likely_feasible_finger_idx(Vector7d x_object, int number, std::vector<long int> *finger_idxs, std::vector<double>* probabilities)
  {
    for (int n = number; n > 0; n--)
    {
      finger_idxs->push_back(randi(this->n_finger_combinations));
      probabilities->push_back(1.0);
    }
  }

  void sample_likely_feasible_finger_idx(State2 state, double t_change, int number, std::vector<long int> *finger_idxs, std::vector<double>* probabilities)
  {
    for (int n = number; n > 0; n--)
    {
      finger_idxs->push_back(randi(this->n_finger_combinations));
      probabilities->push_back(1.0);
    }
  }

  long int pruning_check(const Vector7d &x, const Vector6d &v,
                    const std::vector<ContactPoint> &envs);
  long int pruning_check(const Vector7d &x, const VectorXi &cs_mode, const Vector6d &v,
                    const std::vector<ContactPoint> &envs);

  int max_forward_timestep(const CMGTASK::State2 &state);
  int select_finger_change_timestep(const State2 &state);

  long int encode_action_idx(long int finger_idx, int timestep)
  {
    return timestep * this->n_finger_combinations + finger_idx;
  }

  // void do_action(State2 &state, State::Action action) {
  //   state.timestep = action / this->n_finger_combinations;
  //   state.finger_index = action % this->n_finger_combinations;
  // }

  std::vector<State> saved_object_trajectory;
  std::vector<ContactPoint> object_surface_pts;
  int number_of_robot_contacts;

  int task_dynamics_type = CMG_QUASISTATIC;

  std::shared_ptr<WorldTemplate>
      m_world; // save the object, environment, do collision detections, ...
  unsigned long int n_finger_combinations = 0;

  std::shared_ptr<ReusableRRT> shared_rrt;

  Vector7d start_object_pose;
  Vector7d goal_object_pose;

private:
  bool m_initialized = false;

  double goal_thr;
  double wa; // weigh the importance of angle
  double wt; // weigh the importance of translation

  double charac_len; // characteristic length

  double mu_mnp = 0.8;
  double mu_env = 0.5;

  Matrix6d object_inertia;
  Vector6d f_gravity;

  // std::shared_ptr<WorldTemplate>
  //     m_world; // save the object, environment, do collision detections, ...

  SearchOptions search_options;

  std::unique_ptr<ContactConstraints> cons;

  bool if_refine = false;
  bool refine_dist = 0.0;
};

bool isQuasistatic(const std::vector<ContactPoint> &mnps,
                   const std::vector<ContactPoint> &envs, const Vector6d &v,
                   const Vector6d &f_ext_w, const Vector7d object_pose,
                   double mu_env, double mu_mnp, ContactConstraints *cons);
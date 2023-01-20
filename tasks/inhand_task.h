
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

class InhandTASK {

public:
  struct State {
    typedef int Action;
    static const Action no_action;
    Vector7d m_pose;
    std::vector<ContactPoint> envs;
    int m_mode_idx = -1; // the mode chosen for this state, to the next state
    std::vector<Eigen::VectorXi> modes;
    std::vector<Vector7d>
        m_path; // the path to this state (TODO: only save this path when
                // m_mode_idx = -1 (node type: "pose"))

    static Action action_index_to_action(int action_idx) {
      Action action = action_idx;
      return action;
    }

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
      m_path = state_.m_path;
    }

    void do_action(Action action) { m_mode_idx = action; }

    State &operator=(const State &state_) {
      this->m_pose = state_.m_pose;
      this->m_mode_idx = state_.m_mode_idx;
      this->modes = state_.modes;
      this->envs = state_.envs;
      this->m_path = state_.m_path;
      return *this;
    }
  };

  struct State2 {
    struct Action {
      long int finger_idx = -1;
      int timestep = -1;
      Action() {}
      Action(int timestep_, long int finger_idx_)
          : timestep(timestep_), finger_idx(finger_idx_) {}
      Action &operator=(const Action &action_) {
        this->finger_idx = action_.finger_idx;
        this->timestep = action_.timestep;
        return *this;
      }
      bool operator==(const Action &action_) const {
        return (timestep == action_.timestep &&
                finger_idx == action_.finger_idx);
      }
    };

    static const Action no_action;

    int timestep = 0;
    long int finger_index; // current finger index
    bool is_valid;
    int t_max = -1; // maximum time step this can reach
    State2() {}
    State2(int t, int idx) : timestep(t), finger_index(idx) {}
    void do_action(Action action) {
      this->finger_index = action.finger_idx;
      this->timestep = action.timestep;
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

  InhandTASK() { this->cons = std::make_unique<ContactConstraints>(2); }

  void initialize(const Vector7d &start_object_pose,
                  const Vector7d &goal_object_pose, long int start_finger_idx,
                  long int goal_finger_idx, double goal_thr, double wa,
                  double wt, double charac_len, double mu_env, double mu_mnp,
                  Vector6d f_gravity, std::shared_ptr<WorldTemplate> world,
                  int n_robot_contacts, std::vector<ContactPoint> surface_pts,
                  const SearchOptions &options, bool if_refine = false,
                  double refine_dist = 0.0);

  // --- Level 1 Tree functions ---
  State get_start_state() const { return generate_state(start_object_pose); }

  State generate_state(const Vector7d &object_pose) const;

  std::vector<State> search_a_new_path(const State &start_state);

  // std::vector<State> search_a_new_path_old(const State &start_state);

  bool forward_integration(const Vector7d &x_start, const Vector7d &x_goal,
                           const std::vector<ContactPoint> &envs_,
                           const VectorXi &env_mode_,
                           std::vector<Vector7d> *path);

  double travel_distance(const std::vector<State> &path) const {
    double dist = 0.0;

    for (int i = 0; i < path.size() - 1; ++i) {
      dist += this->shared_rrt->dist(path[i].m_pose, path[i + 1].m_pose);
    }
    return dist;
  }

  double evaluate_path(const std::vector<State> &path) const {
    //

    double dist = this->travel_distance(path);

    double dist_reward = 1.0 / (1.0 + std::exp(dist));
    double depth_reward = 1.0 / (1.0 + std::exp(double(path.size())));

    double reward = dist_reward + depth_reward;

    return reward;
  }

  double evaluate_path(const std::vector<State> &path,
                       const std::vector<State2> &path2) {
    //

    double dist = this->travel_distance(path);
    double best_dist =
        this->shared_rrt->dist(this->start_object_pose, this->goal_object_pose);
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

  long int finger_locations_to_finger_idx(const std::vector<int> &finger_idxs);

  void sample_likely_feasible_finger_idx(Vector7d x_object, int number,
                                         std::vector<long int> *finger_idxs,
                                         std::vector<double> *probabilities);

  void sample_likely_feasible_finger_idx(State2 state, double t_change,
                                         int number,
                                         std::vector<long int> *finger_idxs,
                                         std::vector<double> *probabilities);

  VectorXd get_robot_config_from_action_idx(long int action_index) {

    std::vector<int> finger_locations =
        this->get_finger_locations(action_index);

    // for point fingers
    VectorXd mnp_config(6 * finger_locations.size());
    for (int k = 0; k < finger_locations.size(); ++k) {
      if (finger_locations[k] == -1) {
        // temporary solution:
        // when not in contact, set the finger location to a very far away
        // point, only works for point fingers
        // TODO: for other robots, need to consider IK, collision, etc.
        mnp_config.block(6 * k, 0, 3, 1) =
            Vector3d(std::nan(""), std::nan(""), std::nan(""));
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

  State2 get_start_state2() const {
    State2 state(0, this->start_finger_idx);
    return state;
  }

  double total_finger_change_ratio(const std::vector<State2> &path);

  double evaluate_path(const std::vector<State2> &path);

  double estimate_next_state_value(const State2 &state, State2::Action action) {
    // return 0.0 for now, can use neural networks to estimate values
    return 0.0;
  }

  double action_heuristics_level2(State2::Action action, const State2 &state,
                                  const State2 &pre_state) {
    // return the heuristics of an action in level2, this can be hand designed
    // or learned todo: improve this heuristics
    std::vector<int> finger_locations_1 =
        this->get_finger_locations(pre_state.finger_index);
    std::vector<int> finger_locations_2 =
        this->get_finger_locations(action.finger_idx);

    double heu = 1.0;
    for (int i = 0; i < finger_locations_1.size(); ++i) {
      if (finger_locations_1[i] == finger_locations_2[i]) {
        heu *= 2.0;
      }
    }

    return heu;
  }

  int get_number_of_robot_actions(const State2 &state) {
    return this->n_finger_combinations;
  }

  int get_number_of_actions(const State2 &state) {
    return this->n_finger_combinations * this->saved_object_trajectory.size();
  }

  bool is_terminal(const State2 &state) {
    // TODO: change this

    // check if terminal, also check if valid,
    // if not valid it is also terminal

    if (!state.is_valid) {
      return true;
    } else {
      if (state.timestep >= this->saved_object_trajectory.size() - 1) {
        return true;
      }
    }
    if (this->if_goal_finger) {
      if (state.t_max == (this->saved_object_trajectory.size() - 1)) {

        double total_finger_distance =
            this->get_finger_distance(state.finger_index);
        if (total_finger_distance < this->goal_finger_distance_thr) {
          return true;
        }
      }
      return false;
    }
    if (state.t_max == (this->saved_object_trajectory.size() - 1)) {
      return true;
    }
    return false;
  }

  int total_rrt_nodes() { return shared_rrt->nodes.size(); }

  // bool is_valid(const State2 &state, const State2 &prev_state);

  bool is_finger_valid(long int finger_idx, int timestep);

  bool is_valid_transition(const State2 &state, const State2 &prev_state);

  bool is_valid_transition(long int pre_finger_idx, long int finger_idx,
                           const Vector7d &x,
                           const std::vector<ContactPoint> &envs);

  void save_trajectory(const std::vector<State> &path);

  std::vector<State>
  generate_a_finer_object_trajectory(std::vector<State> &object_traj,
                                     double dist);

  bool robot_contact_feasibile_check(long int finger_idx, const Vector7d &x,
                                     const VectorXi &cs_mode, const Vector6d &v,
                                     const std::vector<ContactPoint> &envs);

  long int pruning_check(const Vector7d &x, const Vector6d &v,
                         const std::vector<ContactPoint> &envs);

  long int pruning_check(const Vector7d &x, const VectorXi &cs_mode,
                         const Vector6d &v,
                         const std::vector<ContactPoint> &envs);

  long int
  pruning_check_w_transition(const Vector7d &x, const Vector7d &x_pre,
                             const VectorXi &cs_mode, const Vector6d &v,
                             const std::vector<ContactPoint> &envs,
                             const std::vector<ContactPoint> &envs_pre);

  int max_forward_timestep(const State2 &state);
  int select_finger_change_timestep(const State2 &state);

  void
  set_goal_finger_locations(const std::vector<Vector3d> &goal_finger_locations_,
                            double goal_finger_distance_thr = 1.0) {
    this->if_goal_finger = true;
    this->goal_finger_locations = goal_finger_locations_;
    this->goal_finger_distance_thr = goal_finger_distance_thr;
  }

  double get_finger_distance(int finger_idx) {
    double total_finger_distance = 0.0;
    std::vector<int> fingertip_idx = this->get_finger_locations(finger_idx);
    for (int k = 0; k < this->number_of_robot_contacts; k++) {
      if (this->goal_finger_locations[k].norm() > 0.0) {
        int idx = fingertip_idx[k];
        Vector3d p;
        if (idx < 0) {
          p.setZero();
        } else {
          p = this->object_surface_pts[idx].p;
        }
        double d = (p - this->goal_finger_locations[k]).norm();
        total_finger_distance += d;
      }
    }
    return total_finger_distance;
  }

  double finger_idx_prob(long int finger_idx, int t) {
    if (this->if_goal_finger) {
      double d = this->get_finger_distance(finger_idx);
      d = d / double(this->number_of_robot_contacts);
      double p = 1.0 / (1.0 + std::exp(4.48413179 * d - 2.2420659));
      return p;
    } else {
      // use grasp measure
      if (this->grasp_measure_charac_length <= 0){
        return 1.0;
      }
      if (t == -1) {
        return 1.0;
      }
      double x_grasp = this->grasp_measure(finger_idx, t);
      double y_grasp = 6.90675 * x_grasp - 6.90675;
      double p = 1.0 / (1.0 + std::exp(y_grasp));
      return p;
    }
  }

  double grasp_measure(long int finger_idx, int timestep) {

    // measure the distance between the centroid of grasp and object com

    Vector3d centroid;
    centroid.setZero();
    int n_pts = 0;

    std::vector<int> fingertip_idx = this->get_finger_locations(finger_idx);
    for (int k = 0; k < this->number_of_robot_contacts; k++) {
      int idx = fingertip_idx[k];
      if (idx >= 0) {
        centroid += this->object_surface_pts[idx].p;
        n_pts += 1;
      }
    }
    for (auto pt : this->saved_object_trajectory[timestep].envs) {
      centroid += pt.p;
      n_pts += 1;
    }

    centroid /= double(n_pts);

    double d = centroid.norm() / this->grasp_measure_charac_length;

    return d;
  }

  // unsigned long int encode_action_idx(int finger_idx, int timestep)
  // {
  //     unsigned long int t = timestep;
  //     unsigned long int f = this->n_finger_combinations;
  //     unsigned long int action_idx = t * f + finger_idx;
  //     return action_idx;
  // }

  // void do_action(State2 &state, unsigned long int action)
  // {
  //     state.timestep = action / this->n_finger_combinations;
  //     state.finger_index = action % this->n_finger_combinations;
  // }

  double grasp_measure_charac_length = -1.0;
  std::vector<State> saved_object_trajectory;
  std::vector<ContactPoint> object_surface_pts;
  int number_of_robot_contacts;

  std::shared_ptr<WorldTemplate>
      m_world; // save the object, environment, do collision detections, ...
  unsigned long int n_finger_combinations = 0;

  std::shared_ptr<ReusableRRT> shared_rrt;

  Vector7d start_object_pose;
  Vector7d goal_object_pose;

  bool if_transition_pruning = false;

  bool if_goal_finger = false;

private:
  bool m_initialized = false;

  long int start_finger_idx = -1;
  long int goal_finger_idx = -1;

  double goal_thr;
  double wa; // weigh the importance of angle
  double wt; // weigh the importance of translation

  double charac_len; // characteristic length

  double mu_mnp = 0.8;
  double mu_env = 0.4;

  Matrix6d object_inertia;
  Vector6d f_gravity;

  // std::shared_ptr<WorldTemplate>
  //     m_world; // save the object, environment, do collision detections, ...

  SearchOptions search_options;

  std::unique_ptr<ContactConstraints> cons;

  bool if_refine = false;
  bool refine_dist = 0.0;

  
  double goal_finger_distance_thr = 1.0;
  std::vector<Vector3d> goal_finger_locations;
};

bool force_closure(Vector7d x, const std::vector<ContactPoint> &mnps,
                   double friction_coeff);
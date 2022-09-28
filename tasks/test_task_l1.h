
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "../mechanics/utilities/utilities.h"

class TestTaskL1
{

public:
  struct State
  {
    Vector7d m_pose;
    int m_mode_idx = -1; // the mode chosen for this state, to the next state
    std::vector<Eigen::VectorXi> modes;

    State() {}

    State(const State &state_)
    {
      // copy constructor
      m_pose = state_.m_pose;
      m_mode_idx = state_.m_mode_idx;
      modes = state_.modes;
    }

    void do_action(int action) { m_mode_idx = action; }

    State &operator=(const State &state_)
    {
      this->m_pose = state_.m_pose;
      this->m_mode_idx = state_.m_mode_idx;
      this->modes = state_.modes;
      return *this;
    }
  };

  struct SearchOptions
  {
    Eigen::Vector3d X_l;
    Eigen::Vector3d X_u;

    SearchOptions(Eigen::Vector3d X_l_, Eigen::Vector3d X_u_)
        : X_l(X_l_), X_u(X_u_) {}

    SearchOptions() {}

    SearchOptions(const SearchOptions &opts) : X_l(opts.X_l), X_u(opts.X_u) {}
    SearchOptions &operator=(const SearchOptions &opts)
    {
      X_l = opts.X_l;
      X_u = opts.X_u;
      return *this;
    }
  };

  Vector7d start_object_pose;
  Vector7d goal_object_pose;

  SearchOptions search_options;

  TestTaskL1() {}

  void initialize(const Vector7d &start_object_pose_,
                  const Vector7d &goal_object_pose_,
                  const SearchOptions &options)
  {
    this->start_object_pose = start_object_pose_;
    this->goal_object_pose = goal_object_pose_;
    this->search_options = options;
  }

  State get_start_state() const { return generate_state(start_object_pose); }

  State generate_state(const Vector7d &object_pose) const
  {
    // mode enumeration

    State state_;
    state_.m_pose = object_pose;

    // TODO: contact detection and contact mode enumeration

    Eigen::VectorXi mode1(2);
    mode1 << 0, 0;
    Eigen::VectorXi mode2(2);
    mode2 << 1, 1;
    state_.modes.push_back(mode1);
    state_.modes.push_back(mode2);

    return state_;
  }

  std::vector<State> search_a_new_path(const State &start_state)
  {
    // search a new path towards the end, given the START_STATE and M_MODE_IDX!!!

    // during the search, it figure out the constriants (modes) for the states
    // for now just return linear interpolation towards the sampled state

    // every state need to be associated with m_mode_idx (mode to this state)

    // int action_idx = start_state.m_mode_idx;

    std::vector<State> path_;

    if (randd() < 0.5)
    {

      Vector7d mid_pose;
      mid_pose.tail(4) = this->start_object_pose.tail(4);
      mid_pose.head(3) =
          (this->start_object_pose.head(3) + this->goal_object_pose.head(3)) /
          2;

      State state_1 = generate_state(mid_pose);
      state_1.m_mode_idx = (double(std::rand() / RAND_MAX) < 0.5) ? 0 : 1;

      State state_2 = generate_state(this->goal_object_pose);
      // state_2.m_mode_idx = (double(std::rand() / RAND_MAX) < 0.5) ? 0 : 1;

      path_.push_back(state_1);
      path_.push_back(state_2);
    }
    else
    {
      for (int k = 0; k < 3; ++k)
      {
        Vector7d mid_pose;
        mid_pose.tail(4) = this->start_object_pose.tail(4);
        Eigen::Vector3d t(randd(),
                          randd(),
                          randd());
        Eigen::Vector3d q;
        mid_pose.head(3) =
            t.cwiseProduct(search_options.X_u - search_options.X_l) +
            search_options.X_l;

        State state_ = generate_state(mid_pose);
        state_.m_mode_idx = (randd() < 0.5) ? 0 : 1;
        path_.push_back(state_);
      }
    }
    return path_;
    // sample between goal state and randomly sample region
  }

  double evaluate_path(const std::vector<State> &path)
  {
    // return the REWARD of the path: larger reward -> better path

    return 1/double(path.size());
    // return 1/(1+((double(path.size()) -7)*(double(path.size()) -7)));
  }

private:
  bool m_initialized;
};
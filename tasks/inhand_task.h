
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

class InhandTASK
{

public:
    struct State
    {
        Vector7d m_pose;
        std::vector<ContactPoint> envs;
        int m_mode_idx = -1; // the mode chosen for this state, to the next state
        std::vector<Eigen::VectorXi> modes;
        std::vector<Vector7d>
            m_path; // the path to this state (TODO: only save this path when
                    // m_mode_idx = -1 (node type: "pose"))

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

        void do_action(int action) { m_mode_idx = action; }

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
        int timestep = 0;
        int finger_index; // current finger index
        bool is_valid;
        int t_max = -1; // maximum time step this can reach
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

    InhandTASK() { this->cons = std::make_unique<ContactConstraints>(2); }

    void initialize(const Vector7d &start_object_pose,
                    const Vector7d &goal_object_pose, int start_finger_idx, int goal_finger_idx, double goal_thr, double wa,
                    double wt, double charac_len, double mu_env, double mu_mnp,  Vector6d f_gravity,
                    std::shared_ptr<WorldTemplate> world, int n_robot_contacts,
                    std::vector<ContactPoint> surface_pts,
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

    double evaluate_path(const std::vector<State> &path) const
    {
        //

        double dist = 0.0;

        for(int i = 0; i < path.size() - 1; ++i){
            dist += this->shared_rrt->dist(path[i].m_pose, path[i+1].m_pose);
        }

        double dist_reward = 1.0 / (1.0 + std::exp(dist));
        double depth_reward = 1.0 / (1.0 + std::exp(double(path.size())));

        double reward = dist_reward + depth_reward;

        return reward;
    }

    // --- Level 2 Tree functions for robot contact planning ----

    std::vector<int> get_finger_locations(int finger_location_index);

    int finger_locations_to_finger_idx(const std::vector<int> &finger_idxs);

    void sample_likely_feasible_finger_idx(Vector7d x_object, int number, std::vector<int> *finger_idxs);

    void sample_likely_feasible_finger_idx(State2 state, double t_change, int number, std::vector<int> *finger_idxs);

    VectorXd get_robot_config_from_action_idx(int action_index)
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
                mnp_config.block(6 * k, 0, 3, 1) = Vector3d(std::nan(""), std::nan(""), std::nan(""));
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
        State2 state(0, this->start_finger_idx);
        return state;
    }

    double total_finger_change_ratio(const std::vector<State2> &path);

    double evaluate_path(const std::vector<State2> &path);

    double estimate_next_state_value(const State2 &state, int action)
    {
        // return 0.0 for now, can use neural networks to estimate values
        return 0.0;
    }

    double action_heuristics_level2(int action_idx, const State2 &state,
                                    const State2 &pre_state)
    {
        // return the heuristics of an action in level2, this can be hand designed
        // or learned todo: improve this heuristics
        std::vector<int> finger_locations_1 =
            this->get_finger_locations(pre_state.finger_index);
        std::vector<int> finger_locations_2 =
            this->get_finger_locations(action_idx);

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

    int get_number_of_robot_actions(const State2 &state)
    {
        return this->n_finger_combinations;
    }

    int get_number_of_actions(const State2 &state)
    {
        return this->n_finger_combinations * this->saved_object_trajectory.size();
    }

    bool is_terminal(const State2 &state)
    {
        // TODO: change this

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

    // bool is_valid(const State2 &state, const State2 &prev_state);

    bool is_finger_valid(int finger_idx, int timestep);

    bool is_valid_transition(const State2 &state, const State2 &prev_state);
    
    void save_trajectory(const std::vector<State> &path);

    std::vector<State>
    generate_a_finer_object_trajectory(std::vector<State> &object_traj,
                                       double dist);

    bool robot_contact_feasibile_check(int finger_idx, const Vector7d &x, const VectorXi &cs_mode, const Vector6d &v,
                                       const std::vector<ContactPoint> &envs);

    int pruning_check(const Vector7d &x, const Vector6d &v,
                      const std::vector<ContactPoint> &envs);
    int pruning_check(const Vector7d &x, const VectorXi &cs_mode, const Vector6d &v,
                      const std::vector<ContactPoint> &envs);

    int max_forward_timestep(const State2 &state);
    int select_finger_change_timestep(const State2 &state);

    int encode_action_idx(int finger_idx, int timestep)
    {
        return timestep * this->n_finger_combinations + finger_idx;
    }

    void do_action(State2 &state, int action)
    {
        state.timestep = action / this->n_finger_combinations;
        state.finger_index = action % this->n_finger_combinations;
    }

    std::vector<State> saved_object_trajectory;
    std::vector<ContactPoint> object_surface_pts;
    int number_of_robot_contacts;

    std::shared_ptr<WorldTemplate>
        m_world; // save the object, environment, do collision detections, ...
    int n_finger_combinations = -1;

    std::shared_ptr<ReusableRRT> shared_rrt;

private:
    bool m_initialized = false;
    
    Vector7d start_object_pose;
    Vector7d goal_object_pose;

    int start_finger_idx = -1;
    int goal_finger_idx = -1;

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
};

bool force_closure(Vector7d x, const std::vector<ContactPoint> &mnps, double friction_coeff);
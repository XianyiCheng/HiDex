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

#ifndef _DART_WORLD
#define _DART_WORLD
#include "../mechanics/worlds/DartWorld.h"
#endif

#ifndef RRT_H
#define RRT_H
#include "rrt.h"
#endif

#ifndef Reward_H
#define Reward_H
#include "rewards.h"
#endif

#include "../mechanics/manipulators/DartWholeHand.h"

int find_part_idx(const std::vector<std::string> &x, const std::string &y);

class WholeHandTASK
{
public:
    // same as TASK::State
    struct State
    {
        typedef int Action;
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

        static bool is_no_action(Action action) { return action == -1; }
        static Action no_action() { return -1; }
    };

    struct State2
    {
        struct Action
        {
            int timestep = -1;

            // for finger relocation
            std::vector<std::string> hand_segments; // which hand segment to move
            std::vector<int> contact_idxes;         // move to which contact point on the object surface
            std::vector<std::string> motion_types;  //"relocate", "roll", "slide", "new", "release"

            Action() {}
            Action(int timestep_, std::vector<std::string> hand_segments_, std::vector<int> contact_idxes_, std::vector<std::string> motion_types_)
                : timestep(timestep_), hand_segments(hand_segments_), contact_idxes(contact_idxes_), motion_types(motion_types_) {}

            Action &operator=(const Action &action_)
            {
                this->timestep = action_.timestep;
                this->hand_segments = action_.hand_segments;
                this->contact_idxes = action_.contact_idxes;
                this->motion_types = action_.motion_types;
                return *this;
            }
        };

        static bool is_no_action(Action action)
        {
            return action.timestep == -1;
        }

        static Action no_action()
        {
            return Action();
        }

        int timestep = 0;
        // The state need to be initialized with all possible hand segments
        // no contact == contact idx -1
        std::vector<std::string> hand_segments; // which hand segment
        std::vector<int> contact_idxes;         // touches which contact point on the object surface
        bool is_valid;
        int t_max = -1; // maximum time step this can reach
        State2() {}
        State2(int t) : timestep(t) {}
        State2(int t, std::vector<std::string> hand_segments_, std::vector<int> contact_idxes_)
            : timestep(t), hand_segments(hand_segments_), contact_idxes(contact_idxes_) {}
        void do_action(Action action)
        {
            this->timestep = action.timestep;

            for (int i = 0; i < action.hand_segments.size(); ++i)
            {
                if (action.motion_types[i] == "release")
                {
                    int k = find_part_idx(this->hand_segments, action.hand_segments[i]);
                    this->hand_segments.erase(this->hand_segments.begin() + k);
                    this->contact_idxes.erase(this->contact_idxes.begin() + k);
                }
                else if (action.motion_types[i] == "new")
                {
                    this->hand_segments.push_back(action.hand_segments[i]);
                    this->contact_idxes.push_back(action.contact_idxes[i]);
                }
                else
                {
                    int k = find_part_idx(this->hand_segments, action.hand_segments[i]);
                    this->contact_idxes[k] = action.contact_idxes[i];
                }
            }
        }
    };

    struct ContactConfig
    {
        std::vector<std::string> hand_segments; // which hand segment
        std::vector<int> contact_idxes;         // touches which contact point on the object surface
        ContactConfig() {}
        ContactConfig(std::vector<std::string> hand_segments_, std::vector<int> contact_idxes_)
            : hand_segments(hand_segments_), contact_idxes(contact_idxes_) {}
        ContactConfig(State2 state) : ContactConfig(state.hand_segments, state.contact_idxes) {}
        ContactConfig(const ContactConfig &contact_config_)
        {
            this->hand_segments = contact_config_.hand_segments;
            this->contact_idxes = contact_config_.contact_idxes;
        }

        void apply_action(State2::Action action)
        {
            for (int i = 0; i < action.hand_segments.size(); ++i)
            {
                if (action.motion_types[i] == "release")
                {
                    int k = find_part_idx(this->hand_segments, action.hand_segments[i]);
                    this->hand_segments.erase(this->hand_segments.begin() + k);
                    this->contact_idxes.erase(this->contact_idxes.begin() + k);
                }
                else if (action.motion_types[i] == "new")
                {
                    this->hand_segments.push_back(action.hand_segments[i]);
                    this->contact_idxes.push_back(action.contact_idxes[i]);
                }
                else
                {
                    int k = find_part_idx(this->hand_segments, action.hand_segments[i]);
                    this->contact_idxes[k] = action.contact_idxes[i];
                }
            }
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

    WholeHandTASK() {}

    // ----------------------- required functions -----------------------

    void set_start_and_goal(const Vector7d &start_object_pose, const Vector7d &goal_object_pose);

    // TODO
    void set_task_parameters(double goal_thr, double wa,
                             double wt, double charac_len, double mu_env, double mu_mnp,
                             Matrix6d object_inertia, Vector6d f_gravity,
                             std::shared_ptr<DartWorld> world, int n_robot_contacts,
                             std::string dynamic_type, std::vector<ContactPoint> surface_pts,
                             const SearchOptions &options, bool if_refine = false,
                             double refine_dist = 0.0);

    void set_reward_functions(std::shared_ptr<RewardFunction> reward_L1,
                              std::shared_ptr<RewardFunction> reward_L2);

    void set_robot(std::shared_ptr<DartWholeHandManipulator> robot);

    void initialize();

    State get_start_state() const;

    State2 get_start_state2() const;

    std::vector<State> search_a_new_path(const State &start_state);

    double evaluate_path_level_1(const std::vector<State> &object_path,
                                 const std::vector<State2> &robot_contact_path);

    double evaluate_path_level_2(const std::vector<State> &object_path,
                                 const std::vector<State2> &robot_contact_path);

    void sample_level2_action(const State2 &state, State2::Action &action);

    int max_forward_timestep(const State2 &state);

    bool is_terminal(const State2 &state);

    void save_trajectory(const std::vector<State> &path);

    std::vector<State> get_saved_object_trajectory();

    int get_saved_object_trajectory_size();

    void clear_saved_object_trajectory();

    std::vector<double>
    get_path_features(const std::vector<State> &object_path,
                      const std::vector<State2> &robot_contact_path,
                      const std::vector<std::string> &feature_names)
    {
        // TODO: to be implemented
        return std::vector<double>();
    }

    // ----------------------- helper functions -----------------------
    int neighbors_on_the_same_manifold(const Vector7d &q,
                                       std::vector<ContactPoint> envs,
                                       std::vector<VectorXi> env_modes,
                                       double dist_thr);

    bool forward_integration(const Vector7d &x_start,
                             const Vector7d &x_goal,
                             const std::vector<ContactPoint> &envs_,
                             const VectorXi &env_mode_,
                             std::vector<Vector7d> *path);

    // roughly check ik for a contact config
    bool rough_ik_check(const ContactConfig &contact_config, const Vector7d &object_pose)
    {
        // TODO: to be implemented
        return true;
    }

    // Rough collision check for fingertips as spheres only
    bool rough_collision_check(const ContactConfig &contact_config, const Vector7d &object_pose)
    {
        // TODO: to be implemented
        return true;
    }

    // check if there is a good contact config exist for the given motion, save in contact_config
    bool pruning_check(const Vector7d &x, const VectorXi &cs_mode, const Vector6d &v,
                       const std::vector<ContactPoint> &envs, ContactConfig &contact_config, int max_sample = 100);

    // check if there is a good contact config exist for the given motion, transitioning from the previous motion, save in contact_config
    bool pruning_check_w_transition(const Vector7d &x, const Vector7d &x_pre,
                                    const VectorXi &cs_mode, const Vector6d &v,
                                    const std::vector<ContactPoint> &envs,
                                    const std::vector<ContactPoint> &envs_pre,
                                    ContactConfig &contact_config,
                                    int max_sample = 100);

    bool contact_force_feasible_check(std::vector<ContactPoint> object_contact_points, const Vector7d &x, const VectorXi &cs_mode,
                                      const Vector6d &v, const std::vector<ContactPoint> &envs);

    // check if the contact config is valid for the given motion
    bool robot_contact_feasible_check(
        const ContactConfig &contact_config, const Vector7d &x, const VectorXi &cs_mode,
        const Vector6d &v, const std::vector<ContactPoint> &envs);

    // check if the contact config is valid for the given timestep's motion, and if is kinematically and collision good to the next timestep
    bool is_contact_config_valid(const ContactConfig &contact_config,
                                 int timestep);

    bool is_valid_transition(const ContactConfig &previous_contact_config, const Vector7d &x, const std::vector<ContactPoint> &envs, const State2::Action &action)
    {
        // TODO: to be implemented
        return false;
    }

    bool sample_a_feasible_action(const Vector7d &object_pose, const ContactConfig &contact_config, const Vector7d &next_object_pose, State2::Action &action, int max_sample = 100)
    {
        // TODO: to be implemented
        return false;
    }

    // TODO: improve this function
    // sample contact configs that satisfy the rough ik and collision check
    void sample_likely_contact_configs(
        const Vector7d &object_pose, const VectorXi &cs_mode, const Vector6d &v,
        const std::vector<ContactPoint> &envs, int max_sample, std::vector<ContactConfig> *sampled_actions, std::vector<double> *probs);

    // Sample actions that satisfy the rough ik and collision check and transition condition
    void sample_likely_feasible_actions(
        const State2 &state, int max_sample, std::vector<State2::Action> *sampled_actions, std::vector<double> *probs)
    {
        // TODO: to be implemented
    }

    // properties
    double grasp_measure_charac_length = -1.0;

    std::vector<State> saved_object_trajectory;
    std::vector<ContactPoint> object_surface_pts;
    int number_of_robot_contacts;

    std::string task_dynamics_type =
        "quasistatic"; // "quasistatic", "quasidynamic", "none", "force_closure"

    std::shared_ptr<DartWorld>
        m_world; // save the object, environment, do collision detections, ...
    // unsigned long int n_finger_combinations = 0;

    std::shared_ptr<ReusableRRT> shared_rrt;

    Vector7d start_object_pose;
    Vector7d goal_object_pose;

    bool if_transition_pruning = false;

    // bool if_goal_finger = false;

    // long int start_finger_idx = -1;
    // long int goal_finger_idx = -1;

    std::shared_ptr<RewardFunction> reward_L1;
    std::shared_ptr<RewardFunction> reward_L2;

    std::string action_prob_L2 = "env";

private:
    bool m_initialized = false;
    bool m_paramter_set = false;
    bool m_start_and_goal_set = false;
    bool m_reward_set = false;
    bool m_robot_set = false;

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

    std::shared_ptr<DartWholeHandManipulator> robot;
};
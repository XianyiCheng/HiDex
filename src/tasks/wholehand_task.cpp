#include "wholehand_task.h"
#include "../mechanics/dart_utils/dart_utils.h"

Vector6d weight_w2o(const Vector7d &x, const Vector6d &f_ext_w)
{
    Matrix4d T = pose2SE3(x);
    Matrix6d Adg = SE32Adj(T);

    Matrix4d T_;
    T_.setIdentity();
    T_.block(0, 0, 3, 3) = T.block(0, 0, 3, 3);
    Vector6d f_ext_o = SE32Adj(T_).transpose() * f_ext_w;
    return f_ext_o;
}

bool is_too_close(const std::vector<ContactPoint> &pts, double d_thr)
{
    for (int i = 0; i < pts.size(); i++)
    {
        for (int j = i + 1; j < pts.size(); j++)
        {
            if ((pts[i].p - pts[j].p).norm() < d_thr)
            {
                return true;
            }
        }
    }
    return false;
}
// ----------------------- required functions -----------------------

void WholeHandTASK::set_start_and_goal(const Vector7d &start_object_pose, const Vector7d &goal_object_pose)
{
    this->start_object_pose = start_object_pose;
    this->goal_object_pose = goal_object_pose;
    this->m_start_and_goal_set = true;
}

void WholeHandTASK::initialize()
{
    this->cons = std::make_unique<ContactConstraints>(2);

    if (!this->m_paramter_set)
    {
        std::cout << "Please set the task parameters before initialize the task." << std::endl;
        exit(0);
    }
    if (!this->m_start_and_goal_set)
    {
        std::cout << "Please set the start and goal before initialize the task." << std::endl;
        exit(0);
    }
    if (!this->m_reward_set)
    {
        std::cout << "Please set the reward function before initialize the task." << std::endl;
        exit(0);
    }
    if (!this->m_robot_set)
    {
        std::cout << "Please set the robot before initialize the task." << std::endl;
        exit(0);
    }

    // initialize the shared RRT
    shared_rrt = std::make_shared<ReusableRRT>(this->wa, this->charac_len * this->wt);

    ReusableRRT::Node start_node(start_object_pose);
    this->m_world->getObjectContacts(&(start_node.envs), start_node.config);
    cs_mode_enumeration(*this->cons.get(), start_node.envs, &start_node.modes);
    shared_rrt->initial_node(&start_node);

    this->m_initialized = true;
}

void WholeHandTASK::set_reward_functions(std::shared_ptr<RewardFunction> reward_L1,
                                         std::shared_ptr<RewardFunction> reward_L2)
{
    this->reward_L1 = reward_L1;
    this->reward_L2 = reward_L2;
    this->m_reward_set = true;
}

void WholeHandTASK::set_robot(std::shared_ptr<DartWholeHandManipulator> robot)
{
    this->robot = robot;
    this->m_world->addRobot(robot.get());
    this->m_robot_set = true;
}

State WholeHandTASK::get_start_state() const
{
    State state_;
    state_.m_pose = this->start_object_pose;
    this->m_world->getObjectContacts(&state_.envs, state_.m_pose);
    cs_mode_enumeration(*this->cons.get(), state_.envs, &state_.modes);
    return state_;
}

State2 WholeHandTASK::get_start_state2() const
{
    if (this->m_robot_set == false)
    {
        std::cout << "Robot not set" << std::endl;
        exit(0);
    }
    State2 state_(0, this->robot->get_hand_segments(), std::vector<int>(this->robot->get_hand_segments().size(), -1));
    return state_;
}

std::vector<State> WholeHandTASK::search_a_new_path(const State &start_state)
{
    // search a new path towards the end, given the START_STATE and M_MODE_IDX!!!

    // during the search, it figure out the constriants (modes) for the states
    // for now just return linear interpolation towards the sampled state

    // every state need to be associated with m_mode_idx (mode to this state)

    std::vector<State> path_;

    std::cout << "Search a new path with shared rrt" << std::endl;

    int root_node_idx = shared_rrt->find_node(start_state.m_pose);

    // In this case we are allowed to expand the root_node_under our specified
    // mode (after it has been expanded towards the goal for all modes in the
    // first time)

    if (root_node_idx == -1)
    {
        std::cout << "The start state you requested is not in the shared_rrt tree. "
                     "There is a bug in your code."
                  << std::endl;
        exit(-1);
    }

    // allow rrt to extend the root node (but not towards goal)
    shared_rrt->nodes[root_node_idx].is_explored = false;

    std::vector<int> subtree = shared_rrt->subtree_node_idxes(
        root_node_idx, start_state.modes[start_state.m_mode_idx]);

    set_rand_seed();

    int goal_idx = -1;

    // -------
    // bool if_extend_root_to_goal = false;
    // if (!shared_rrt->nodes[root_node_idx].has_been_root) {
    //   if_extend_root_to_goal = true;
    //   shared_rrt->nodes[root_node_idx].has_been_root = true;
    //   shared_rrt->nodes[root_node_idx].is_extended_to_goal = true;
    // }
    bool if_extend_root_to_goal = true;

    for (int kk = 0; kk < this->search_options.max_samples; kk++)
    {

        std::cout << "rrt iter: " << kk << std::endl;

        // bias sample toward the goal
        Vector7d x_rand;
        int near_idx;

        if (randd() > this->search_options.goal_biased_prob)
        {
            if (randd() > 0.5)
            {
                Vector3d p_rand;
                Quaterniond q_rand;
                p_rand = sample_position(this->search_options.x_ub,
                                         this->search_options.x_lb);

                q_rand =
                    (this->search_options.sampleSO3)
                        ? generate_unit_quaternion()
                        : sample_rotation(this->search_options.sample_rotation_axis);

                x_rand << p_rand[0], p_rand[1], p_rand[2], q_rand.x(), q_rand.y(),
                    q_rand.z(), q_rand.w();
                // near_idx = shared_rrt->nearest_neighbor(x_rand);
                near_idx = shared_rrt->nearest_neighbor_subtree(x_rand, root_node_idx,
                                                                subtree, false, true);
                std::cout << "sampled random state" << std::endl;
            }
            else
            {
                bool if_sampled = false;
                near_idx = shared_rrt->nearest_neighbor_subtree(x_rand, root_node_idx,
                                                                subtree, true, true);
                double near_dist = shared_rrt->dist(shared_rrt->nodes[near_idx].config,
                                                    this->goal_object_pose);
                for (int sample_idx_i = 0; sample_idx_i < 50; sample_idx_i++)
                {
                    near_idx = randi(this->shared_rrt->nodes.size());
                    if ((!this->shared_rrt->nodes[near_idx].is_explored) &&
                        (!this->shared_rrt->nodes[near_idx].is_extended_to_goal) &&
                        (this->shared_rrt->dist(shared_rrt->nodes[near_idx].config,
                                                this->goal_object_pose) >
                         near_dist + 0.1))
                    {
                        if_sampled = true;
                        x_rand = this->goal_object_pose;
                        break;
                    }
                }
                if (!if_sampled)
                {
                    continue;
                }
            }
            std::cout << "sampled idx to extend to goal" << std::endl;
        }
        else
        {
            x_rand = this->goal_object_pose;
            if (if_extend_root_to_goal)
            {
                near_idx = root_node_idx;
                if_extend_root_to_goal = false;
                shared_rrt->nodes[near_idx].is_extended_to_goal = true;
            }
            else
            {
                near_idx = shared_rrt->nearest_neighbor_subtree(x_rand, root_node_idx,
                                                                subtree, true, true);
                shared_rrt->nodes[near_idx].is_extended_to_goal = true;
                std::cout << "sampled goal state" << std::endl;
            }
            if (near_idx < 0)
            {
                // all the nodes has extended to goal, try random sample again
                continue;
            }
        }

        if (near_idx < 0)
        {
            std::cout << "There is no unexplored nodes in this subtree. Cannot find "
                         "a new path. "
                      << std::endl;
            return path_;
        }

        // -----------------------------------

        // extend all cs modes

        // steer goal
        x_rand = steer_config(shared_rrt->nodes[near_idx].config, x_rand,
                              this->search_options.eps_trans,
                              this->search_options.eps_angle);

        std::cout << "x_rand " << x_rand.transpose() << std::endl;

        Vector6d v_star =
            compute_rbvel_body(shared_rrt->nodes[near_idx].config, x_rand);

        Vector6d f_o =
            weight_w2o(shared_rrt->nodes[near_idx].config, this->f_gravity);

        // contact mode enumeration
        if (shared_rrt->nodes[near_idx].modes.size() == 0)
        {
            if (shared_rrt->nodes[near_idx].envs.size() > 0)
            {
                shared_rrt->nodes[near_idx].envs.clear();
            }
            this->m_world->getObjectContacts(&(shared_rrt->nodes[near_idx].envs),
                                             shared_rrt->nodes[near_idx].config);
            cs_mode_enumeration(*this->cons.get(), shared_rrt->nodes[near_idx].envs,
                                &shared_rrt->nodes[near_idx].modes);
        }

        // for every mode do forward integration
        Vector6d v_zero = Vector6d::Zero();
        // move

        std::vector<VectorXi> extendable_cs_modes;
        if ((near_idx == root_node_idx) && (!if_extend_root_to_goal))
        {
            extendable_cs_modes.push_back(start_state.modes[start_state.m_mode_idx]);
        }
        else
        {
            extendable_cs_modes = shared_rrt->nodes[near_idx].modes;
        }

        for (const auto &cs_mode : extendable_cs_modes)
        {

            std::cout << "cs mode " << cs_mode.transpose() << std::endl;

            std::vector<VectorXi> check_modes;
            {
                VectorXi all_sticking_mode(3 * cs_mode.size());
                all_sticking_mode.setZero();
                all_sticking_mode.block(0, 0, cs_mode.size(), 1) = cs_mode;
                check_modes.push_back(all_sticking_mode);
                check_modes.push_back(cs_mode);
            }

            std::vector<VectorXi> mode_to_extend;
            for (const auto &check_mode : check_modes)
            {
                Vector6d v;
                if (check_mode.size() == shared_rrt->nodes[near_idx].envs.size())
                {
                    v = EnvironmentConstrainedVelocity_CSModeOnly(
                        v_star, shared_rrt->nodes[near_idx].envs, check_mode,
                        *this->cons);
                }
                else
                {
                    v = EnvironmentConstrainedVelocity(v_star,
                                                       shared_rrt->nodes[near_idx].envs,
                                                       check_mode, *this->cons);
                }

                if (v.norm() > 1e-6)
                {
                    bool is_pass_pruning;
                    ContactConfig contact_config;
                    if ((shared_rrt->nodes[near_idx].parent < 0) ||
                        !this->if_transition_pruning)
                    {
                        is_pass_pruning = this->pruning_check(
                            shared_rrt->nodes[near_idx].config, cs_mode,
                            v, shared_rrt->nodes[near_idx].envs, contact_config);
                    }
                    else
                    {

                        is_pass_pruning =
                            this->pruning_check_w_transition(
                                shared_rrt->nodes[near_idx].config,
                                shared_rrt->nodes[shared_rrt->nodes[near_idx].parent].config,
                                cs_mode, v, shared_rrt->nodes[near_idx].envs,
                                shared_rrt->nodes[shared_rrt->nodes[near_idx].parent].envs,
                                contact_config);
                    }
                    if (is_pass_pruning)
                    {
                        mode_to_extend.push_back(check_mode);
                    }
                }
            }
            /// choose sliding mode end

            for (const auto &mode : mode_to_extend)
            {

                std::cout << "Extend mode: " << mode.transpose() << std::endl;

                std::vector<Vector7d> path;

                // move
                this->forward_integration(shared_rrt->nodes[near_idx].config, x_rand,
                                          shared_rrt->nodes[near_idx].envs, mode,
                                          &path);

                // if integration is successful
                if (path.size() > 2)
                {
                    ReusableRRT::Node new_node(path.back());
                    this->m_world->getObjectContacts(&(new_node.envs), new_node.config);

                    if (shared_rrt->find_node(new_node.config, new_node.envs.size(),
                                              near_idx, 1e-2) != -1)
                    {
                        // printf("This node is already in the tree!\n");
                        continue;
                    }

                    if (this->search_options.control_neighbors)
                    {

                        // skip this node if it does not make progress (example: object
                        // scale: ~1m, moved < 2cm) and the contacts stays the same
                        if ((shared_rrt->dist(shared_rrt->nodes[near_idx].config,
                                              new_node.config) < 0.02))
                        {
                            VectorXi contact_remain =
                                track_contacts_remain(this->shared_rrt->nodes[near_idx].envs,
                                                      new_node.envs, 0.8, 0.02);
                            if (contact_remain.size() ==
                                this->shared_rrt->nodes[near_idx].modes[0].size())
                            {
                                continue;
                            }
                        }

                        // skip this node if there are enough neighbors on the same manifold

                        cs_mode_enumeration(*this->cons.get(), new_node.envs,
                                            &new_node.modes);

                        int num_neighbors = this->neighbors_on_the_same_manifold(
                            new_node.config, new_node.envs, new_node.modes,
                            this->search_options.eps_trans / 2.0);

                        if (num_neighbors > 3)
                        {
                            // std::cout << "Skip node " << new_node.config.transpose()
                            //           << " because it has " << num_neighbors
                            //           << " neighbors on the same manifold" << std::endl;
                            continue;
                        }
                    }

                    std::cout << "New node idx: " << this->shared_rrt->nodes.size()
                              << ", Parent: " << near_idx << ", "
                              << path.back().transpose() << std::endl;

                    ReusableRRT::Edge new_edge(mode, path);

                    shared_rrt->add_node(&new_node, near_idx, &new_edge);

                    if (near_idx == root_node_idx)
                    {
                        // for the nodes expaned from the root node, we need to check if
                        // the mode is the desired one
                        if ((cs_mode - start_state.modes[start_state.m_mode_idx]).norm() ==
                            0)
                        {
                            subtree.push_back(shared_rrt->nodes.size() - 1);
                        }
                    }
                    else
                    {
                        subtree.push_back(shared_rrt->nodes.size() - 1);
                    }
                }
            }
        }

        //----------------------------------

        int goal_near_idx = shared_rrt->nearest_neighbor_subtree(
            this->goal_object_pose, root_node_idx, subtree, false, true);
        if (shared_rrt->dist(shared_rrt->nodes[goal_near_idx].config,
                             this->goal_object_pose) <= goal_thr)
        {
            printf("Found goal node in %d samples. \n", kk + 1);
            goal_idx = goal_near_idx;
            break;
        }
    }

    bool ifsuccess = false;

    if (goal_idx != -1)
    {
        ifsuccess = true;
        printf("GOAL REACHED! \n");
    }
    else
    {
        ifsuccess = false;
        std::cout << "GOAL NOT REACHED" << std::endl;
    }

    /// end of search

    if (ifsuccess)
    {
        // backtrack the node path until the root_node_idx, root_node_idx is
        // included
        std::vector<int> node_path;
        shared_rrt->backtrack(goal_idx, &node_path, root_node_idx);
        std::reverse(node_path.begin(), node_path.end());
        for (int kn : node_path)
        {
            shared_rrt->nodes[kn].is_explored = true;
        }
        for (int k = 1; k < node_path.size() - 1; k++)
        {
            int kn = node_path[k];
            int k_child = node_path[k + 1];
            shared_rrt->nodes[k_child].is_explored = true;
            VectorXi mode = shared_rrt->edges[shared_rrt->nodes[k_child].edge].mode;
            int mode_idx = -1;
            for (int idx = 0; idx < shared_rrt->nodes[kn].modes.size(); ++idx)
            {
                if ((mode.head(shared_rrt->nodes[kn].modes[idx].size()) -
                     shared_rrt->nodes[kn].modes[idx])
                        .norm() == 0)
                {
                    mode_idx = idx;
                    break;
                }
            }
            State new_state(shared_rrt->nodes[kn].config,
                            shared_rrt->nodes[kn].envs, mode_idx,
                            shared_rrt->nodes[kn].modes);
            new_state.m_path = shared_rrt->edges[shared_rrt->nodes[kn].edge].path;

            path_.push_back(new_state);
        }
        State new_state(shared_rrt->nodes[node_path.back()].config,
                        shared_rrt->nodes[node_path.back()].envs, -1,
                        shared_rrt->nodes[node_path.back()].modes);
        new_state.m_path =
            shared_rrt->edges[shared_rrt->nodes[node_path.back()].edge].path;

        path_.push_back(new_state);
    }

    for (auto s : path_)
    {
        std::cout << "Pose " << s.m_pose.transpose() << std::endl;
        if (s.m_mode_idx != -1)
        {
            std::cout << "Mode " << s.modes[s.m_mode_idx].transpose() << std::endl;
        }
    }

    return path_;
}

bool WholeHandTASK::forward_integration(const Vector7d &x_start,
                                        const Vector7d &x_goal,
                                        const std::vector<ContactPoint> &envs_,
                                        const VectorXi &env_mode_,
                                        std::vector<Vector7d> *path)
{

    // The env_mode_ can either be the full mode (cs + ss) or cs mode

    double thr = 1e-4;

    double h = 0.04;

    int max_counter = 150;

    Vector7d x = x_start;
    VectorXi env_mode = env_mode_;

    path->push_back(x);

    std::vector<ContactPoint> envs;
    envs = envs_;

    std::vector<ContactPoint> envs_pre;
    envs_pre = envs;

    int mode_type =
        (env_mode.size() == envs.size()) ? MODE_TYPE_CS : MODE_TYPE_FULL;

    Vector6d v_b_pre;
    v_b_pre.setZero();

    int counter;
    int delete_c = 0;

    ContactConfig selected_contact_config;

    for (counter = 0; counter < max_counter; counter++)
    {
        Vector6d v_star = compute_rbvel_body(x, x_goal);

        if (v_star.norm() < thr)
        {
            // std::cout << "v_star < thr : " << v_star.transpose() << std::endl;
            break;
        }

        Matrix4d T = pose2SE3(x);
        Matrix6d Adg = SE32Adj(T);

        Matrix4d T_;
        T_.setIdentity();
        T_.block(0, 0, 3, 3) = T.block(0, 0, 3, 3);

        Vector6d v_b;
        if (mode_type == MODE_TYPE_FULL)
        {
            v_b = EnvironmentConstrainedVelocity(v_star, envs, env_mode, *this->cons);
        }
        else
        {
            // mode_type == MODE_TYPE_CS
            v_b = EnvironmentConstrainedVelocity_CSModeOnly(v_star, envs, env_mode,
                                                            *this->cons);
        }

        if (v_b.norm() < thr)
        {
            // std::cout << "v_b < thr : " << v_b.transpose() << std::endl;
            break;
        }

        if ((v_b_pre.transpose() * v_b)[0] < -1e-5)
        {
            // printf("v_b back and forth. \n");
            break;
        }

        if (counter == 0)
        {
            bool if_select_contact_config =
                this->pruning_check(x, env_mode.head(envs.size()), v_b, envs, selected_contact_config);
            if (if_select_contact_config == -1)
            {
                std::cout << "Cannot find contact config in forward_integration" << std::endl;
                break;
            }
        }
        else
        {
            bool pass_pruning_check = this->robot_contact_feasible_check(
                selected_contact_config, x, env_mode.head(envs.size()), v_b, envs);
            if (!pass_pruning_check)
            {
                break;
            }
        }

        steer_velocity(v_b, h, this->charac_len);

        // integrate v
        Vector7d x_new = SE32pose(T * se32SE3(v_b));

        this->m_world->updateObjectPose(x_new);

        // check penetration & interpolation (break may happen)
        envs.clear();
        this->m_world->getObjectContacts(&envs, x_new);

        // velocity correction
        int pre_env_size =
            (mode_type == MODE_TYPE_FULL) ? env_mode.size() / 3 : env_mode.size();

        if (envs.size() != 0 && envs.size() == pre_env_size &&
            (ifNeedVelocityCorrection(env_mode, envs)))
        {
            // std::cout << "velocity correction " << counter << std::endl;

            int iter_corr = 0;
            while (iter_corr < 10)
            {
                if (envs.size() == pre_env_size)
                {
                    if (!ifNeedVelocityCorrection(env_mode, envs))
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
                Vector6d v_corr = VelocityCorrection(envs);
                x_new = SE32pose(pose2SE3(x_new) * se32SE3(v_corr));
                envs.clear();
                this->m_world->getObjectContacts(&envs, x_new);
                iter_corr++;
            }

            if (is_penetrate(envs) || (envs.size() != pre_env_size))
            {
                break;
            }
        }

        if (envs.size() > pre_env_size)
        {
            // Detects new contacts: project the object back to zero contact distance
            int iter_corr = 0;
            while (iter_corr < 10)
            {
                VectorXi mode_corr(envs.size());
                mode_corr.setZero();
                if (!ifNeedVelocityCorrection(mode_corr, envs))
                {
                    path->push_back(x_new);
                    break;
                }
                Vector6d v_corr = VelocityCorrection(envs);
                x_new = SE32pose(pose2SE3(x_new) * se32SE3(v_corr));
                envs.clear();
                this->m_world->getObjectContacts(&envs, x_new);
                iter_corr++;
            }
            // printf("Made new contacts! \n");
            // print_contacts(envs);
            break;
        }

        // update contact mode if needed (less contact detected)
        if (envs.size() < pre_env_size)
        {
            VectorXi remain_idx = track_contacts_remain(envs_pre, envs);
            if (envs.size() != 0 && remain_idx.size() == 0)
            {
                // printf("contact track fails.\n");
                break;
            }
            if (ifContactingModeDeleted(env_mode, remain_idx, envs_pre.size()))
            {
                if (h < 0.004 / 5)
                {

                    // delete_c++;
                    // if (delete_c > 4)
                    //   break;
                    // printf("Collision Detection delelte contacting mode \n");
                    Vector6d v_corr =
                        recoverContactingContacts(envs_pre, env_mode, remain_idx);

                    x_new = SE32pose(pose2SE3(x_new) * se32SE3(v_corr));
                    envs.clear();
                    this->m_world->getObjectContacts(&envs, x_new);

                    remain_idx = track_contacts_remain(envs_pre, envs);

                    if (ifContactingModeDeleted(env_mode, remain_idx, envs_pre.size()))
                    {

                        break;
                    }
                    else
                    {
                        env_mode = deleteModebyRemainIndex(env_mode, remain_idx, mode_type);
                    }
                }
                else
                {
                    h = h / 1.5;
                    envs = envs_pre;
                    continue;
                }
            }
            else
            {
                env_mode = deleteModebyRemainIndex(env_mode, remain_idx, mode_type);
            }
        }

        if (is_penetrate(envs))
        {
            printf("penetrate! \n");
            break;
        }

        // if ((x_start[0] < 1.5) && (x_new[2] > 0.25)) {
        //   std::cout << "debug here" << std::endl;
        //   env_mode = env_mode_;
        //   envs = envs_pre;
        //   continue;
        // }

        x = x_new;
        envs_pre = envs;
        v_b_pre = v_b;

        path->push_back(x);

        if (counter == max_counter - 1)
        {
            // printf("Reach the end.\n");
        }
    }

    // std::cout << "counter:" << counter << " x: " << x.transpose() << std::endl;

    return true;
}

double WholeHandTASK::evaluate_path_level_1(const std::vector<State> &object_path,
                                            const std::vector<State2> &robot_contact_path)
{
    std::vector<double> features = this->get_path_features(
        object_path, robot_contact_path, this->reward_L1->get_feature_names());
    double reward = this->reward_L1->get(features);
    return reward;
}

double WholeHandTASK::evaluate_path_level_2(const std::vector<State> &object_path,
                                            const std::vector<State2> &robot_contact_path)
{
    std::vector<double> features = this->get_path_features(
        object_path, robot_contact_path, this->reward_L2->get_feature_names());
    double reward = this->reward_L1->get(features);
    return reward;
}

void WholeHandTASK::sample_level2_action(const State2 &state, State2::Action &action)
{
    int max_sample = 300;

    bool if_valid = false;
    State2::Action new_action;

    std::vector<State2::Action> sampled_actions;
    std::vector<double> probs;

    this->sample_likely_feasible_actions(state, max_sample, &sampled_actions, &probs);

    std::default_random_engine randgen;
    std::discrete_distribution<int> distribution{probs.begin(), probs.end()};

    for (int k_sample = 0; k_sample < sampled_actions.size(); k_sample++)
    {
        int ik_sample = distribution(randgen);
        new_action = sampled_actions[ik_sample];

        ContactConfig contact_config(state);
        contact_config.apply_action(new_action);

        // is valid transition & valid for at least one timestep
        bool is_contact_config_valid = this->is_contact_config_valid(contact_config, new_action.timestep);
        if (is_contact_config_valid)
        {
            bool is_transition_valid = this->is_valid_transition(contact_config, this->saved_object_trajectory[new_action.timestep].m_pose, this->saved_object_trajectory[new_action.timestep].envs, new_action.timestep);
            if (is_transition_valid)
            {
                if_valid = true;
                break;
            }
        }
    }

    if (if_valid)
    {
        action = new_action;
    }
    else
    {
        action = State2::no_action;
    }
    return;
}

int WholeHandTASK::max_forward_timestep(const State2 &state)
{
    int t_max;
    ContactConfig contact_config(state);

    for (t_max = state.timestep; t_max < this->saved_object_trajectory.size();
         ++t_max)
    {
        bool is_feasible = this->is_contact_config_valid(contact_config, t_max);
        if (!is_feasible)
        {
            break;
        }
        if (is_feasible && (t_max == (this->saved_object_trajectory.size() - 1)))
        {
            return t_max;
        }
    }

    return t_max - 1;
}

bool WholeHandTASK::is_terminal(const State2 &state)
{
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

void WholeHandTASK::save_trajectory(const std::vector<State> &path)
{
    if (this->saved_object_trajectory.size() > 0)
    {
        this->saved_object_trajectory.clear();
    }

    if (!this->if_refine)
    {
        this->saved_object_trajectory = path;
    }
    else
    {

        this->saved_object_trajectory.push_back(path[0]);

        for (int i = 1; i < path.size(); ++i)
        {
            State state = path[i];

            int pre_step = 0;
            for (int k = 0; k < state.m_path.size() - 1; k++)
            {
                double cur_dist = this->shared_rrt->dist(
                    this->saved_object_trajectory.back().m_pose, state.m_path[k]);
                if (cur_dist >= this->refine_dist)
                {
                    // add a new state
                    State new_state;
                    new_state.m_pose = state.m_path[k];
                    std::vector<Vector7d> new_path(state.m_path.begin() + pre_step,
                                                   state.m_path.begin() + k);
                    new_state.m_path = new_path;
                    pre_step = k;
                    this->saved_object_trajectory.push_back(new_state);
                }
            }

            this->m_world->getObjectContacts(&state.envs, state.m_pose);
            std::vector<Vector7d> new_path(state.m_path.begin() + pre_step,
                                           state.m_path.end());
            state.m_path = new_path;
            this->saved_object_trajectory.push_back(state);
        }
    }

    for (int i = 0; i < this->saved_object_trajectory.size(); ++i)
    {
        this->m_world->getObjectContacts(&(this->saved_object_trajectory[i].envs),
                                         this->saved_object_trajectory[i].m_pose);
    }
}

std::vector<State> WholeHandTASK::get_saved_object_trajectory()
{
    return this->saved_object_trajectory;
}

int WholeHandTASK::get_saved_object_trajectory_size()
{
    return this->saved_object_trajectory.size();
}

void WholeHandTASK::clear_saved_object_trajectory()
{
    this->saved_object_trajectory.clear();
}

// ------------------- helper functions -------------------

int WholeHandTASK::neighbors_on_the_same_manifold(const Vector7d &q,
                                                  std::vector<ContactPoint> envs,
                                                  std::vector<VectorXi> env_modes,
                                                  double dist_thr)
{
    int num = 0;
    for (int i = 0; i < this->shared_rrt->nodes.size(); i++)
    {
        if (this->shared_rrt->nodes[i].envs.size() != envs.size())
        {
            continue;
        }
        double d = this->shared_rrt->dist(this->shared_rrt->nodes[i].config, q);
        if (d > dist_thr)
        {
            continue;
        }
        num++;
    }
    return num;
}

bool WholeHandTASK::contact_force_feasible_check(std::vector<ContactPoint> object_contact_points, const Vector7d &x, const VectorXi &cs_mode,
                                                 const Vector6d &v, const std::vector<ContactPoint> &envs)
{
    std::vector<ContactPoint> mnps;
    this->m_world->getRobot()->Fingertips2PointContacts(object_contact_points, &mnps);

    if (this->task_dynamics_type == "quasistatic")
    {
        if (v.norm() < 1e-6)
        {
            env_mode = conservative_mode_from_velocity(envs, cs_mode, v, this->cons.get());
        }
        dynamic_feasibility =
            isQuasistatic(mnps, envs, env_mode, this->f_gravity, x, this->mu_env,
                          this->mu_mnp, this->cons.get());
    }
    else if (this->task_dynamics_type == "quasidynamic")
    {
        double h_time = 1.0;
        double thr = 0.5;

        dynamic_feasibility =
            isQuasidynamic(v, mnps, envs, env_mode, this->f_gravity,
                           this->object_inertia, x, this->mu_env, this->mu_mnp,
                           this->wa, this->wt, h_time, this->cons.get(), thr);
    }
    else
    {
        return true;
    }

    return dynamic_feasibility;
}

bool WholeHandTASK::robot_contact_feasible_check(
    const ContactConfig &contact_config, const Vector7d &x, const VectorXi &cs_mode,
    const Vector6d &v, const std::vector<ContactPoint> &envs)
{
    VectorXi env_mode = mode_from_velocity(v, envs, this->cons.get());
    env_mode.head(envs.size()) = cs_mode;

    bool if_config_possible = this->rough_ik_and_collision_check(contact_config, x);

    if (!if_config_possible)
    {
        return false;
    }

    bool dynamic_feasibility = false;

    std::vector<ContactPoint> fingertips;
    for (int k : contact_config.contact_idxes)
    {
        fingertips.push_back(this->object_surface_pts[k]);
    }
    dynamic_feasibility = this->contact_force_feasible_check(fingertips, x, env_mode, v, envs);

    return dynamic_feasibility;
}

bool WholeHandTASK::pruning_check(const Vector7d &x, const VectorXi &cs_mode, const Vector6d &v,
                                  const std::vector<ContactPoint> &envs, ContactConfig &contact_config, int max_sample)
{

    std::vector<ContactConfig> sampled_contact_configs;
    std::vector<double> probs;
    this->sample_likely_contact_configs(x, cs_mode, v, envs, max_sample, &sampled_contact_configs,
                                        &probs);

    if (sampled_contact_configs.size() == 0)
    {
        return false;
    }

    bool if_feasible = false;

    std::default_random_engine randgen;
    std::discrete_distribution<int> distribution{probs.begin(), probs.end()};

    for (int k_sample = 0; k_sample < sampled_contact_configs.size(); k_sample++)
    {
        int ik_sample = distribution(randgen);
        if_feasible = this->robot_contact_feasible_check(sampled_contact_configs[ik_sample], x, cs_mode, v, envs);
        if (if_feasible)
        {
            contact_config = sampled_contact_configs[ik_sample];
            return true;
        }
    }
    return false;
}

bool WholeHandTASK::pruning_check_w_transition(const Vector7d &x, const Vector7d &x_pre,
                                               const VectorXi &cs_mode, const Vector6d &v,
                                               const std::vector<ContactPoint> &envs,
                                               const std::vector<ContactPoint> &envs_pre,
                                               ContactConfig &contact_config,
                                               int max_sample)
{

    Vector6d v_pre = compute_rbvel_body(x_pre, x);
    VectorXi cs_mode_pre = mode_from_velocity(v_pre, envs_pre, this->cons.get())
                               .head(envs_pre.size());

    std::vector<ContactConfig> sampled_contact_configs;
    std::vector<double> probs;
    this->sample_likely_contact_configs(x_pre, cs_mode_pre, v_pre, envs_pre, max_sample, &sampled_contact_configs,
                                        &probs);

    if (sampled_contact_configs.size() == 0)
    {
        return false;
    }

    bool if_feasible = false;

    std::default_random_engine randgen;
    std::discrete_distribution<int> distribution{probs.begin(), probs.end()};

    for (int k_sample = 0; k_sample < sampled_contact_configs.size(); k_sample++)
    {
        int ik_sample = distribution(randgen);
        ContactConfig pre_contact_config;
        if_feasible = this->robot_contact_feasible_check(sampled_contact_configs[ik_sample], x, cs_mode, v, &envs);
        if (if_feasible)
        {
            pre_contact_config = sampled_contact_configs[ik_sample];
        }
        else
        {
            continue;
        }
        State2::Action action;
        bool is_action_sampled = sample_a_feasible_action(x, pre_contact_config, x_pre, action, max_sample);
        if (is_action_sampled)
        {
            pre_contact_config.apply_action(action);
            contact_config = pre_contact_config;
            return true;
        }
    }
    return false;
}

bool WholeHandTASK::is_contact_config_valid(const ContactConfig &contact_config,
                                            int timestep)
{
    Vector7d x_object = this->saved_object_trajectory[timestep].m_pose;
    Vector7d x_object_next;
    VectorXi reference_cs_mode(
        this->saved_object_trajectory[timestep].envs.size());
    reference_cs_mode.setOnes();
    // update to the x in the next step if timestep < total_timesteps - 1,
    // otherwise will check for zero velocity
    Vector6d v;

    if (timestep < this->saved_object_trajectory.size() - 1)
    {

        x_object_next = this->saved_object_trajectory[timestep + 1].m_pose;

        if ((this->saved_object_trajectory[timestep].m_mode_idx != -1) &&
            (this->saved_object_trajectory[timestep].m_mode_idx <
             this->saved_object_trajectory[timestep].modes.size()))
        {
            reference_cs_mode =
                this->saved_object_trajectory[timestep]
                    .modes[this->saved_object_trajectory[timestep].m_mode_idx];
            VectorXi estimate_cs_mode = cs_mode_from_contacts(
                this->saved_object_trajectory[timestep].envs,
                this->saved_object_trajectory[timestep + 1].envs);

            reference_cs_mode =
                conservative_cs_mode(reference_cs_mode, estimate_cs_mode);
        }

        if (this->saved_object_trajectory[timestep + 1].m_path.size() > 1)
        {
            v = compute_rbvel_body(
                x_object, this->saved_object_trajectory[timestep + 1].m_path[1]);
        }
    }
    else
    {
        x_object_next = x_object;
        v.setZero();
    }

    bool next_feasibility = this->rough_ik_and_collision_check(contact_config, x_object_next);

    if (!next_feasibility)
    {
        return false;
    }

    bool current_feasibility = this->robot_contact_feasible_check(contact_config, x_object, refernece_cs_mode, v, this->saved_object_trajectory[timestep].envs);
    if (!current_feasibility)
    {
        return false;
    }

    return true;
}

void WholeHandTASK::sample_likely_contact_configs(
    const Vector7d &object_pose, const VectorXi &cs_mode, const Vector6d &v,
    const std::vector<ContactPoint> &envs, int max_sample, std::vector<ContactConfig> *sampled_actions, std::vector<double> *probs)
{
    // TODO: test it
    
    // Pipeline
    // 1. Sample number of contact points
    // 2. Sample contact points that are feasible in force
    // 3. Sample corresponding hand segments
        // 3.1 Option 1: use robot->ifConsiderPartPairs 
        // 3.2 Option 2: Sample a hand config around the object pose, find nearest hand segments; check ifConsiderPartPairs

    int n_sample = 0;

    int max_part_sample = this->robot->allowed_part_names.size();

    while (n_sample < max_sample)
    {
        n_sample++;

        // 1. Sample number of contact points
        int n_contacts = randi(this->robot->maximum_simultaneous_contact);

        // 2. Sample contact points that are feasible in force
        std::vector<ContactPoint> sampled_fingertips;
        std::vector<idx> sampled_contact_idxes;
        for (int i = 0; i < n_contacts; i++)
        {
            int k = randi(this->object_surface_pts.size());
            sampled_contact_idxes.push_back(k);
            sampled_fingertips.push_back(this->object_surface_pts[k]);
        }
        // Points should not be too close from each other (> radius)
        bool is_close = this->is_too_close(sampled_fingertips, 2.5 * this->robot->getPatchContactRadius());
        if (is_close)
        {
            continue;
        }

        bool if_force_feasible = this->contact_force_feasible_check(sampled_fingertips, object_pose, cs_mode, v, envs);
        if (!if_force_feasible)
        {
            continue;
        }

        // 3. Sample corresponding hand segments

        int sample_option = 1;

        if (sample_option == 1) // 3.1 Option 1: use robot->ifConsiderPartPairs
        {

            std::vector<int> part_idxs;
            for (int i_contact = 0; i_contact < n_contact; i_contact++)
            {

                bool is_valid_part = false;

                int n_part_sample = 0;

                int part_idx;

                while ((!is_valid_part) && (n_part_sample < max_part_sample))
                {
                    n_part_sample++;

                    part_idx = randi(this->robot->allowed_part_names.size());

                    is_valid_part = true;

                    for (int j_contact = 0; j_contact < i_contact; j_contact++)
                    {
                        double d_check = (sampled_fingertips[i_contact].p - sampled_fingertips[j_contact].p).norm();

                        bool is_valid_pair = this->robot->ifConsiderPartPairs(part_idxs[i_contact], part_idxs[j_contact], d_check);

                        if (!is_valid_pair)
                        {
                            is_valid_part = false;
                            break;
                        }
                    }
                }
                if (is_valid_part)
                {
                    part_idxs.push_back(part_idx);
                }
                else
                {
                    break;
                }
            }
            if (part_idxs.size() != n_contacts)
            {
                continue;
            }
            else
            {
                std::vector<std::string> sampled_segments;
                for (int i = 0; i < part_idxs.size(); i++)
                {
                    sampled_segments.push_back(this->robot->allowed_part_names[part_idxs[i]]);
                }
                sampled_actions->push_back(ContactConfig(sampled_segments, sampled_contact_idxes));
                probs->push_back(1.0);
            }
        }
    }
    else // 3.2 Option 2: Sample a hand config around the object pose, find nearest hand segments; check ifConsiderPartPairs
    {
        VectorXd random_config = this->robot->sampleRandomConfig();
        random_config.head(3) = pose7d_to_pose6d(object_pose).head(3);
        // Get points on the hand
        std::vector<std::string> sampled_segments;

        std::vector<ContactPoint> part_points = this->robot->get_points_in_world(this->robot->allowed_part_names, this->robot->allowed_part_point_idxes, random_config);

        for (int i_contact = 0; i_contact < n_contact; i_contact++)
        {
            int closest_part;
            double closest_dist = 1e10;
            for (int i_part = 0; i_part < part_points.size(); i_part++)
            {
                double d_check = (sampled_fingertips[i_contact].p - part_points[i_part].p).norm();
                if (d_check < closest_dist)
                {
                    closest_dist = d_check;
                    closest_part = i_part;
                }
            }
            sampled_segments.push_back(this->robot->allowed_part_names[closest_part]);
        }

        sampled_actions->push_back(ContactConfig(sampled_segments, sampled_contact_idxes));
        probs->push_back(1.0);
    }
}
#include "task.h"
#include "../mechanics/contacts/contact_kinematics.h"
#include "../mechanics/contacts/contact_mode_enumeration.h"
#include "../mechanics/force_check.h"
#include "../mechanics/mode_utils.h"
#include "../mechanics/utilities/combinatorics.h"
#include "../mechanics/utilities/eiquadprog.hpp"

// ---------------------------
// Utility functions
#define MODE_TYPE_CS 0
#define MODE_TYPE_FULL 1
#include "integration_utils.h"

#define QUASIDYNAMIC_RANGE 0.3 // (-1,1) 1 exactly satisfy the accelerate direction, -1: whatever is ok

bool is_contact_free(std::vector<ContactPoint> envs, double free_thr)
{
  if (envs.size() == 0)
  {
    return true;
  }

  for (auto p : envs)
  {
    if (p.d > -free_thr)
    {
      return false;
    }
  }

  return true;
}

bool is_repeated_idxes(const std::vector<int> &vec)
{
  for (int i = 0; i < vec.size(); i++)
  {
    if (vec[i] == -1)
    {
      continue;
    }
    for (int j = i + 1; j < vec.size(); j++)
    {
      if (vec[i] == vec[j])
      {
        return true;
      }
    }
  }
  return false;
}

int number_of_different_elements(const std::vector<int> &v1,
                                 const std::vector<int> &v2)
{
  int count = 0;
  for (int i = 0; i < v1.size(); i++)
  {
    if (v1[i] != v2[i])
    {
      count++;
    }
  }
  return count;
}

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

int TASK::neighbors_on_the_same_manifold(const Vector7d &q,
                                         const std::vector<ContactPoint> &envs,
                                         const std::vector<VectorXi> &env_modes,
                                         double dist_thr)
{

  std::vector<long int> sampled_finger_idxes;
  Vector6d v;
  v.setZero();

  for (auto env_mode : env_modes)
  {
    long int idx = this->pruning_check(q, env_mode, v, envs);
    sampled_finger_idxes.push_back(idx);
  }

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

    bool if_feasible = true;
    for (int k = 0; k < env_modes.size(); ++k)
    {
      if (sampled_finger_idxes[k] == -1)
      {
        continue;
      }
      if_feasible = this->robot_contact_feasible_check(
          sampled_finger_idxes[k], this->shared_rrt->nodes[i].config,
          env_modes[k], v, this->shared_rrt->nodes[i].envs);
      if (!if_feasible)
      {
        break;
      }
    }

    if (if_feasible)
    {
      num++;
    }
  }

  return num;
}

bool TASK::forward_integration(const Vector7d &x_start, const Vector7d &x_goal,
                               const std::vector<ContactPoint> &envs_,
                               const VectorXi &env_mode_,
                               std::vector<Vector7d> *path, const VectorXd &manipulator_config)
{

  // The env_mode_ can either be the full mode (cs + ss) or cs mode

  double thr = 1e-4;

  double h = 0.04;

  int max_counter = 150;

  bool if_provide_mnp_config = manipulator_config.size() == 0 ? false : true;

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

  long int selected_finger_idx;

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

    // select a feasible finger index to execute the motion, if not feasible
    // anymore, break
    if (!if_provide_mnp_config)
    {
      if (counter == 0)
      {
        selected_finger_idx =
            this->pruning_check(x, env_mode.head(envs.size()), v_b, envs);
      }
      if (selected_finger_idx == -1)
      {
        break;
      }
      else
      {
        bool pass_pruning_check = this->robot_contact_feasible_check(
            selected_finger_idx, x, env_mode.head(envs.size()), v_b, envs);
        if (!pass_pruning_check)
        {
          break;
        }
      }
    }
    else
    {
      VectorXd new_robot_config;
      bool pass_pruning_check = pruning_check_w_manipulator_config(x, env_mode.head(envs.size()), v_b, envs,
                                                                   manipulator_config, new_robot_config, false);
      if (!pass_pruning_check)
      {
        break;
      }
    }

    steer_velocity(v_b, h, this->charac_len);

    // integrate v
    Vector7d x_new = SE32pose(T * se32SE3(v_b));

    this->m_world->updateObjectPose(x_new);

    // --- Check penetration & interpolation (break may happen)
    envs.clear();
    this->m_world->getObjectContacts(&envs, x_new);

    // velocity correction
    int pre_env_size =
        (mode_type == MODE_TYPE_FULL) ? env_mode.size() / 3 : env_mode.size();

    if (envs.size() != 0 && envs.size() == pre_env_size &&
        (ifNeedVelocityCorrection(env_mode, envs)))
    {
      // std::cout << "velocity correction " << counter << std::endl;
      Vector7d x_projected;
      bool if_projected =
          this->project_to_zero_contact_distance(x_new, x_projected);
      if (if_projected)
      {
        x_new = x_projected;
      }

      this->m_world->getObjectContacts(&envs, x_new);
      if ((!if_projected) || (envs.size() != pre_env_size))
      {
        std::cout << "Velocity correction failed! " << std::endl;
        break;
      }
    }

    if (envs.size() > pre_env_size)
    {
      // Detects new contacts, project the object back to zero contact distance
      Vector7d x_projected;
      bool if_projected =
          this->project_to_zero_contact_distance(x_new, x_projected);
      if (if_projected)
      {
        x_new = x_projected;
        path->push_back(x_projected);
      }
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
          // TODO: need to fix this
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

bool TASK::forward_integration_velocity(const Vector7d &x_start,
                                        const Vector6d &v_goal,
                                        const std::vector<ContactPoint> &mnps_,
                                        const std::vector<ContactPoint> &envs_,
                                        const VectorXi &env_mode_,
                                        std::vector<Vector7d> *path)
{

  // The env_mode_ can either be the full mode (cs + ss) or cs mode

  double thr = 1e-4;
  double h = 0.04;
  int max_counter = 150;

  Vector7d x = x_start;
  Vector7d x_goal = SE32pose(pose2SE3(x_start) * se32SE3(v_goal));

  VectorXi env_mode = env_mode_;

  path->push_back(x);

  if (v_goal.norm() < thr)
  {
    std::cout << "Goal velocity is too small." << std::endl;
    return false;
  }

  Vector6d v_goal_dir = v_goal.normalized();

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

  for (counter = 0; counter < max_counter; counter++)
  {
    // compute the goal velocity v_star by multiplying the direction of v_goal
    // and the norm of v_flow
    Vector6d v_star = compute_rbvel_body(x, x_goal);

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
      std::cout << "v_b < thr : " << v_b.transpose() << std::endl;
      break;
    }

    if ((v_b_pre.transpose() * v_b)[0] < -1e-5)
    {
      printf("v_b back and forth. \n");
      break;
    }

    bool pass_pruning_check = this->robot_contact_feasible_check(
        mnps_, x, env_mode.head(envs.size()), v_b, envs);
    if (!pass_pruning_check)
    {
      std::cout << "pass_pruning_check: " << pass_pruning_check << std::endl;
      break;
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
      Vector7d x_projected;
      bool if_projected =
          this->project_to_zero_contact_distance(x_new, x_projected);
      if (if_projected)
      {
        x_new = x_projected;
      }

      this->m_world->getObjectContacts(&envs, x_new);
      if ((!if_projected) || (envs.size() != pre_env_size))
      {
        std::cout << "Velocity correction failed! " << std::endl;
        break;
      }
    }

    if (envs.size() > pre_env_size)
    {
      // Detects new contacts
      Vector7d x_projected;
      bool if_projected =
          this->project_to_zero_contact_distance(x_new, x_projected);
      if (if_projected)
      {
        x_new = x_projected;
        path->push_back(x_projected);
      }
      break;
    }

    // update contact mode if needed (less contact detected)
    if (envs.size() < pre_env_size)
    {
      VectorXi remain_idx = track_contacts_remain(envs_pre, envs);
      if (envs.size() != 0 && remain_idx.size() == 0)
      {
        printf("contact track fails.\n");
        break;
      }
      if (ifContactingModeDeleted(env_mode, remain_idx, envs_pre.size()))
      {
        if (h < 0.004 / 5)
        {
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

    x = x_new;
    envs_pre = envs;
    v_b_pre = v_b;

    path->push_back(x);

    if (counter == max_counter - 1)
    {
      printf("Reach the end.\n");
    }
  }
  std::cout << "counter:" << counter << " x: " << x.transpose() << std::endl;

  return true;
}
// -----------------------------------------------------------
// TASK
void TASK::set_task_parameters(double goal_thr, double wa, double wt,
                               double charac_len, double mu_env, double mu_mnp,
                               Matrix6d object_inertia, Vector6d f_gravity,
                               std::shared_ptr<WorldTemplate> world,
                               int n_robot_contacts, std::string dynamic_type,
                               std::vector<ContactPoint> surface_pts,
                               const SearchOptions &options, bool if_refine,
                               double refine_dist)
{

  this->goal_thr = goal_thr;
  this->wa = wa;
  this->wt = wt;
  this->charac_len = charac_len;
  this->mu_env = mu_env;
  this->mu_mnp = mu_mnp;
  this->object_inertia = object_inertia;
  this->f_gravity = f_gravity;
  this->search_options = options;
  this->m_world = world;
  this->number_of_robot_contacts = n_robot_contacts;
  this->task_dynamics_type = dynamic_type;
  this->object_surface_pts = surface_pts;
  this->if_refine = if_refine;
  this->refine_dist = refine_dist;

  this->m_paramter_set = true;
}

void TASK::set_start_and_goal(const Vector7d &start_object_pose,
                              const Vector7d &goal_object_pose)
{
  this->start_object_pose = start_object_pose;
  this->goal_object_pose = goal_object_pose;
  this->m_start_and_goal_set = true;
}

void TASK::initialize()
{
  if (!this->m_paramter_set)
  {
    std::cout << "Please set the task parameters before initialize the task."
              << std::endl;
    exit(0);
  }
  if (!this->m_start_and_goal_set)
  {
    std::cout << "Please set the start and goal before initialize the task."
              << std::endl;
    exit(0);
  }
  if (!this->m_reward_set)
  {
    std::cout << "Please set the reward function before initialize the task."
              << std::endl;
    exit(0);
  }
  // initialize the shared RRT
  shared_rrt =
      std::make_shared<ReusableRRT>(this->wa, this->charac_len * this->wt);
  ReusableRRT::Node start_node(start_object_pose);

  this->m_world->getObjectContacts(&(start_node.envs), start_node.config);
  cs_mode_enumeration(*this->cons.get(), start_node.envs, &start_node.modes);
  shared_rrt->initial_node(&start_node);

  // calculate total number of finger combinations
  // each finger can be zeros, but other than that do not allow overlap
  this->n_finger_combinations = 0;
  for (int k = 0; k <= this->number_of_robot_contacts; k++)
  {
    // k: number of fingers on a surface contact point
    long int sum_i = combination(this->number_of_robot_contacts, k) *
                     permutation(this->object_surface_pts.size(), k);
    // std::cout << "sum_i: " << sum_i << std::endl;
    this->n_finger_combinations += sum_i;
  }

  this->m_initialized = true;
}

TASK::State TASK::generate_state(const Vector7d &object_pose) const
{
  TASK::State state_;
  state_.m_pose = object_pose;
  this->m_world->getObjectContacts(&state_.envs, object_pose);
  cs_mode_enumeration(*this->cons.get(), state_.envs, &state_.modes);

  return state_;
}

std::vector<TASK::State>
TASK::search_a_new_path(const TASK::State &start_state)
{
  // search a new path towards the end, given the START_STATE and M_MODE_IDX!!!

  // during the search, it figure out the constriants (modes) for the states
  // for now just return linear interpolation towards the sampled state

  // every state need to be associated with m_mode_idx (mode to this state)

  std::vector<TASK::State> path_;

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

  // set_rand_seed();

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

    // Project x_rand to zero contact distance
    {
      Vector7d x_project;
      bool if_projected =
          this->project_to_zero_contact_distance(x_rand, x_project);
      if (if_projected)
      {
        x_rand = x_project;
      }
    }

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
      std::vector<VectorXd> new_manipulator_configs;
      // filter modes with velocity and force checks
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
        if (v.norm() < 1e-6)
        {
          continue;
        }

        // Pruning check with manipulator feasibility
        bool is_pass_pruning;
        VectorXd new_manipulator_config;

        if (!this->search_options.search_with_manipulator_config)
        {
          // If not save manipulator config during search, do a relaxed check: does there exist any feasible manipulator config?
          if ((shared_rrt->nodes[near_idx].parent < 0) ||
              !this->if_transition_pruning)
          {
            is_pass_pruning = (this->pruning_check(
                                   shared_rrt->nodes[near_idx].config, cs_mode,
                                   v, shared_rrt->nodes[near_idx].envs) != -1);
          }
          else
          {
            is_pass_pruning =
                (this->pruning_check_w_transition(
                     shared_rrt->nodes[near_idx].config,
                     shared_rrt->nodes[shared_rrt->nodes[near_idx].parent]
                         .config,
                     cs_mode, v, shared_rrt->nodes[near_idx].envs,
                     shared_rrt->nodes[shared_rrt->nodes[near_idx].parent]
                         .envs) != -1);
          }
        }
        else
        {
          // If save and search the manipulator config, we need to use the saved manipulator config to do the pruning check, and update the manipuator config
          VectorXd new_manipulator_config;
          is_pass_pruning = this->pruning_check_w_manipulator_config(
              shared_rrt->nodes[near_idx].config, cs_mode, v,
              shared_rrt->nodes[near_idx].envs, shared_rrt->nodes[near_idx].manipulator_config,
              new_manipulator_config, true);
          if (is_pass_pruning)
          {
            new_manipulator_configs.push_back(new_manipulator_config);
          }
        }
        if (is_pass_pruning)
        {
          mode_to_extend.push_back(check_mode);
        }
      }
      /// choose sliding mode end
      for (int k_mode = 0; k_mode < mode_to_extend.size(); k_mode++)
      {
        VectorXi mode = mode_to_extend[k_mode];
        std::cout << "Extend mode: " << mode.transpose() << std::endl;

        std::vector<Vector7d> path;

        if (!this->search_options.search_with_manipulator_config)
        {
          // forward integration without predefined manipulator config
          this->forward_integration(shared_rrt->nodes[near_idx].config, x_rand,
                                    shared_rrt->nodes[near_idx].envs, mode,
                                    &path);
        }
        else
        {
          // forward integration with a manipulator config
          this->forward_integration(shared_rrt->nodes[near_idx].config, x_rand,
                                    shared_rrt->nodes[near_idx].envs, mode,
                                    &path,
                                    new_manipulator_configs[k_mode]);
        }

        // If integration is successful
        if (path.size() > 2)
        {
          ReusableRRT::Node new_node(path.back());

          if (this->search_options.search_with_manipulator_config)
          {
            new_node.manipulator_config = new_manipulator_configs[k_mode];
          }

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

            // skip this node if there are enough neighbors on the same
            // manifold

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

          std::cout << "New config " << new_node.config.transpose()
                    << std::endl;

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
      TASK::State new_state(shared_rrt->nodes[kn].config,
                            shared_rrt->nodes[kn].envs, mode_idx,
                            shared_rrt->nodes[kn].modes);
      new_state.m_path = shared_rrt->edges[shared_rrt->nodes[kn].edge].path;

      path_.push_back(new_state);
    }
    TASK::State new_state(shared_rrt->nodes[node_path.back()].config,
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

bool TASK::is_valid(const TASK::State2 &state, const TASK::State2 &prev_state)
{
  // TODO: we need finer path to calculate both quasistatic feasibility and
  // quasidynamic feasibility finer that state1 path (maybe modify the
  // saved_object_trajectory)
  if (state.timestep == 0)
  {
    return true;
  }

  // consider the transition from previous timestep to current state is valid
  int pre_timestep = prev_state.timestep; // state.timestep - 1;
  Vector7d x_object = this->saved_object_trajectory[pre_timestep].m_pose;
  Vector7d x_object_now = this->saved_object_trajectory[state.timestep].m_pose;

  long int finger_idx = state.finger_index;

  std::vector<ContactPoint> mnps;

  // TODO: 0 means no contact!!!
  if (finger_idx != 0)
  {

    // quiry robot configuration from state2 action_index
    VectorXd mnp_config =
        this->get_robot_config_from_action_idx(state.finger_index);

    // update object pose
    this->m_world->updateObjectPose(x_object_now);

    // if there is no ik solution, not valid
    if (!this->m_world->getRobot()->ifIKsolution(mnp_config, x_object_now))
    {
      return false;
    }

    // if the robot collides, not valid
    if (this->m_world->isRobotCollide(mnp_config))
    {
      return false;
    }

    this->m_world->updateObjectPose(x_object);

    // if there is no ik solution, not valid
    if (!this->m_world->getRobot()->ifIKsolution(mnp_config, x_object))
    {
      return false;
    }

    // if the robot collides, not valid
    if (this->m_world->isRobotCollide(mnp_config))
    {
      return false;
    }

    // check if there is quasistatic, or quasidynamic
    {
      std::vector<int> fingertip_idx =
          this->get_finger_locations(state.finger_index);
      std::vector<ContactPoint> fingertips;
      for (int idx : fingertip_idx)
      {
        if (idx != -1)
        {
          fingertips.push_back(this->object_surface_pts[idx]);
        }
      }
      this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
    }
  }

  bool dynamic_feasibility;
  Vector6d v = compute_rbvel_body(x_object, x_object_now);

  if (this->task_dynamics_type == "quasistatic")
  {

    dynamic_feasibility = isQuasistatic(
        mnps, this->saved_object_trajectory[pre_timestep].envs,
        mode_from_velocity(v, this->saved_object_trajectory[pre_timestep].envs,
                           this->cons.get()),
        this->f_gravity, x_object, this->mu_env, this->mu_mnp,
        this->cons.get());
  }
  else if (this->task_dynamics_type == "quasidynamic")
  {
    double h_time = 10.0;

    VectorXi env_mode_full = mode_from_velocity(
        v, this->saved_object_trajectory[pre_timestep].envs, this->cons.get());

    dynamic_feasibility = isQuasidynamic(
        v, mnps, this->saved_object_trajectory[pre_timestep].envs,
        env_mode_full, this->f_gravity, this->object_inertia, x_object,
        this->mu_env, this->mu_mnp, this->wa, this->wt, h_time,
        this->cons.get(), QUASIDYNAMIC_RANGE);
  }

  if (dynamic_feasibility)
  {

    // also check if the relocation is feasible

    std::vector<int> cur_fingertips =
        this->get_finger_locations(state.finger_index);
    std::vector<int> pre_fingertips =
        this->get_finger_locations(prev_state.finger_index);
    std::vector<int> remain_idxes;
    bool if_relocate = false;
    for (int k = 0; k < cur_fingertips.size(); ++k)
    {
      if (cur_fingertips[k] == pre_fingertips[k])
      {
        remain_idxes.push_back(pre_fingertips[k]);
      }
      else if (pre_fingertips[k] == -1)
      {
        remain_idxes.push_back(cur_fingertips[k]);
      }
      else
      {
        if_relocate = true;
      }
    }

    if (if_relocate)
    {

      std::vector<ContactPoint> remain_fingertips;
      for (auto ir : remain_idxes)
      {
        remain_fingertips.push_back(this->object_surface_pts[ir]);
      }
      std::vector<ContactPoint> remain_mnps;
      this->m_world->getRobot()->Fingertips2PointContacts(remain_fingertips,
                                                          &remain_mnps);

      Eigen::VectorXi ss_mode_relocate = Eigen::VectorXi::Zero(
          this->saved_object_trajectory[pre_timestep].envs.size() * 3);
      dynamic_feasibility = isQuasistatic(
          remain_mnps, this->saved_object_trajectory[pre_timestep].envs,
          ss_mode_relocate, this->f_gravity, x_object, this->mu_env,
          this->mu_mnp, this->cons.get());
    }
  }

  return dynamic_feasibility;
}

bool TASK::is_valid_transition(const TASK::State2 &state,
                               const TASK::State2 &prev_state)
{

  // also check if the relocation is feasible

  std::vector<int> cur_fingertips =
      this->get_finger_locations(state.finger_index);
  std::vector<int> pre_fingertips =
      this->get_finger_locations(prev_state.finger_index);
  std::vector<int> remain_idxes;
  bool if_relocate = false;
  for (int k = 0; k < cur_fingertips.size(); ++k)
  {
    if (cur_fingertips[k] == pre_fingertips[k])
    {
      remain_idxes.push_back(pre_fingertips[k]);
    }
    else if (pre_fingertips[k] == -1)
    {
      remain_idxes.push_back(cur_fingertips[k]);
    }
    else
    {
      if_relocate = true;
      // break;
    }
  }

  if (if_relocate)
  {

    std::vector<ContactPoint> remain_fingertips;
    for (auto ir : remain_idxes)
    {
      remain_fingertips.push_back(this->object_surface_pts[ir]);
    }
    std::vector<ContactPoint> remain_mnps;
    this->m_world->getRobot()->Fingertips2PointContacts(remain_fingertips,
                                                        &remain_mnps);

    Eigen::VectorXi ss_mode_relocate = Eigen::VectorXi::Zero(
        this->saved_object_trajectory[state.timestep].envs.size() * 3);

    bool dynamic_feasibility;

    if (this->task_dynamics_type == "quasistatic")
    {
      dynamic_feasibility = isQuasistatic(
          remain_mnps, this->saved_object_trajectory[state.timestep].envs,
          ss_mode_relocate, this->f_gravity,
          this->saved_object_trajectory[state.timestep].m_pose, this->mu_env,
          this->mu_mnp, this->cons.get());
    }
    else if (this->task_dynamics_type == "quasidynamic")
    {
      if (state.timestep == this->saved_object_trajectory.size() - 1)
      {
        return true;
      }
      double h_time = 10.0;
      Vector6d v_b = compute_rbvel_body(
          this->saved_object_trajectory[state.timestep].m_pose,
          this->saved_object_trajectory[state.timestep + 1].m_pose);
      dynamic_feasibility = isQuasidynamic(
          v_b, remain_mnps, this->saved_object_trajectory[state.timestep].envs,
          ss_mode_relocate, this->f_gravity, this->object_inertia,
          this->saved_object_trajectory[state.timestep].m_pose, this->mu_env,
          this->mu_mnp, this->wa, this->wt, 1.0, this->cons.get(), QUASIDYNAMIC_RANGE);
      // return true;
    }
    else
    {
      dynamic_feasibility = true;
    }
    return dynamic_feasibility;
  }
  return true;
}

bool TASK::is_valid_transition(long int pre_finger_idx, long int finger_idx,
                               const Vector7d &x,
                               const std::vector<ContactPoint> &envs)
{

  // also check if the relocation is feasible

  std::vector<int> cur_fingertips = this->get_finger_locations(finger_idx);
  std::vector<int> pre_fingertips = this->get_finger_locations(pre_finger_idx);
  std::vector<int> remain_idxes;
  bool if_relocate = false;
  for (int k = 0; k < cur_fingertips.size(); ++k)
  {
    if (cur_fingertips[k] == pre_fingertips[k])
    {
      remain_idxes.push_back(pre_fingertips[k]);
    }
    else if (pre_fingertips[k] == -1)
    {
      remain_idxes.push_back(cur_fingertips[k]);
    }
    else
    {
      if_relocate = true;
    }
  }

  if (if_relocate)
  {

    std::vector<ContactPoint> remain_fingertips;
    for (auto ir : remain_idxes)
    {
      remain_fingertips.push_back(this->object_surface_pts[ir]);
    }
    std::vector<ContactPoint> remain_mnps;
    this->m_world->getRobot()->Fingertips2PointContacts(remain_fingertips,
                                                        &remain_mnps);

    Eigen::VectorXi ss_mode_relocate = Eigen::VectorXi::Zero(envs.size() * 3);
    bool dynamic_feasibility =
        isQuasistatic(remain_mnps, envs, ss_mode_relocate, this->f_gravity, x,
                      this->mu_env, this->mu_mnp, this->cons.get());
    return dynamic_feasibility;
  }
  return true;
}

double TASK::total_finger_change_ratio(const std::vector<State2> &path)
{

  double finger_change = 0.0;
  for (int k = 0; k < path.size() - 1; ++k)
  {
    finger_change +=
        double(number_of_different_elements(
            this->get_finger_locations(path[k].finger_index),
            this->get_finger_locations(path[k + 1].finger_index))) /
        double(this->number_of_robot_contacts);
  }

  return finger_change;
}

void TASK::save_trajectory(const std::vector<TASK::State> &path)
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

std::vector<int> TASK::get_finger_locations(long int finger_location_index)
{

  // obtain finger location idxes from the single location idx

  int N =
      this->object_surface_pts.size();    // N: number of surface contact points
  int n = this->number_of_robot_contacts; // n: number of fingers
  long int action_idx =
      finger_location_index; // action_idx: index of the action

  std::vector<int> locations;

  // find out the number of active fingers
  int k = 0; // the number of active fingers
  long int sum = 0;
  for (k = 0; k <= n; k++)
  {
    // k: number of fingers on a surface contact point
    long int sum_i = combination(n, k) * permutation(N, k);
    if (sum + sum_i > action_idx)
    {
      break;
    }
    sum += sum_i;
  }

  // find out active finger indices
  std::vector<int> active_idxes;
  long int comb_idx = (action_idx - sum) / permutation(N, k);
  active_idxes = combination_set(n, k, comb_idx);
  // find out the locations of active finger indices
  long int loc_idx = (action_idx - sum) % permutation(N, k);
  std::vector<int> loc_idxes;
  loc_idxes = permutation_set(N, k, loc_idx);

  // create a vector of -1 with size n

  for (int i = 0; i < n; i++)
  {
    locations.push_back(-1);
  }

  // assign locations to active fingers
  for (int i = 0; i < active_idxes.size(); ++i)
  {
    locations[active_idxes[i]] = loc_idxes[i];
  }

  return locations;
}

long int
TASK::finger_locations_to_finger_idx(const std::vector<int> &finger_idxs)
{

  long int finger_idx = 0;

  int n_active = 0;
  std::vector<int> active_idxes;
  std::vector<int> loc_idxes;
  for (int i = 0; i < finger_idxs.size(); i++)
  {
    if (finger_idxs[i] != -1)
    {
      n_active += 1;
      active_idxes.push_back(i);
      loc_idxes.push_back(finger_idxs[i]);
    }
  }

  for (int k = 0; k < n_active; k++)
  {
    finger_idx += combination(this->number_of_robot_contacts, k) *
                  permutation(this->object_surface_pts.size(), k);
  }

  long int comb_idx = index_in_combination_set(this->number_of_robot_contacts,
                                               n_active, active_idxes);
  long int loc_idx = index_in_permutation_set(this->object_surface_pts.size(),
                                              n_active, loc_idxes);

  finger_idx +=
      comb_idx * permutation(this->object_surface_pts.size(), n_active) +
      loc_idx;

  return finger_idx;
}

bool TASK::robot_contact_feasible_check(long int finger_idx, const Vector7d &x,
                                        const VectorXi &cs_mode,
                                        const Vector6d &v,
                                        const std::vector<ContactPoint> &envs)
{

  VectorXi env_mode = mode_from_velocity(v, envs, this->cons.get());
  env_mode.head(envs.size()) = cs_mode;

  bool dynamic_feasibility = false;

  std::vector<ContactPoint> mnps;
  std::vector<int> fingertip_idx;
  if (finger_idx != 0)
  {
    VectorXd mnp_config = this->get_robot_config_from_action_idx(finger_idx);

    // update object pose
    this->m_world->updateObjectPose(x);

    // if there is no ik solution, not valid
    if (!this->m_world->getRobot()->ifIKsolution(mnp_config, x))
    {
      return false;
    }

    if (this->m_world->isRobotCollide(mnp_config))
    {
      return false;
    }

    // check if there is quasistatic, or quasidynamic

    {
      fingertip_idx = this->get_finger_locations(finger_idx);
      std::vector<ContactPoint> fingertips;
      for (int idx : fingertip_idx)
      {
        if (idx != -1)
        {
          fingertips.push_back(this->object_surface_pts[idx]);
        }
      }
      this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
    }
  }

  if (this->task_dynamics_type == "quasistatic")
  {

    dynamic_feasibility =
        isQuasistatic(mnps, envs, env_mode, this->f_gravity, x, this->mu_env,
                      this->mu_mnp, this->cons.get());
  }
  else if (this->task_dynamics_type == "quasidynamic")
  {
    double h_time = 10.0;

    dynamic_feasibility =
        isQuasidynamic(v, mnps, envs, env_mode, this->f_gravity,
                       this->object_inertia, x, this->mu_env, this->mu_mnp,
                       this->wa, this->wt, h_time, this->cons.get(), QUASIDYNAMIC_RANGE);
  }

  return dynamic_feasibility;
}

bool TASK::robot_contact_feasible_check(
    const std::vector<ContactPoint> &fingertips, const Vector7d &x,
    const VectorXi &cs_mode, const Vector6d &v,
    const std::vector<ContactPoint> &envs)
{

  VectorXi env_mode = mode_from_velocity(v, envs, this->cons.get());
  env_mode.head(envs.size()) = cs_mode;

  bool dynamic_feasibility = false;

  std::vector<ContactPoint> mnps;
  std::vector<int> fingertip_idx;
  if (fingertips.size() != 0)
  {
    VectorXd mnp_config = this->get_robot_config_from_points(fingertips);

    // update object pose
    this->m_world->updateObjectPose(x);

    // if there is no ik solution, not valid
    if (!this->m_world->getRobot()->ifIKsolution(mnp_config, x))
    {
      return false;
    }

    if (this->m_world->isRobotCollide(mnp_config))
    {
      return false;
    }

    // check if there is quasistatic, or quasidynamic
    {
      this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
    }
  }

  if (this->task_dynamics_type == "quasistatic")
  {

    dynamic_feasibility =
        isQuasistatic(mnps, envs, env_mode, this->f_gravity, x, this->mu_env,
                      this->mu_mnp, this->cons.get());
  }
  else if (this->task_dynamics_type == "quasidynamic")
  {
    double h_time = 10.0;

    dynamic_feasibility =
        isQuasidynamic(v, mnps, envs, env_mode, this->f_gravity,
                       this->object_inertia, x, this->mu_env, this->mu_mnp,
                       this->wa, this->wt, h_time, this->cons.get(), QUASIDYNAMIC_RANGE);
  }

  return dynamic_feasibility;
}

long int TASK::pruning_check(const Vector7d &x, const VectorXi &cs_mode,
                             const Vector6d &v,
                             const std::vector<ContactPoint> &envs)
{

  VectorXi env_mode = mode_from_velocity(v, envs, this->cons.get());
  env_mode.head(envs.size()) = cs_mode;

  bool dynamic_feasibility = false;
  int max_sample = 100;
  max_sample = (max_sample > this->n_finger_combinations)
                   ? this->n_finger_combinations
                   : max_sample;

  std::vector<long int> sampled_finger_idxes;
  std::vector<double> probs;
  this->sample_likely_feasible_finger_idx(x, max_sample, &sampled_finger_idxes,
                                          &probs);

  long int finger_idx = 0;

  for (auto sample_finger_idx : sampled_finger_idxes)
  {
    finger_idx = sample_finger_idx;
    std::vector<ContactPoint> mnps;
    std::vector<int> fingertip_idx;
    if (finger_idx != 0)
    {
      VectorXd mnp_config = this->get_robot_config_from_action_idx(finger_idx);

      // update object pose
      this->m_world->updateObjectPose(x);

      // if there is no ik solution, not valid
      if (!this->m_world->getRobot()->ifIKsolution(mnp_config, x))
      {
        continue;
      }

      // if the robot collides, not valid
      if (this->m_world->isRobotCollide(mnp_config))
      {
        continue;
      }

      // check if there is quasistatic, or quasidynamic

      {
        fingertip_idx = this->get_finger_locations(finger_idx);
        std::vector<ContactPoint> fingertips;
        for (int idx : fingertip_idx)
        {
          if (idx != -1)
          {
            fingertips.push_back(this->object_surface_pts[idx]);
          }
        }
        this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
      }
    }

    if (this->task_dynamics_type == "quasistatic")
    {

      dynamic_feasibility =
          isQuasistatic(mnps, envs, env_mode, this->f_gravity, x, this->mu_env,
                        this->mu_mnp, this->cons.get());
    }
    else if (this->task_dynamics_type == "quasidynamic")
    {
      double h_time = 10.0;
      dynamic_feasibility =
          isQuasidynamic(v, mnps, envs, env_mode, this->f_gravity,
                         this->object_inertia, x, this->mu_env, this->mu_mnp,
                         this->wa, this->wt, h_time, this->cons.get(), QUASIDYNAMIC_RANGE);
    }

    if (dynamic_feasibility)
    {
      break;
    }
  }

  // {
  //   // test: delele this afterwards
  //   std::vector<ContactPoint> fingertips;
  //   fingertips.push_back(ContactPoint(Vector3d(-0.5,0,0), Vector3d(0,0,1)));
  //   std::vector<ContactPoint> mnps;
  //   this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
  //   bool test_dynamic_feasibility =
  //         isQuasidynamic(v, mnps, envs, env_mode, this->f_gravity,
  //                        this->object_inertia, x, this->mu_env, this->mu_mnp,
  //                        this->wa, this->wt, 10.0, this->cons.get(), QUASIDYNAMIC_RANGE);
  //   std::cout << "test dynamic feasibility: " << test_dynamic_feasibility << std::endl;

  // }

  if (dynamic_feasibility)
  {
    return finger_idx;
  }
  else
  {
    return -1;
  }

}

long int
TASK::pruning_check_w_transition(const Vector7d &x, const Vector7d &x_pre,
                                 const VectorXi &cs_mode, const Vector6d &v,
                                 const std::vector<ContactPoint> &envs,
                                 const std::vector<ContactPoint> &envs_pre)
{
  // while loop
  // sample a finger from last timestep
  // check if there exist a transition to the next step
  Vector6d v_pre = compute_rbvel_body(x_pre, x);
  VectorXi cs_mode_pre = mode_from_velocity(v_pre, envs_pre, this->cons.get())
                             .head(envs_pre.size());

  int max_sample = 50;

  max_sample = (max_sample > this->n_finger_combinations)
                   ? this->n_finger_combinations
                   : max_sample;

  std::vector<long int> sampled_finger_idxes_pre;
  std::vector<double> probs_pre;
  this->sample_likely_feasible_finger_idx(
      x_pre, max_sample, &sampled_finger_idxes_pre, &probs_pre);

  std::vector<long int> sampled_finger_idxes;
  std::vector<double> probs;
  this->sample_likely_feasible_finger_idx(x, max_sample, &sampled_finger_idxes,
                                          &probs);

  for (int k_sample = 0; k_sample < sampled_finger_idxes_pre.size();
       k_sample++)
  {
    int i_sample = randi(sampled_finger_idxes_pre.size());
    long int finger_idx_pre = sampled_finger_idxes_pre[i_sample];

    bool if_feasible_pre = robot_contact_feasible_check(
        finger_idx_pre, x_pre, cs_mode_pre, v_pre, envs_pre);

    if (!if_feasible_pre)
    {
      sampled_finger_idxes_pre.erase(sampled_finger_idxes_pre.begin() +
                                     i_sample);
      continue;
    }

    // another for loop checks for valid next step and transition
    for (int kk_sample = 0; kk_sample < sampled_finger_idxes.size();
         kk_sample++)
    {
      int ii_sample = randi(sampled_finger_idxes.size());
      long int finger_idx = sampled_finger_idxes[ii_sample];
      bool if_feasible =
          robot_contact_feasible_check(finger_idx, x, cs_mode, v, envs);
      if (!if_feasible)
      {
        sampled_finger_idxes.erase(sampled_finger_idxes.begin() + ii_sample);
        continue;
      }
      bool if_transition =
          this->is_valid_transition(finger_idx_pre, finger_idx, x, envs);
      if (if_transition)
      {
        return finger_idx;
      }
    }
  }
  return -1;
}

bool TASK::randomRelocateFingers(const VectorXd &mnp_config, Vector7d x, Vector6d v,
                                 const std::vector<ContactPoint> &envs,
                                 const VectorXi &env_mode,
                                 VectorXd &new_config, int samples)
{
  this->m_world->updateObjectPose(x);

  VectorXd new_mnp_config(0);
  // TODO: this can be again formulated as a search problem
  int n_pts = this->number_of_robot_contacts;

  VectorXi relocate_mode(env_mode.size());
  relocate_mode.setZero();

  for (int k_sample = 0; k_sample < samples; k_sample++)
  {

    int n_on = randi(n_pts + 1);

    std::vector<ContactPoint> remain_mnps;
    bool isresample = this->m_world->getRobot()->resampleFingers(
        n_on, mnp_config, x, this->object_surface_pts, new_mnp_config,
        &remain_mnps);

    if (!isresample)
    {
      continue;
    }

    bool isremainbalance = false;
    {
      std::vector<ContactPoint> remain_grasp_mnps;
      this->m_world->getRobot()->Fingertips2PointContacts(remain_mnps,
                                                          &remain_grasp_mnps);
      if (this->task_dynamics_type == "quasistatic")
      {

        isremainbalance =
            isQuasistatic(remain_grasp_mnps, envs, relocate_mode, this->f_gravity, x, this->mu_env,
                          this->mu_mnp, this->cons.get());
      }
      else if (this->task_dynamics_type == "quasidynamic")
      {
        double h_time = 10.0;
        // Vector6d v_relocate;
        // v_relocate.setZero();
        isremainbalance =
            isQuasidynamic(v, remain_grasp_mnps, envs, relocate_mode, this->f_gravity,
                           this->object_inertia, x, this->mu_env, this->mu_mnp,
                           this->wa, this->wt, h_time, this->cons.get(), QUASIDYNAMIC_RANGE);
      }
    }
    if (!isremainbalance)
    {
      continue;
    }

    bool isbalance = false;
    {
      std::vector<ContactPoint> grasp_mnps;
      std::vector<ContactPoint> mnp_fingertips;
      this->m_world->getRobot()->getFingertipsOnObject(new_mnp_config, x,
                                                       &mnp_fingertips);
      this->m_world->getRobot()->Fingertips2PointContacts(mnp_fingertips,
                                                          &grasp_mnps);
      if (this->task_dynamics_type == "quasistatic")
      {

        isremainbalance =
            isQuasistatic(grasp_mnps, envs, env_mode, this->f_gravity, x, this->mu_env,
                          this->mu_mnp, this->cons.get());
      }
      else if (this->task_dynamics_type == "quasidynamic")
      {
        double h_time = 10.0;
        isremainbalance =
            isQuasidynamic(v, grasp_mnps, envs, env_mode, this->f_gravity,
                           this->object_inertia, x, this->mu_env, this->mu_mnp,
                           this->wa, this->wt, h_time, this->cons.get(), QUASIDYNAMIC_RANGE);
      }
    }
    if (!isbalance)
    {
      continue;
    }

    bool ifCollide = this->m_world->isRobotCollide(new_mnp_config);
    if (ifCollide)
    {
      continue;
    }

    new_config = new_mnp_config;
    return true;
  }

  return false;
}

bool TASK::pruning_check_w_manipulator_config(const Vector7d &x, const VectorXi &cs_mode, const Vector6d &v,
                                              const std::vector<ContactPoint> &envs,
                                              const VectorXd &robot_config, VectorXd &new_robot_config,
                                              bool if_allow_transition)
{

  // Check if current robot configuration is valid
  if(robot_config.size() == 0){
    int selected_finger_idx =
            this->pruning_check(x, cs_mode, v, envs);
    new_robot_config = this->get_robot_config_from_action_idx(selected_finger_idx);
    return true;
  }
  this->m_world->updateObjectPose(x);

  VectorXi env_mode = mode_from_velocity(v, envs, this->cons.get());
  env_mode.head(envs.size()) = cs_mode;

  std::vector<ContactPoint> mnps;
  std::vector<ContactPoint> fingertips;
  this->m_world->getRobot()->getFingertipsOnObject(robot_config, x, &fingertips);
  this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);

  bool current_valid = true;
  bool dynamic_feasibility = true;
  // if there is no ik solution, not valid

  if (!this->m_world->getRobot()->ifIKsolution(robot_config, x))
  {
    current_valid = false;
  }

  // if the robot collides, not valid
  if (current_valid)
  {
    if (this->m_world->isRobotCollide(robot_config))
    {
      current_valid = false;
    }
  }

  if (current_valid)
  {
    if (this->task_dynamics_type == "quasistatic")
    {

      dynamic_feasibility =
          isQuasistatic(mnps, envs, env_mode, this->f_gravity, x, this->mu_env,
                        this->mu_mnp, this->cons.get());
    }
    else if (this->task_dynamics_type == "quasidynamic")
    {
      double h_time = 10.0;
      dynamic_feasibility =
          isQuasidynamic(v, mnps, envs, env_mode, this->f_gravity,
                         this->object_inertia, x, this->mu_env, this->mu_mnp,
                         this->wa, this->wt, h_time, this->cons.get(), QUASIDYNAMIC_RANGE);
    }

    if (!dynamic_feasibility)
    {
      current_valid = false;
    }
  }

  // Randomly choose relocate even if current config is valid
  bool random_relocate = randd() < 0.3;
  if (current_valid && !random_relocate)
  {
    new_robot_config = robot_config;
    return true;
  }

  // If current robot config is not valid, check if can find valid transition and update new_robot_config

  VectorXd relocate_robot_config;
  int max_sample = 30;

  bool is_relocatable = this->randomRelocateFingers(robot_config, x, v, envs, env_mode,
                                                    relocate_robot_config, max_sample);

  if (is_relocatable)
  {
    new_robot_config = relocate_robot_config;
    return true;
  }
  else
  {
    if (current_valid)
    {
      new_robot_config = robot_config;
      return true;
    }
    else
    {
      return false;
    }
  }
}

int TASK::max_forward_timestep(const TASK::State2 &state)
{
  int t_max;

  for (t_max = state.timestep; t_max < this->saved_object_trajectory.size();
       ++t_max)
  {
    bool is_feasible = this->is_finger_valid(state.finger_index, t_max);
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

int TASK::select_finger_change_timestep(const TASK::State2 &state)
{
  // select a timestep to change the finger configuration
  int t_max;
  if (state.t_max == -1)
  {
    t_max = this->max_forward_timestep(state);
  }
  else
  {
    t_max = state.t_max;
  }

  int t;
  double random_prob = 0.5;
  if ((randd() > random_prob) || (t_max + 1 - state.timestep) <= 1)
  {
    t = t_max + 1;
  }
  else
  {
    t = randi(t_max + 1 - state.timestep) + state.timestep + 1;
  }
  if (t > this->saved_object_trajectory.size() - 1)
  {
    t = this->saved_object_trajectory.size() - 1;
  }
  return t;
}

bool TASK::is_finger_valid(long int finger_idx, int timestep)
{
  // check if the finger is valid to move one timestep forward

  // check for the validity of timestep and timestep+1

  Vector7d x_object = this->saved_object_trajectory[timestep].m_pose;
  Vector7d x_object_next;
  VectorXi reference_cs_mode(
      this->saved_object_trajectory[timestep].envs.size());
  reference_cs_mode.setZero();
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

  std::vector<ContactPoint> mnps;

  // TODO: 0 means no contact!!!
  if (finger_idx != 0)
  {
    // check current collision and IK
    // quiry robot configuration from state2 action_index
    VectorXd mnp_config = this->get_robot_config_from_action_idx(finger_idx);

    // update object pose
    this->m_world->updateObjectPose(x_object);

    // if there is no ik solution, not valid
    if (!this->m_world->getRobot()->ifIKsolution(mnp_config, x_object))
    {
      return false;
    }

    // if the robot collides, not valid
    if (this->m_world->isRobotCollide(mnp_config))
    {
      return false;
    }

    if (timestep < this->saved_object_trajectory.size() - 1)
    {

      this->m_world->updateObjectPose(x_object_next);

      // if there is no ik solution, not valid
      if (!this->m_world->getRobot()->ifIKsolution(mnp_config, x_object_next))
      {
        return false;
      }

      // if the robot collides, not valid
      if (this->m_world->isRobotCollide(mnp_config))
      {
        return false;
      }
    }
  }

  // check if there is quasistatic, or quasidynamic
  {
    std::vector<int> fingertip_idx = this->get_finger_locations(finger_idx);
    std::vector<ContactPoint> fingertips;
    for (int idx : fingertip_idx)
    {
      if (idx != -1)
      {
        fingertips.push_back(this->object_surface_pts[idx]);
      }
    }
    this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
  }

  bool dynamic_feasibility;

  if ((this->task_dynamics_type == "quasistatic") || v.norm() < 1e-6)
  {

    VectorXi conservative_full_mode = conservative_mode_from_velocity(
        this->saved_object_trajectory[timestep].envs, reference_cs_mode, v,
        this->cons.get());

    dynamic_feasibility =
        isQuasistatic(mnps, this->saved_object_trajectory[timestep].envs,
                      conservative_full_mode, this->f_gravity, x_object,
                      this->mu_env, this->mu_mnp, this->cons.get());
  }
  else if (this->task_dynamics_type == "quasidynamic")
  {
    double h_time = 10.0;

    VectorXi env_mode = mode_from_velocity(
        v, this->saved_object_trajectory[timestep].envs, this->cons.get());
    env_mode.head(this->saved_object_trajectory[timestep].envs.size()) =
        reference_cs_mode;

    dynamic_feasibility = isQuasidynamic(
        v, mnps, this->saved_object_trajectory[timestep].envs, env_mode,
        this->f_gravity, this->object_inertia, x_object, this->mu_env,
        this->mu_mnp, this->wa, this->wt, h_time, this->cons.get(), QUASIDYNAMIC_RANGE);
  }

  return dynamic_feasibility;
}

std::vector<double>
TASK::get_path_features(const std::vector<State> &object_path,
                        const std::vector<State2> &robot_contact_path,
                        const std::vector<std::string> &feature_names)
{
  std::vector<double> features;

  for (auto feature_name : feature_names)
  {

    double x;

    if (feature_name == "object_path_size")
    {

      x = double(object_path.size());
    }
    else if (feature_name == "finger_change_ratio")
    {

      double total_finger_changes =
          this->total_finger_change_ratio(robot_contact_path);

      x = total_finger_changes / double(robot_contact_path.back().t_max);
    }
    else if (feature_name == "number_environment_contact_changes")
    {

      x = double(this->number_environment_contact_changes(object_path));
    }
    else if (feature_name == "object_travel_distance_ratio")
    {

      double dist = this->travel_distance(object_path);
      double best_dist = this->shared_rrt->dist(this->start_object_pose,
                                                this->goal_object_pose);
      x = dist / best_dist;
    }
    else if (feature_name == "average_grasp_centroid_distance")
    {

      double avg_grasp_d = 0.0;
      for (auto s2 : robot_contact_path)
      {
        if (s2.finger_index == -1 || s2.timestep == -1)
        {
          continue;
        }
        double grasp_d = this->grasp_measure(s2.finger_index, s2.timestep);
        avg_grasp_d += grasp_d;
      }
      x = avg_grasp_d / (double(robot_contact_path.size()) - 1);
    }
    else if (feature_name == "average_distance_to_goal_fingertips")
    {
      if (!this->if_goal_finger)
      {
        x = -1.0;
      }
      else
      {
        double total_finger_distance = 0.0;
        for (auto state : robot_contact_path)
        {
          total_finger_distance += get_finger_distance(state.finger_index);
        }
        // let final distance to be half of the average distance
        total_finger_distance +=
            double(robot_contact_path.size()) *
            get_finger_distance(robot_contact_path.back().finger_index);
        total_finger_distance /= double(this->number_of_robot_contacts);
        total_finger_distance /= 2 * double(robot_contact_path.size());
        x = total_finger_distance;
      }
    }
    else
    {
      std::cout << "Error in TASK::get_path_features: feature name not found"
                << std::endl;
      exit(0);
    }
    features.push_back(x);
  }

  return features;
}

void TASK::sample_likely_feasible_finger_idx(
    Vector7d x_object, int number, std::vector<long int> *finger_idxs,
    std::vector<double> *probabilities)
{
  if (this->action_prob_L2 == "env")
  {
    // CMG TASK
    for (int n = number; n > 0; n--)
    {
      finger_idxs->push_back(randi(this->n_finger_combinations));
      probabilities->push_back(this->finger_idx_prob(finger_idxs->back(), -1));
    }
  }
  else if (this->action_prob_L2 == "inhand")
  {
    // Inhand task

    std::vector<ContactPoint> object_surface_world;
    Eigen::Matrix4d T;
    T = pose2SE3(x_object);
    Eigen::Matrix3d R;
    R = T.block(0, 0, 3, 3);
    Eigen::Vector3d p;
    p = T.block(0, 3, 3, 1);

    for (auto pt : this->object_surface_pts)
    {
      ContactPoint pt_w;
      pt_w.p = R * pt.p + p;
      pt_w.n = R * pt.n;
      object_surface_world.push_back(pt_w);
    }

    std::vector<int> active_idxes;
    for (int k = 0; k < this->number_of_robot_contacts; ++k)
    {
      if (randd() < 0.9)
      {
        active_idxes.push_back(k);
      }
    }

    std::vector<std::vector<int>> points_in_workspaces;

    std::vector<int> locs_;

    for (int k = 0; k < this->number_of_robot_contacts; ++k)
    {
      std::vector<int> pws;
      points_in_workspaces.push_back(pws);
      locs_.push_back(-1);
    }
    for (int k : active_idxes)
    {
      this->m_world->getRobot()->points_in_workspace(k, object_surface_world,
                                                     &points_in_workspaces[k]);
    }

    for (int iter = 0; iter < number; iter++)
    {
      std::vector<int> locs;
      locs = locs_;

      for (int k : active_idxes)
      {
        if (points_in_workspaces[k].size() == 0)
        {
          locs[k] = -1;
        }
        else
        {
          locs[k] =
              points_in_workspaces[k][randi(points_in_workspaces[k].size())];
        }
      }
      if (is_repeated_idxes(locs))
      {
        continue;
      }
      finger_idxs->push_back(this->finger_locations_to_finger_idx(locs));

      probabilities->push_back(this->finger_idx_prob(finger_idxs->back(), -1));
    }
  }
  return;
}

void TASK::sample_likely_feasible_finger_idx(
    State2 state, double t_change, int number,
    std::vector<long int> *finger_idxs, std::vector<double> *probabilities)
{

  // CMG TASK
  if (this->action_prob_L2 == "env")
  {
    for (int n = number; n > 0; n--)
    {
      finger_idxs->push_back(randi(this->n_finger_combinations));
      probabilities->push_back(
          this->finger_idx_prob(finger_idxs->back(), t_change));
    }
  }
  else if (this->action_prob_L2 == "inhand")
  {

    // Inhand Task

    // sample the finger idxes that are likely to be feasible in this state
    // and move to the next state

    // feasible: in workspace & changing contact

    // 1. find contact points in workspace
    Vector7d x_object = this->saved_object_trajectory[t_change].m_pose;
    Vector7d x_object_next;
    bool check_next_step = true;
    if (t_change == this->saved_object_trajectory.size() - 1)
    {
      check_next_step = false;
    }
    else
    {
      x_object_next = this->saved_object_trajectory[t_change + 1].m_pose;
    }

    Eigen::VectorXi ss_mode_relocate = Eigen::VectorXi::Zero(
        this->saved_object_trajectory[t_change].envs.size() * 3);

    std::vector<ContactPoint> object_surface_world_current;
    std::vector<ContactPoint> object_surface_world_next;
    {
      Eigen::Matrix4d T;
      T = pose2SE3(x_object);
      Eigen::Matrix3d R;
      R = T.block(0, 0, 3, 3);
      Eigen::Vector3d p;
      p = T.block(0, 3, 3, 1);

      for (auto pt : this->object_surface_pts)
      {
        ContactPoint pt_w;
        pt_w.p = R * pt.p + p;
        pt_w.n = R * pt.n;
        object_surface_world_current.push_back(pt_w);
      }
    }
    if (check_next_step)
    {
      Eigen::Matrix4d T;
      T = pose2SE3(x_object_next);
      Eigen::Matrix3d R;
      R = T.block(0, 0, 3, 3);
      Eigen::Vector3d p;
      p = T.block(0, 3, 3, 1);

      for (auto pt : this->object_surface_pts)
      {
        ContactPoint pt_w;
        pt_w.p = R * pt.p + p;
        pt_w.n = R * pt.n;
        object_surface_world_next.push_back(pt_w);
      }
    }

    std::vector<std::vector<int>> points_in_workspaces;

    std::vector<int> locs_;

    for (int k = 0; k < this->number_of_robot_contacts; ++k)
    {
      locs_.push_back(-1);
      std::vector<int> pws;
      std::vector<int> pws_next;
      points_in_workspaces.push_back(pws);
      if (check_next_step)
      {
        this->m_world->getRobot()->points_in_workspace(
            k, object_surface_world_current, &pws);
        this->m_world->getRobot()->points_in_workspace(
            k, object_surface_world_next, &pws_next);
        for (int pt_i : pws)
        {
          if (std::find(pws_next.begin(), pws_next.end(), pt_i) !=
              pws_next.end())
          {
            points_in_workspaces[k].push_back(pt_i);
          }
        }
      }
      else
      {
        this->m_world->getRobot()->points_in_workspace(
            k, object_surface_world_next, &points_in_workspaces[k]);
      }
    }

    // 2. find feasible finger changing pattern

    std::vector<int> pre_fingertips =
        this->get_finger_locations(state.finger_index);

    std::vector<std::vector<int>> remain_idxes;

    for (int finger_stay = 0; finger_stay < this->number_of_robot_contacts;
         ++finger_stay)
    {
      for (int idx = 0;
           idx < combination(this->number_of_robot_contacts, finger_stay);
           ++idx)
      {
        std::vector<int> remain_idx =
            combination_set(this->number_of_robot_contacts, finger_stay, idx);
        bool is_remain_ws =
            true; // is all the remaining fingers in workspace, if not, skip

        // check if the remain_idx can maintain the object in quasistatic
        // balance
        std::vector<ContactPoint> remain_fingertips;
        for (int ir : remain_idx)
        {
          int finger_loc = pre_fingertips[ir];
          if (std::find(points_in_workspaces[ir].begin(),
                        points_in_workspaces[ir].end(),
                        finger_loc) == points_in_workspaces[ir].end())
          {
            is_remain_ws = false;
            break;
          }
          if (finger_loc != -1)
          {
            remain_fingertips.push_back(this->object_surface_pts[finger_loc]);
          }
        }
        if (!is_remain_ws)
        {
          continue;
        }
        std::vector<ContactPoint> remain_mnps;
        this->m_world->getRobot()->Fingertips2PointContacts(remain_fingertips,
                                                            &remain_mnps);

        bool dynamic_feasibility = isQuasistatic(
            remain_mnps, this->saved_object_trajectory[t_change].envs,
            ss_mode_relocate, this->f_gravity,
            this->saved_object_trajectory[t_change].m_pose, this->mu_env,
            this->mu_mnp, this->cons.get());

        if (dynamic_feasibility)
        {
          remain_idxes.push_back(remain_idx);
        }
      }
    }

    // 3. sample from feasible finger contacts

    if (remain_idxes.size() == 0)
    {
      // just random sample quasistatic feasible finger contacts
      for (int iter = 0; iter < number; ++iter)
      {
        std::vector<int> locs = locs_;
        std::vector<int> remain_locs;
        for (int k_finger = 0; k_finger < this->number_of_robot_contacts;
             ++k_finger)
        {
          // sample -1
          // sample relocation
          // sample stay

          if (points_in_workspaces[k_finger].size() == 0)
          {
            locs[k_finger] = -1;
            remain_locs.push_back(locs[k_finger]);
            continue;
          }

          // recore remain_fingertips

          if (pre_fingertips[k_finger] ==
              -1) // if the fingertip was not touching
          {
            // 0.8 relocate, 0.2 stay
            if (randd() < 0.8)
            {
              locs[k_finger] = points_in_workspaces[k_finger][randi(
                  points_in_workspaces[k_finger].size())];
              remain_locs.push_back(locs[k_finger]);
            }
          }
          else // if the fingertip was thouching
          {

            // out of ws
            if (std::find(points_in_workspaces[k_finger].begin(),
                          points_in_workspaces[k_finger].end(),
                          pre_fingertips[k_finger]) ==
                points_in_workspaces[k_finger].end())
            {
              if (randd() < 0.7)
              {
                // change
                locs[k_finger] = points_in_workspaces[k_finger][randi(
                    points_in_workspaces[k_finger].size())];
              }
              else
              {
                // leave
                locs[k_finger] = -1;
                remain_locs.push_back(pre_fingertips[k_finger]);
              }
            }
            else
            {
              // in ws
              if (randd() < 0.7)
              {
                // stay
                locs[k_finger] = pre_fingertips[k_finger];
                remain_locs.push_back(locs[k_finger]);
              }
              else if (randd() < 0.8)
              {
                // change
                locs[k_finger] = points_in_workspaces[k_finger][randi(
                    points_in_workspaces[k_finger].size())];
              }
              else
              {
                // leave
                locs[k_finger] = -1;
                remain_locs.push_back(pre_fingertips[k_finger]);
              }
            }
          }
        }

        if (is_repeated_idxes(locs))
        {
          continue;
        }

        std::vector<ContactPoint> new_fingertips;
        for (int ir : locs)
        {
          if (ir != -1)
          {
            new_fingertips.push_back(this->object_surface_pts[ir]);
          }
        }
        std::vector<ContactPoint> new_mnps;
        this->m_world->getRobot()->Fingertips2PointContacts(new_fingertips,
                                                            &new_mnps);

        bool dynamic_feasibility = isQuasistatic(
            new_mnps, this->saved_object_trajectory[t_change].envs,
            ss_mode_relocate, this->f_gravity,
            this->saved_object_trajectory[t_change].m_pose, this->mu_env,
            this->mu_mnp, this->cons.get());

        std::vector<ContactPoint> remain_fingertips;
        for (int ir : remain_locs)
        {
          if (ir != -1)
          {
            remain_fingertips.push_back(this->object_surface_pts[ir]);
          }
        }
        std::vector<ContactPoint> remain_mnps;
        this->m_world->getRobot()->Fingertips2PointContacts(remain_fingertips,
                                                            &remain_mnps);

        bool remain_dynamic_feasibility = isQuasistatic(
            remain_mnps, this->saved_object_trajectory[t_change].envs,
            ss_mode_relocate, this->f_gravity,
            this->saved_object_trajectory[t_change].m_pose, this->mu_env,
            this->mu_mnp, this->cons.get());

        if (dynamic_feasibility && remain_dynamic_feasibility)
        {
          // sanity check for IK
          // int check_idx = this->finger_locations_to_finger_idx(locs);
          // VectorXd mnp_config =
          // this->get_robot_config_from_action_idx(check_idx); bool ik_next;
          // if (check_next_step)
          // {
          //   ik_next = this->m_world->getRobot()->ifIKsolution(mnp_config,
          //   x_object_next);
          // }
          // bool ik = this->m_world->getRobot()->ifIKsolution(mnp_config,
          // x_object); bool if_feasible = this->is_finger_valid(check_idx,
          // t_change);

          finger_idxs->push_back(this->finger_locations_to_finger_idx(locs));

          probabilities->push_back(
              this->finger_idx_prob(finger_idxs->back(), t_change));
        }
      }
      return;
    }

    // if remain_idxes.size() > 0
    // sample from remain idx

    for (int iter = 0; iter < number; ++iter)
    {
      // random sample a remain idx
      int idx = randi(remain_idxes.size());
      std::vector<int> locs = locs_;
      for (int k_finger = 0; k_finger < this->number_of_robot_contacts;
           ++k_finger)
      {
        if (std::find(remain_idxes[idx].begin(), remain_idxes[idx].end(),
                      k_finger) != remain_idxes[idx].end())
        {
          locs[k_finger] = pre_fingertips[k_finger];
        }
        else
        {

          if ((randd() > 0.7) || points_in_workspaces[k_finger].size() == 0)
          {
            locs[k_finger] = -1;
          }
          else
          {
            locs[k_finger] = points_in_workspaces[k_finger][randi(
                points_in_workspaces[k_finger].size())];
          }
        }
      }
      if (is_repeated_idxes(locs))
      {
        continue;
      }
      finger_idxs->push_back(this->finger_locations_to_finger_idx(locs));

      probabilities->push_back(
          this->finger_idx_prob(finger_idxs->back(), t_change));
    }
  }
  return;
}

void TASK::sample_level2_action(const TASK::State2 &state,
                                TASK::State2::Action &action)
{
  int t = this->select_finger_change_timestep(state);

  bool if_valid = false;

  long int finger_idx;

  int max_sample = 300;

  std::vector<long int> sampled_finger_idxes;
  std::vector<double> probs;

  // TODO: here sample finger idxes based on State2
  // TODO: change it to sample_actions
  this->sample_likely_feasible_finger_idx(state, t, max_sample,
                                          &sampled_finger_idxes, &probs);

  std::default_random_engine randgen;
  std::discrete_distribution<int> distribution{probs.begin(), probs.end()};

  for (int k_sample = 0; k_sample < sampled_finger_idxes.size(); k_sample++)
  {
    int ik_sample = distribution(randgen);
    finger_idx = sampled_finger_idxes[ik_sample];
    // finger_idx = sampled_finger_idxes[k_sample];

    // finger_idx =
    //     randi(this->m_task->get_number_of_robot_actions(node->m_state));

    State2::Action new_action = State2::Action(t, finger_idx);

    State2 new_state = state;
    new_state.do_action(new_action);
    new_state.is_valid =
        true; // make sure it is valid for every state in select_action
    new_state.t_max = -1;

    if ((new_state.timestep < 0) ||
        (new_state.timestep >= this->saved_object_trajectory.size()))
    {
      std::cout << "timestep is issue, debug here" << std::endl;
    }

    // is valid transition & valid for at least one timestep
    if (this->is_finger_valid(finger_idx, t))
    {
      if (this->is_valid_transition(new_state, state))
      {
        if_valid = true;
        break;
      }
    }
  }

  if (if_valid)
  {
    // std::cout << "select action " << finger_idx << " at timestep " << t
    // << std::endl;
    action = State2::Action(t, finger_idx);
  }
  else
  {
    // Set to no action
    action.timestep = -1;
  }
  return;
}

bool TASK::project_to_zero_contact_distance(const Vector7d &object_pose,
                                            Vector7d &projected_pose)
{

  Vector7d x = object_pose;
  std::vector<ContactPoint> envs;
  this->m_world->getObjectContacts(&envs, x);

  if (envs.size() == 0)
  {
    projected_pose = x;
    return false;
  }

  // std::cout << "Previous x_rand " << object_pose.transpose() << std::endl;

  // std::cout << "Project to zero contact distance" << std::endl;

  VectorXi mode_corr(envs.size());
  mode_corr.setZero();

  // Detects new contacts: project the object back to zero contact distance
  int iter_corr = 0;
  while (iter_corr < 20)
  {

    if (!ifNeedVelocityCorrection(mode_corr, envs))
    {
      projected_pose = x;
      return true;
    }
    Vector6d v_corr = VelocityCorrection(envs);
    if (v_corr.norm() > 0.02)
    {
      v_corr = v_corr / v_corr.norm() * 0.02;
    }
    x = SE32pose(pose2SE3(x) * se32SE3(v_corr));
    envs.clear();
    this->m_world->getObjectContacts(&envs, x);
    iter_corr++;
  }
  projected_pose = x;
  return false;
}

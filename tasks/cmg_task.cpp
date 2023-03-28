#include "cmg_task.h"
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

int CMGTASK::neighbors_on_the_same_manifold(const Vector7d &q,
                                            std::vector<ContactPoint> envs,
                                            std::vector<VectorXi> env_modes,
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
      if_feasible = this->robot_contact_feasibile_check(
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

bool CMGTASK::forward_integration(const Vector7d &x_start,
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
      bool pass_pruning_check = this->robot_contact_feasibile_check(
          selected_finger_idx, x, env_mode.head(envs.size()), v_b, envs);
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
        // std::cout << "velocity correction failed! "
        //           << "At counter" << counter << std::endl;

        // visualize the traj
        // path->push_back(x_new);
        // this->m_world->setObjectTrajectory(*path);
        // char *aa;
        // int a = 1;
        // this->m_world->startWindow(&a, &aa);
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

// -----------------------------------------------------------
// CMGTASK

void CMGTASK::initialize(
    const Vector7d &start_object_pose, const Vector7d &goal_object_pose,
    double goal_thr, double wa, double wt, double charac_len, double mu_env,
    double mu_mnp, Matrix6d object_inertia, Vector6d f_gravity,
    std::shared_ptr<WorldTemplate> world, int n_robot_contacts,
    int dynamic_type, std::vector<ContactPoint> surface_pts,
    const SearchOptions &options, bool if_refine, double refine_dist)
{

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
  this->number_of_robot_contacts = n_robot_contacts;
  this->task_dynamics_type = dynamic_type;
  this->object_surface_pts = surface_pts;
  this->if_refine = if_refine;
  this->refine_dist = refine_dist;

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
}

CMGTASK::State CMGTASK::generate_state(const Vector7d &object_pose) const
{
  CMGTASK::State state_;
  state_.m_pose = object_pose;
  this->m_world->getObjectContacts(&state_.envs, object_pose);
  cs_mode_enumeration(*this->cons.get(), state_.envs, &state_.modes);

  return state_;
}

std::vector<CMGTASK::State>
CMGTASK::search_a_new_path(const CMGTASK::State &start_state)
{
  // search a new path towards the end, given the START_STATE and M_MODE_IDX!!!

  // during the search, it figure out the constriants (modes) for the states
  // for now just return linear interpolation towards the sampled state

  // every state need to be associated with m_mode_idx (mode to this state)

  std::vector<CMGTASK::State> path_;

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

      // if (cs_mode.size() >= 4) {
      //   if ((cs_mode[0] == 0) && (cs_mode[1] == 0) && (cs_mode[2] == 1) &&
      //       (cs_mode[3] == 1)) {
      //     std::cout << "debug here " << std::endl;
      //   }
      // }

      // extend cs_mode with all sticking mode and free sliding mode

      // check for force feasibilitiy of this cs_mode with all possible
      // direction of friction forces

      std::vector<VectorXi> mode_to_extend;
      {
        VectorXi all_sticking_mode(3 * cs_mode.size());
        all_sticking_mode.setZero();
        all_sticking_mode.block(0, 0, cs_mode.size(), 1) = cs_mode;

        Vector6d v = EnvironmentConstrainedVelocity(
            v_star, shared_rrt->nodes[near_idx].envs, all_sticking_mode,
            *this->cons);

        if (v.norm() > 1e-6)
        {
          bool is_pass_pruning =
              (this->pruning_check(shared_rrt->nodes[near_idx].config, cs_mode,
                                   v, shared_rrt->nodes[near_idx].envs) != -1);

          if (is_pass_pruning)
          {
            mode_to_extend.push_back(all_sticking_mode);
          }
        }
      }

      {
        // extend this cs_mode in free sliding way
        Vector6d v = EnvironmentConstrainedVelocity_CSModeOnly(
            v_star, shared_rrt->nodes[near_idx].envs, cs_mode, *this->cons);

        if (v.norm() > 1e-6)
        {

          bool is_pass_pruning =
              (this->pruning_check(shared_rrt->nodes[near_idx].config, cs_mode,
                                   v, shared_rrt->nodes[near_idx].envs) != -1);
          if (is_pass_pruning)
          {
            mode_to_extend.push_back(cs_mode);
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
      CMGTASK::State new_state(shared_rrt->nodes[kn].config,
                               shared_rrt->nodes[kn].envs, mode_idx,
                               shared_rrt->nodes[kn].modes);
      new_state.m_path = shared_rrt->edges[shared_rrt->nodes[kn].edge].path;

      path_.push_back(new_state);
    }
    CMGTASK::State new_state(shared_rrt->nodes[node_path.back()].config,
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

bool CMGTASK::is_valid(const CMGTASK::State2 &state, const State2 &prev_state)
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
      // this->m_world->getRobot()->getFingertipsOnObject(mnp_config, x_object,
      //                                                  &fingertips);
      this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
    }
  }

  bool dynamic_feasibility;
  Vector6d v = compute_rbvel_body(x_object, x_object_now);

  if (this->task_dynamics_type == CMG_QUASISTATIC)
  {

    dynamic_feasibility = isQuasistatic(
        mnps, this->saved_object_trajectory[pre_timestep].envs,
        mode_from_velocity(v, this->saved_object_trajectory[pre_timestep].envs,
                           this->cons.get()),
        this->f_gravity, x_object, this->mu_env, this->mu_mnp,
        this->cons.get());
  }
  else if (this->task_dynamics_type == CMG_QUASIDYNAMIC)
  {
    double h_time = 1.0;

    VectorXi env_mode_full = mode_from_velocity(
        v, this->saved_object_trajectory[pre_timestep].envs, this->cons.get());

    dynamic_feasibility = isQuasidynamic(
        v, mnps, this->saved_object_trajectory[pre_timestep].envs,
        env_mode_full, this->f_gravity, this->object_inertia, x_object,
        this->mu_env, this->mu_mnp, this->wa, this->wt, h_time,
        this->cons.get(), 0.5);
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

double CMGTASK::total_finger_change_ratio(const std::vector<State2> &path)
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
double CMGTASK::evaluate_path(const std::vector<State2> &path)
{

  if (!path.back().is_valid)
  {
    return 0.0;
  }

  double total_finger_changes = this->total_finger_change_ratio(path);

  // double reward = double(node->m_state.t_max) + 1.0 - total_finger_changes;

  double x = total_finger_changes / double(path.back().t_max);
  double y = 10.80772595 * x + -4.59511985;
  double reward = 1.0 / (1.0 + std::exp(y));

  if (this->grasp_measure_charac_length <= 0.0)
  {
    return reward;
  }
  else
  {
    reward *= 0.5;

    double avg_grasp_d = 0.0;
    for (auto s2 : path)
    {
      if (s2.finger_index == -1 || s2.timestep == -1)
      {
        continue;
      }
      double grasp_d = this->grasp_measure(s2.finger_index, s2.timestep);
      avg_grasp_d += grasp_d;
    }
    double x_grasp = avg_grasp_d / (double(path.size()) - 1);

    double y_grasp = 6.90675 * x_grasp - 6.90675;
    double reward_grasp = 1.0 / (1.0 + std::exp(y_grasp));

    reward += 0.5 * reward_grasp;
  }

  return reward;
}

void CMGTASK::save_trajectory(const std::vector<CMGTASK::State> &path)
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
      this->saved_object_trajectory.push_back(state);
    }
  }

  for (int i = 0; i < this->saved_object_trajectory.size(); ++i)
  {
    this->m_world->getObjectContacts(&(this->saved_object_trajectory[i].envs),
                                     this->saved_object_trajectory[i].m_pose);
  }
}

std::vector<int> CMGTASK::get_finger_locations(long int finger_location_index)
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

bool CMGTASK::robot_contact_feasibile_check(
    long int finger_idx, const Vector7d &x, const VectorXi &cs_mode,
    const Vector6d &v, const std::vector<ContactPoint> &envs)
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

  if (this->task_dynamics_type == CMG_QUASISTATIC)
  {

    dynamic_feasibility =
        isQuasistatic(mnps, envs, env_mode, this->f_gravity, x, this->mu_env,
                      this->mu_mnp, this->cons.get());
  }
  else if (this->task_dynamics_type == CMG_QUASIDYNAMIC)
  {
    double h_time = 1.0;

    dynamic_feasibility =
        isQuasidynamic(v, mnps, envs, env_mode, this->f_gravity,
                       this->object_inertia, x, this->mu_env, this->mu_mnp,
                       this->wa, this->wt, h_time, this->cons.get(), 0.5);
  }

  return dynamic_feasibility;
}

long int CMGTASK::pruning_check(const Vector7d &x, const Vector6d &v,
                                const std::vector<ContactPoint> &envs)
{

  VectorXi env_mode = mode_from_velocity(v, envs, this->cons.get());

  bool dynamic_feasibility = false;
  int max_sample = 100;
  // TODO: calculate n_finger_combination during initialization
  long int finger_idx;
  max_sample = (max_sample > this->n_finger_combinations)
                   ? this->n_finger_combinations
                   : max_sample;

  for (int k_sample = 0; k_sample < max_sample; k_sample++)
  {
    finger_idx = randi(this->n_finger_combinations);
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
        // this->m_world->getRobot()->getFingertipsOnObject(mnp_config,
        // x_object,
        //                                                  &fingertips);
        this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
      }
    }

    if (this->task_dynamics_type == CMG_QUASISTATIC)
    {

      dynamic_feasibility = isQuasistatic(
          mnps, envs, mode_from_velocity(v, envs, this->cons.get()),
          this->f_gravity, x, this->mu_env, this->mu_mnp, this->cons.get());
    }
    else if (this->task_dynamics_type == CMG_QUASIDYNAMIC)
    {
      double h_time = 1.0;
      dynamic_feasibility =
          isQuasidynamic(v, mnps, envs, env_mode, this->f_gravity,
                         this->object_inertia, x, this->mu_env, this->mu_mnp,
                         this->wa, this->wt, h_time, this->cons.get(), 0.5);
    }

    if (dynamic_feasibility)
    {
      break;
    }
  }

  if (dynamic_feasibility)
  {
    return finger_idx;
  }
  else
  {
    return -1;
  }

  return dynamic_feasibility;
}

long int CMGTASK::pruning_check(const Vector7d &x, const VectorXi &cs_mode,
                                const Vector6d &v,
                                const std::vector<ContactPoint> &envs)
{

  VectorXi env_mode = mode_from_velocity(v, envs, this->cons.get());
  env_mode.head(envs.size()) = cs_mode;

  bool dynamic_feasibility = false;
  int max_sample = 100;
  // TODO: calculate n_finger_combination during initialization

  // max_sample = (max_sample > this->n_finger_combinations)
  //                  ? this->n_finger_combinations
  //                  : max_sample;
  long int finger_idx;
  for (int k_sample = 0; k_sample < max_sample; k_sample++)
  {

    if (max_sample > this->n_finger_combinations)
    {
      if (k_sample >= this->n_finger_combinations)
      {
        break;
      }
      finger_idx = k_sample;
    }
    else
    {
      finger_idx = randi(this->n_finger_combinations);
    }
    std::vector<ContactPoint> mnps;
    std::vector<int> fingertip_idx;

    // // debug
    // fingertip_idx = this->get_finger_locations(finger_idx);
    // std::cout << "finger_idx " << finger_idx << std::endl;
    // std::cout << "idx ";
    // for (auto i : fingertip_idx)
    // {
    //   std::cout << i << " ";
    // }
    // std::cout << std::endl;

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

      if (this->m_world->isRobotCollide(mnp_config))
      {
        continue;
      }
      // if the robot collides, not valid
      // bool is_collide = this->m_world->isRobotCollide(mnp_config);
      // if (is_collide) {
      //   continue;
      // }

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
        // this->m_world->getRobot()->getFingertipsOnObject(mnp_config,
        // x_object,
        //                                                  &fingertips);
        this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
      }
    }

    if (this->task_dynamics_type == CMG_QUASISTATIC)
    {

      dynamic_feasibility =
          isQuasistatic(mnps, envs, env_mode, this->f_gravity, x, this->mu_env,
                        this->mu_mnp, this->cons.get());
    }
    else if (this->task_dynamics_type == CMG_QUASIDYNAMIC)
    {
      double h_time = 1.0;

      dynamic_feasibility =
          isQuasidynamic(v, mnps, envs, env_mode, this->f_gravity,
                         this->object_inertia, x, this->mu_env, this->mu_mnp,
                         this->wa, this->wt, h_time, this->cons.get(), 0.5);
    }

    if (dynamic_feasibility)
    {
      break;
    }
  }

  if (dynamic_feasibility)
  {
    return finger_idx;
  }
  else
  {
    return -1;
  }
}

int CMGTASK::max_forward_timestep(const CMGTASK::State2 &state)
{
  // select a timestep to change the finger configuration

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

int CMGTASK::select_finger_change_timestep(const CMGTASK::State2 &state)
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

bool CMGTASK::is_finger_valid(long int finger_idx, int timestep)
{
  // check if the finger is valid to move one timestep forward

  // check for the validity of timestep and timestep+1

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
    // this->m_world->getRobot()->getFingertipsOnObject(mnp_config,
    // x_object,
    //                                                  &fingertips);
    this->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
  }

  bool dynamic_feasibility;

  if ((this->task_dynamics_type == CMG_QUASISTATIC) || v.norm() < 1e-6)
  {

    VectorXi conservative_full_mode = conservative_mode_from_velocity(
        this->saved_object_trajectory[timestep].envs, reference_cs_mode, v,
        this->cons.get());

    dynamic_feasibility =
        isQuasistatic(mnps, this->saved_object_trajectory[timestep].envs,
                      conservative_full_mode, this->f_gravity, x_object,
                      this->mu_env, this->mu_mnp, this->cons.get());
  }
  else if (this->task_dynamics_type == CMG_QUASIDYNAMIC)
  {
    double h_time = 1.0;

    VectorXi env_mode = mode_from_velocity(
        v, this->saved_object_trajectory[timestep].envs, this->cons.get());
    env_mode.head(this->saved_object_trajectory[timestep].envs.size()) =
        reference_cs_mode;

    dynamic_feasibility = isQuasidynamic(
        v, mnps, this->saved_object_trajectory[timestep].envs, env_mode,
        this->f_gravity, this->object_inertia, x_object, this->mu_env,
        this->mu_mnp, this->wa, this->wt, h_time, this->cons.get(), 0.5);
  }

  return dynamic_feasibility;
}

bool CMGTASK::is_valid_transition(const CMGTASK::State2 &state,
                                  const CMGTASK::State2 &prev_state)
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

    if (this->task_dynamics_type == CMG_QUASISTATIC)
    {
      dynamic_feasibility = isQuasistatic(
          remain_mnps, this->saved_object_trajectory[state.timestep].envs,
          ss_mode_relocate, this->f_gravity,
          this->saved_object_trajectory[state.timestep].m_pose, this->mu_env,
          this->mu_mnp, this->cons.get());
    }
    else if (this->task_dynamics_type == CMG_QUASIDYNAMIC)
    {
      if (state.timestep == this->saved_object_trajectory.size() - 1)
      {
        return true;
      }
      Vector6d v_b = compute_rbvel_body(
          this->saved_object_trajectory[state.timestep].m_pose,
          this->saved_object_trajectory[state.timestep + 1].m_pose);
      dynamic_feasibility = isQuasidynamic(
          v_b, remain_mnps, this->saved_object_trajectory[state.timestep].envs,
          ss_mode_relocate, this->f_gravity, this->object_inertia,
          this->saved_object_trajectory[state.timestep].m_pose, this->mu_env,
          this->mu_mnp, this->wa, this->wt, 1.0, this->cons.get(), 0.0);
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

std::vector<double>
CMGTASK::get_path_features(const std::vector<State> &object_path,
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
    else if (feature_name == "average_finger_change_ratio")
    {

      double total_finger_changes =
          this->total_finger_change_ratio(robot_contact_path);

      x = total_finger_changes / double(robot_contact_path.back().t_max);
    }
    else if (feature_name == "total_finger_change_ratio")
    {

      double total_finger_changes =
          this->total_finger_change_ratio(robot_contact_path);

      x = total_finger_changes;
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
    else if (feature_name == "distance_to_goal_fingertips")
    {
      // TODO: to implement
      x = 0.0;
    }
    else
    {
      std::cout << "Error in get_path_features: feature name not found"
                << std::endl;
      exit(0);
    }
    features.push_back(x);
  }

  return features;
}
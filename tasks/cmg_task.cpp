#include "cmg_task.h"
#include "../mechanics/contacts/contact_kinematics.h"
#include "../mechanics/utilities/eiquadprog.hpp"

// ---------------------------
// Utility functions

double dist_vel(const Vector6d &v, const Vector6d &v0, double wt, double wa) {
  double d = wt * (v - v0).block(0, 0, 3, 1).norm() +
             wa * (v - v0).block(3, 0, 3, 1).norm();
  return d;
}

Vector6d weight_w2o(const Vector7d &x, const Vector6d &f_ext_w) {
  Matrix4d T = pose2SE3(x);
  Matrix6d Adg = SE32Adj(T);

  Matrix4d T_;
  T_.setIdentity();
  T_.block(0, 0, 3, 3) = T.block(0, 0, 3, 3);
  Vector6d f_ext_o = SE32Adj(T_).transpose() * f_ext_w;
  return f_ext_o;
}

void copy_pts(const std::vector<ContactPoint> &pts,
              std::vector<ContactPoint> *pts_new) {
  for (auto &pt : pts) {
    pts_new->push_back(pt);
  }
}

Vector7d steer_config(Vector7d x_near, Vector7d x_rand,
                      double epsilon_translation, double epsilon_angle) {

  // double epsilon_translation = 1.5;
  // double epsilon_angle = 3.14*100/180;

  Vector3d p_rand = x_rand.head(3);
  Vector3d p_near = x_near.head(3);
  Quaterniond q_near(x_near[6], x_near[3], x_near[4], x_near[5]);
  Quaterniond q_rand(x_rand[6], x_rand[3], x_rand[4], x_rand[5]);
  p_rand = steer_position(p_near, p_rand, epsilon_translation);
  q_rand = steer_quaternion(q_near, q_rand, epsilon_angle);
  Vector7d x_steer;
  x_steer << p_rand[0], p_rand[1], p_rand[2], double(q_rand.x()),
      double(q_rand.y()), double(q_rand.z()), double(q_rand.w());
  return x_steer;
}
bool ifConstraintsSatisfied(const VectorXd &x, const MatrixXd A,
                            const VectorXd &b, const MatrixXd G,
                            const VectorXd &h) {

  double Ax_b = (A * x - b).cwiseAbs().sum();
  if (Ax_b > 1e-3) {
    return false;
  }

  VectorXd g = G * x - h;
  for (int i = 0; i < g.size(); i++) {
    if (g[i] < -1e-3) {
      return false;
    }
  }

  return true;
}

void steer_velocity(Vector6d &x, double h, double cl = 1.0) {

  Vector6d xx = x;
  xx.head(3) = xx.head(3) * cl;
  if (xx.norm() > h) {
    xx = (h / xx.norm()) * xx;
    double xx_norm = xx.tail(3).norm();
    double x_norm = x.tail(3).norm();
    if (xx_norm > 1e-4 && x_norm > 1e-4) {
      x = x * (xx.tail(3).norm() / x.tail(3).norm());
    } else {
      x = xx / cl;
    }
  }
  return;
}

bool ifCollide(const std::vector<ContactPoint> &pts) {
  double thr = -0.04;
  for (const auto &pt : pts) {
    if (pt.d < thr) {
      return true;
    }
  }
  return false;
}

VectorXi deleteSeparatingMode(const VectorXi &mode) {

  VectorXi m(mode.size());
  int n_cpts = 0;
  int n_pts = mode.size() / 3;

  for (int i = 0; i < n_pts; i++) {
    int cs_mode = mode(i);
    if (cs_mode == 0) { // contacting
      n_cpts += 1;
    }
  }
  int k = 0;
  for (int i = 0; i < n_pts; i++) {
    int cs_mode = mode(i);
    if (cs_mode == 0) { // contacting
      m(k) = cs_mode;
      m.block(n_cpts + 2 * k, 0, 2, 1) = mode.block(n_pts + 2 * i, 0, 2, 1);
      k += 1;
    }
  }
  m.conservativeResize(3 * n_cpts);
  return m;
}

VectorXi deleteModebyRemainIndex(const VectorXi &mode,
                                 const VectorXi &remain_idx) {

  int n_cpts = remain_idx.size();
  int n_pts = int(mode.size() / 3);
  VectorXi m(3 * n_cpts);
  for (int i = 0; i < n_cpts; i++) {
    m[i] = mode[remain_idx[i]];
    m.block(n_cpts + 2 * i, 0, 2, 1) =
        mode.block(n_pts + 2 * remain_idx[i], 0, 2, 1);
  }
  return m;
}

bool ifContactingModeDeleted(const VectorXi &mode, const VectorXi &remain_idx) {
  for (int i = 0; i < int(mode.size() / 3); i++) {
    if (mode[i] == 0) {
      bool ifremained = false;
      for (int k = 0; k < remain_idx.size(); k++) {
        if (i == remain_idx[k]) {
          ifremained = true;
          break;
        }
      }
      if (!ifremained) {
        return true;
      }
    }
  }
  return false;
}

double CollisionInterpolation(const Vector6d &v,
                              const std::vector<ContactPoint> &pts) {

  double d_min = 0;
  Vector3d p;
  Vector3d n;

  for (const auto &pt : pts) {
    if (abs(pt.d) > abs(d_min)) {
      d_min = pt.d;
      p = pt.p;
      n = pt.n;
    }
  }
  Vector3d vel = v.block(0, 0, 3, 1);
  Vector3d omega = v.block(3, 0, 3, 1);
  Vector3d v_p_max;
  v_p_max = omega.cross(p) + vel;
  double k = 0;
  double a = (v_p_max.transpose() * n)(0);
  if (std::abs(a) >= std::abs(d_min)) {
    if (d_min > 0) {
      k = 1 + (std::abs(d_min) - 0.005) / std::abs(a);
    } else {
      k = 1 - (std::abs(d_min) - 0.005) / std::abs(a);
    }
  }
  return k;
}

bool ifNeedVelocityCorrection(VectorXi mode,
                              const std::vector<ContactPoint> &pts) {
  double thr = 0.03;
  for (int i = 0; i < pts.size(); i++) {

    if ((abs(pts[i].d) > thr) && mode[i] == 0) {
      return true;
    }
  }
  return false;
}

Vector6d VelocityCorrection(const std::vector<ContactPoint> &pts) {

  int n_pts = pts.size();
  double violation = -1e-4;

  Vector6d z_axis;
  z_axis << 0, 0, 1, 0, 0, 0;
  MatrixXd N(n_pts, 6);
  VectorXd d(n_pts);

  for (int i = 0; i < n_pts; i++) {
    Matrix6d Adgco = contact_jacobian(pts[i].p, pts[i].n);
    N.block(i, 0, 1, 6) = z_axis.transpose() * Adgco;

    d[i] = -(pts[i].d - violation);
  }

  Matrix6d I;
  I.setIdentity();
  Matrix6d G;
  G = N.transpose() * N + 0.001 * I;
  Vector6d g0;
  g0 = -2 * (d.transpose() * N).transpose();

  // MatrixXd A(6,0);
  // VectorXd b(0);
  // MatrixXd C(6,0);
  // VectorXd e(0);

  Vector6d x;
  x = -G.inverse() * g0 / 2;

  // double f = solve_quadprog(G, g0, A, b,  C, e, x);

  // if (f > 0.0){
  //     x.setZero();
  // }

  return x;
}

bool contactTrack(ContactPoint pt0, ContactPoint pt1) {
  if (((pt0.p - pt1.p).norm() < 0.1) &&
      ((pt0.n.transpose() * pt1.n)[0] > 0.85)) {
    return true;
  } else {
    // std::cout << "d: " << (pt0.p - pt1.p).norm() << " ";
    return false;
  }
}

VectorXi track_contacts_remain(const std::vector<ContactPoint> &pts,
                               const std::vector<ContactPoint> &pts_new) {
  VectorXi remain_idx(pts_new.size());
  int i = 0;
  int j = 0;
  while ((i < pts.size()) && (j < pts_new.size())) {
    if (contactTrack(pts[i], pts_new[j])) {
      remain_idx[j] = i;
      j++;
    }
    i++;
  }
  if (j < pts_new.size()) {
    VectorXi empty_idx(0);
    return empty_idx;
  }
  return remain_idx;
}

void deleteExtraContacts(const std::vector<ContactPoint> &pts0,
                         std::vector<ContactPoint> &pts) {
  std::vector<ContactPoint> pts2;
  VectorXi track(pts0.size());
  track.setZero();

  for (auto &pt : pts) {
    bool iftracked = false;
    for (int i = 0; i < pts0.size(); i++) {
      if (track[i] == 1)
        continue;
      ContactPoint pt0 = pts0[i];
      iftracked = contactTrack(pt0, pt);
      if (iftracked) {
        track[i] = 1;
        break;
      }
    }
    if (iftracked)
      pts2.push_back(pt);
  }
  pts = pts2;
}

bool simplify_line_contacts(const std::vector<ContactPoint> &pts,
                            std::vector<ContactPoint> *pts_update) {

  if (pts.size() <= 2) {
    return false;
  }

  double thr = 5e-2;
  Vector3d vec = pts[0].p - pts[1].p;
  vec = vec / vec.norm();
  int idx = 1;
  double d = vec.norm();
  // check if pts are in the same line
  for (int i = 2; i < pts.size(); i++) {
    Vector3d vec1 = pts[i].p - pts[0].p;
    vec1 = vec1 / vec1.norm();
    double cross_product = vec.cross(vec1).norm();
    if (cross_product > thr) {
      // not in the same line
      return false;
    } else {
      double dd = (pts[i].p - pts[0].p).norm();
      if (dd > d) {
        d = dd;
        idx = i;
      }
    }
  }
  pts_update->push_back(pts[0]);
  pts_update->push_back(pts[idx]);
  return true;
}

bool same_line_update(const std::vector<ContactPoint> &pts,
                      const std::vector<ContactPoint> &pts_new,
                      std::vector<ContactPoint> *pts_update) {

  if (pts.size() != 2) {
    return false;
  }

  double thr = 1e-3;
  Vector3d vec = pts[0].p - pts[1].p;
  double d = vec.norm();
  ContactPoint pt = pts[1];
  // check if pts_new are in the same line
  for (int i = 0; i < pts_new.size(); i++) {
    if ((vec.cross(pts_new[i].p - pts[0].p)).norm() > thr) {
      // not in the same line
      return false;
    } else {
      double dd = (pts_new[i].p - pts[0].p).norm();
      if (dd > d) {
        d = dd;
        pt = pts_new[i];
      }
    }
  }
  pts_update->push_back(pts[0]);
  pts_update->push_back(pt);
  return true;
}

Vector6d recoverContactingContacts(const std::vector<ContactPoint> &pts,
                                   const VectorXi &mode,
                                   const VectorXi &remain_idx) {
  std::vector<ContactPoint> envs;
  for (int i = 0; i < int(mode.size() / 3); i++) {
    if (mode[i] == 0) {
      bool ifremained = false;
      for (int k = 0; k < remain_idx.size(); k++) {
        if (i == remain_idx[k]) {
          ifremained = true;
        }
      }
      if (!ifremained) {
        envs.push_back(pts[i]);
        envs.back().d = 0.042;
      }
    }
  }
  for (int k = 0; k < remain_idx.size(); k++) {
    envs.push_back(pts[remain_idx[k]]);
  }
  return VelocityCorrection(envs);
}

Vector6d EnvironmentConstrainedVelocity(const Vector6d &v_goal,
                                        const std::vector<ContactPoint> &envs,
                                        const VectorXi &env_mode,
                                        ContactConstraints &cons) {

  MatrixXd A_env;
  MatrixXd G_env;
  VectorXd b_env;
  VectorXd h_env;

  cons.ModeConstraints(envs, env_mode, 0.2, Vector6d::Zero(), &A_env, &b_env,
                       &G_env, &h_env);

  MatrixXd A;
  MatrixXd G;
  VectorXd b;
  VectorXd h;

  // std::cout << "A_env\n" << A_env.block(0,0,A_env.rows(),6) << std::endl;
  // std::cout << "G_env\n" << G_env.block(0,0,G_env.rows(),6) << std::endl;

  deleteZeroRows(A_env.block(0, 0, A_env.rows(), 6), b_env, &A, &b);
  deleteZeroRows(G_env.block(0, 0, G_env.rows(), 6), h_env, &G, &h);

  // std::cout << "A\n" << A << std::endl;
  // std::cout << "G\n" << G << std::endl;

  mergeDependentRows(A, b, &A, &b);

  int n_var = 6;

  MatrixXd P(6, 6);
  VectorXd p(6);
  P.setIdentity();
  p = -v_goal;

  VectorXd x(n_var);
  // x.setZero();
  x = v_goal;

  if (A.rows() > n_var) {
    FullPivLU<MatrixXd> lu_decomp(A.transpose());

    if (lu_decomp.rank() >= n_var) {
      // if A fully constrainted the velocity
      x.setZero();
      // double f = solve_quadprog(P, p, A.transpose(), -b,  G.transpose(), -h,
      // x);
      return x;
    } else {
      A = (lu_decomp.image(A.transpose())).transpose();
      b = VectorXd::Zero(A.rows());
    }
  }

  double f = solve_quadprog(P, p, A.transpose(), -b, G.transpose(), -h, x);
  // std::cout << "x: \n" << x << std::endl;

  // if (std::isinf(f) || (!ifConstraintsSatisfied(x, A, b, G, h)) || f > 0.0){
  // // if fail to solve the problem
  //     x.setZero();
  // }
  if (std::isinf(f) || (!ifConstraintsSatisfied(
                           x, A, b, G, h))) { // if fail to solve the problem
    x.setZero();
  }

  // if(f > 0.0){
  //     std::cout << "solve_quadprog error" << std::endl;
  // }

  return x;
}

bool CMGTASK::forward_integration(const Vector7d &x_start,
                                  const Vector7d &x_goal,
                                  const std::vector<ContactPoint> &envs_,
                                  const VectorXi &env_mode_,
                                  std::vector<Vector7d> *path) {

  std::cout << "Forward integration from: " << x_start.transpose() << std::endl;

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

  Vector6d v_b_pre;
  v_b_pre.setZero();

  int counter;
  int delete_c = 0;

  for (counter = 0; counter < max_counter; counter++) {
    Vector6d v_star = compute_rbvel_body(x, x_goal);

    if (v_star.norm() < thr) {
      std::cout << "v_star < thr : " << v_star.transpose() << std::endl;
      break;
    }

    Matrix4d T = pose2SE3(x);
    Matrix6d Adg = SE32Adj(T);

    Matrix4d T_;
    T_.setIdentity();
    T_.block(0, 0, 3, 3) = T.block(0, 0, 3, 3);

    Vector6d v_b =
        EnvironmentConstrainedVelocity(v_star, envs, env_mode, *this->cons);

    if (v_b.norm() < thr) {
      std::cout << "v_b < thr : " << v_b.transpose() << std::endl;
      break;
    }

    if ((v_b_pre.transpose() * v_b)[0] < -1e-5) {
      printf("v_b back and forth. \n");
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
    if (envs.size() != 0 && envs.size() == env_mode.size() / 3 &&
        (ifNeedVelocityCorrection(env_mode, envs))) {
      // std::cout << "velocity correction "<< counter << std::endl;
      Vector6d v_corr = VelocityCorrection(envs);
      x_new = SE32pose(pose2SE3(x_new) * se32SE3(v_corr));
      envs.clear();
      this->m_world->getObjectContacts(&envs, x_new);
    }

    if (envs.size() > env_mode.size() / 3) {
      Vector6d v_corr = VelocityCorrection(envs);
      x_new = SE32pose(pose2SE3(x_new) * se32SE3(v_corr));
      ;
      path->push_back(x_new);
      x = x_new;
      envs.clear();
      this->m_world->getObjectContacts(&envs, x_new);
      printf("Made new contacts! \n");
      print_contacts(envs);
      break;
    }

    // update contact mode if needed
    if (envs.size() < env_mode.size() / 3) {
      VectorXi remain_idx = track_contacts_remain(envs_pre, envs);
      if (envs.size() != 0 && remain_idx.size() == 0) {
        printf("contact track fails.\n");
        break;
      }
      if (ifContactingModeDeleted(env_mode, remain_idx)) {
        if (h < 0.004 / 3) {
          // TODO: need to fix this
          delete_c++;
          if (delete_c > 4)
            break;
          printf("Collision Detection delelte contacting mode \n");
          Vector6d v_corr =
              recoverContactingContacts(envs_pre, env_mode, remain_idx);
          x_new = SE32pose(pose2SE3(x_new) * se32SE3(v_corr));
          envs.clear();
          this->m_world->getObjectContacts(&envs, x_new);
          if (envs.size() <= env_mode.size() / 3)
            continue;
          else
            break;
        }
        h = h / 1.5;
        envs = envs_pre;
        continue;
      } else {
        env_mode = deleteModebyRemainIndex(env_mode, remain_idx);
      }
    }

    x = x_new;
    envs_pre = envs;
    v_b_pre = v_b;

    path->push_back(x);

    if (counter == max_counter - 1) {
      printf("Reach the end.\n");
    }
  }

  std::cout << "counter:" << counter << " x: " << x.transpose() << std::endl;

  return true;
}

void enumerate_cs_modes(ContactConstraints &cons,
                        const std::vector<ContactPoint> &envs,
                        std::vector<VectorXi> *modes) {
  // contacting-separating mode enumeration
  MatrixXd A;
  VectorXd b;
  MatrixXd D;
  VectorXd d;
  cons.NormalVelocityConstraints(envs, &A, &b);
  cons.TangentVelocityConstraints(envs, &D, &d);

  if (envs.size() == 0) {
    VectorXi m(0);
    modes->push_back(m);
  } else {
    cs_mode_enumeration(A, modes);
  }
}

void enumerate_ss_modes(ContactConstraints &cons,
                        const std::vector<ContactPoint> &envs,
                        const VectorXi &cs_mode,
                        std::vector<VectorXi> *ss_modes) {

  MatrixXd A;
  VectorXd b;
  MatrixXd D;
  VectorXd d;
  cons.NormalVelocityConstraints(envs, &A, &b);
  cons.TangentVelocityConstraints(envs, &D, &d);
  ss_mode_enumeration(A, D, cs_mode, ss_modes);
}

// -----------------------------------------------------------
// CMGTASK

CMGTASK::State CMGTASK::generate_state(const Vector7d &object_pose) const {
  CMGTASK::State state_;
  state_.m_pose = object_pose;
  this->m_world->getObjectContacts(&state_.envs, object_pose);
  enumerate_cs_modes(*this->cons.get(), state_.envs, &state_.modes);

  return state_;
}

std::vector<CMGTASK::State>
CMGTASK::search_a_new_path(const CMGTASK::State &start_state) {
  // search a new path towards the end, given the START_STATE and M_MODE_IDX!!!

  // during the search, it figure out the constriants (modes) for the states
  // for now just return linear interpolation towards the sampled state

  // every state need to be associated with m_mode_idx (mode to this state)

  // int action_idx = start_state.m_mode_idx;

  // initialize a tree
  RRTTree rrt_tree(this->wa, this->charac_len * this->wt);
  RRTTree::Node start_node(start_state.m_pose);
  start_node.modes.push_back(start_state.modes[start_state.m_mode_idx]);
  start_node.envs = start_state.envs;
  rrt_tree.initial_node(&start_node);

  set_rand_seed();

  int goal_idx = -1;

  // -------

  for (int kk = 0; kk < this->search_options.max_samples; kk++) {
    std::cout << "rrt iter: " << kk << std::endl;

    // bias sample toward the goal
    Vector7d x_rand;
    int near_idx;
    // if ((randd() > this->search_options.goal_biased_prob) && (kk >= 1)) {
    if (randd() > this->search_options.goal_biased_prob) {
      Vector3d p_rand;
      Quaterniond q_rand;
      p_rand =
          sample_position(this->search_options.x_ub, this->search_options.x_lb);

      q_rand = (this->search_options.sampleSO3)
                   ? generate_unit_quaternion()
                   : sample_rotation(this->search_options.sample_rotation_axis);

      x_rand << p_rand[0], p_rand[1], p_rand[2], q_rand.x(), q_rand.y(),
          q_rand.z(), q_rand.w();
      near_idx = rrt_tree.nearest_neighbor(x_rand);
    } else {
      x_rand = this->goal_object_pose;
      near_idx = rrt_tree.nearest_unextended_to_goal(this->goal_object_pose);
      if (near_idx != 0) {
        rrt_tree.nodes[near_idx].is_extended_to_goal = true;
      }
    }

    // -----------------------------------
    // extend all cs modes, best ss modes

    // steer goal

    // move
    x_rand = steer_config(rrt_tree.nodes[near_idx].config, x_rand,
                          this->search_options.eps_trans,
                          this->search_options.eps_angle);

    // move
    Vector6d v_star =
        compute_rbvel_body(rrt_tree.nodes[near_idx].config, x_rand);

    // move
    Vector6d f_o = weight_w2o(rrt_tree.nodes[near_idx].config, this->f_gravity);

    // contact mode enumeration
    if (rrt_tree.nodes[near_idx].modes.size() == 0) {
      this->m_world->getObjectContacts(&(rrt_tree.nodes[near_idx].envs),
                                       rrt_tree.nodes[near_idx].config);
      enumerate_cs_modes(*this->cons.get(), rrt_tree.nodes[near_idx].envs,
                         &rrt_tree.nodes[near_idx].modes);
    }

    // for every mode do forward integration
    Vector6d v_zero = Vector6d::Zero();
    // move
    double d_zero = dist_vel(v_zero, v_star, rrt_tree.translation_weight,
                             rrt_tree.angle_weight);
    for (const auto &cs_mode : rrt_tree.nodes[near_idx].modes) {

      std::cout << "cs mode " << cs_mode.transpose() << std::endl;

      std::vector<VectorXi> modes;

      enumerate_ss_modes(*this->cons.get(), rrt_tree.nodes[near_idx].envs,
                         cs_mode, &modes);

      double dv_best = d_zero;

      VectorXi mode_best;

      std::vector<VectorXi> mode_to_extend;
      {
        VectorXi all_sticking_mode(3 * cs_mode.size());
        all_sticking_mode.setZero();
        all_sticking_mode.block(0, 0, cs_mode.size(), 1) = cs_mode;
        // move
        Vector6d v = EnvironmentConstrainedVelocity(
            v_star, rrt_tree.nodes[near_idx].envs, all_sticking_mode,
            *this->cons);

        if (dist_vel(v, v_star, rrt_tree.translation_weight,
                     rrt_tree.angle_weight) < d_zero) {
          mode_to_extend.push_back(all_sticking_mode);
        }
      }
      for (const auto &mode : modes) {
        // move
        Vector6d v = EnvironmentConstrainedVelocity(
            v_star, rrt_tree.nodes[near_idx].envs, mode, *this->cons);
        // std::cout << "mode: " << mode.transpose() <<  " velocity: " <<
        // v.transpose() << std::endl;
        if (dist_vel(v, v_star, rrt_tree.translation_weight,
                     rrt_tree.angle_weight) < dv_best) {
          dv_best = dist_vel(v, v_star, rrt_tree.translation_weight,
                             rrt_tree.angle_weight);
          mode_best = mode;
        }
      }
      std::cout << "best mode " << mode_best.transpose() << std::endl;

      if (dv_best < d_zero - 1e-4) {
        mode_to_extend.push_back(mode_best);
      }

      /// choose sliding mode end

      for (const auto &mode : mode_to_extend) {

        std::cout << "Extend mode: " << mode.transpose() << std::endl;

        std::vector<Vector7d> path;

        // move
        this->forward_integration(rrt_tree.nodes[near_idx].config, x_rand,
                                  rrt_tree.nodes[near_idx].envs, mode, &path);

        // if integration is successful
        if (path.size() > 1) {

          RRTTree::Node new_node(path.back());
          RRTTree::Edge new_edge(mode);

          rrt_tree.add_node(&new_node, near_idx, &new_edge);
        }
      }
    }
    //----------------------------------

    int goal_near_idx = rrt_tree.nearest_neighbor(this->goal_object_pose);
    if (rrt_tree.dist(rrt_tree.nodes[goal_near_idx].config,
                      this->goal_object_pose) <= goal_thr) {
      printf("Found goal node in %d samples. \n", kk + 1);
      goal_idx = goal_near_idx;
      break;
    }
  }

  bool ifsuccess = false;

  if (goal_idx != -1) {
    ifsuccess = true;
    printf("GOAL REACHED! \n");
  } else {
    ifsuccess = false;
    goal_idx = rrt_tree.nearest_neighbor(this->goal_object_pose);
    std::cout << "GOAL NOT REACHED. Dist: "
              << rrt_tree.dist(rrt_tree.nodes[goal_idx].config,
                               this->goal_object_pose)
              << ". Closest config: "
              << rrt_tree.nodes[goal_idx].config.transpose() << std::endl;
  }

  /// end of search

  std::vector<CMGTASK::State> path_;

  if (ifsuccess) {
    std::vector<int> node_path;
    rrt_tree.backtrack(goal_idx, &node_path);
    std::reverse(node_path.begin(), node_path.end());
    for (int k = 1; k < node_path.size() - 1; k++) {
      int kn = node_path[k];
      int k_child = node_path[k+1];
      VectorXi mode = rrt_tree.edges[rrt_tree.nodes[k_child].edge].mode;
      int mode_idx = -1;
      for (int idx = 0; idx < rrt_tree.nodes[kn].modes.size(); ++idx) {
        if ((mode.head(rrt_tree.nodes[kn].modes[idx].size()) - rrt_tree.nodes[kn].modes[idx]).norm() == 0) {
          mode_idx = idx;
          break;
        }
      }
      CMGTASK::State new_state(rrt_tree.nodes[kn].config,
                               rrt_tree.nodes[kn].envs, mode_idx,
                               rrt_tree.nodes[kn].modes);
      path_.push_back(new_state);
    }
    CMGTASK::State new_state(rrt_tree.nodes[node_path.back()].config,
                             rrt_tree.nodes[node_path.back()].envs, -1,
                             rrt_tree.nodes[node_path.back()].modes);
    path_.push_back(new_state);
  }

  return path_;
}

double CMGTASK::evaluate_path(const std::vector<CMGTASK::State> &path) const {
  // return the REWARD of the path: larger reward -> better path

  // TODO: define reward
  double reward = 1 / (double(path.size()-7)*double(path.size()-7)+1);

  return reward;
}
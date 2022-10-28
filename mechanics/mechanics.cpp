#include "mechanics.h"

#ifndef CONTACTKINEMATICS_H
#define CONTACTKINEMATICS_H
#include "contacts/contact_kinematics.h"
#endif

#ifndef CONTACTCONSTRAINTS_H
#define CONTACTCONSTRAINTS_H
#include "contacts/contact_constraints.h"
#endif

#include "utilities/eiquadprog.hpp"

bool force_balance(const std::vector<Vector6d> &mnp_contacts,
                   const std::vector<Vector6d> &env_contacts,
                   const VectorXi &env_mode, const Vector6d &f_ext_w,
                   const Vector7d object_pose, double mu_env, double mu_mnp) {

  if (mnp_contacts.size() + env_contacts.size() == 0) {
    return false;
  }

  Vector6d f_ext_o;
  {
    Matrix4d T = pose2SE3(object_pose);
    Matrix4d T_;
    T_.setIdentity();
    T_.block(0, 0, 3, 3) = T.block(0, 0, 3, 3);
    f_ext_o = SE32Adj(T_).transpose() * f_ext_w;
  }

  std::vector<ContactPoint> mnps;
  std::vector<ContactPoint> envs;

  for (auto cp : mnp_contacts) {
    mnps.push_back(ContactPoint(cp.head(3), cp.tail(3)));
  }
  for (auto cp : env_contacts) {
    envs.push_back(ContactPoint(cp.head(3), cp.tail(3)));
  }

  MatrixXd A_env;
  MatrixXd G_env;
  VectorXd b_env;
  VectorXd h_env;

  ContactConstraints cons(2);

  cons.ModeConstraints(envs, env_mode, mu_env, f_ext_o, &A_env, &b_env, &G_env,
                       &h_env);

  MatrixXd A_mnp;
  MatrixXd G_mnp;
  VectorXd b_mnp;
  VectorXd h_mnp;

  VectorXi mmode;
  mmode.resize(3 * mnps.size());
  mmode.setZero(); // all fixed manipulator contacts

  cons.ModeConstraints(mnps, mmode, mu_mnp, f_ext_o, &A_mnp, &b_mnp, &G_mnp,
                       &h_mnp);

  MatrixXd A;
  MatrixXd G;
  VectorXd b;
  VectorXd h;

  mergeManipulatorandEnvironmentConstraints(
      A_mnp, b_mnp, G_mnp, h_mnp, A_env, b_env, G_env, h_env, &A, &b, &G, &h);

  int n_var = A.cols() - 6;
  if (n_var == 0) {
    return false;
  }
  A = A.block(0, 6, A.rows(), n_var);
  G = G.block(0, 6, G.rows(), n_var);

  deleteZeroRows(A, b, &A, &b);
  deleteZeroRows(G, h, &G, &h);

  VectorXd
      xl; // = VectorXd::Constant(n_var, std::numeric_limits<double>::min());
  VectorXd
      xu; // = VectorXd::Constant(n_var, std::numeric_limits<double>::max());

  VectorXd C = VectorXd::Constant(n_var, 0);
  VectorXd x(n_var);
  x.setZero();
  double optimal_cost;

  bool result = lp(C, -G, -h, A, b, xl, xu, &x, &optimal_cost);

  return result;

  // VectorXd sigma = VectorXd::Constant(b.size(), 1e-4);
  // MatrixXd A_relax;
  // MatrixXd G_relax(G.rows() + 2 * A.rows(), n_var);
  // VectorXd b_relax;
  // VectorXd h_relax(G.rows() + 2 * A.rows());
  // h_relax.setZero();
  // G_relax.block(0, 0, G.rows(), n_var) = G;
  // G_relax.block(G.rows(), 0, A.rows(), n_var) = A;
  // G_relax.block(G.rows() + A.rows(), 0, A.rows(), n_var) = -A;
  // h_relax.block(G.rows(), 0, A.rows(), 1) = b - sigma;
  // h_relax.block(G.rows() + A.rows(), 0, A.rows(), 1) = -(b + sigma);

  // bool result_relax =
  //     lp(C, -G_relax, -h_relax, A_relax, b_relax, xl, xu, &x, &optimal_cost);

  // return result_relax;
}

bool quasistatic_check(const std::vector<ContactPoint> &mnps,
                       const std::vector<ContactPoint> &envs, const Vector6d v,
                       const Vector7d object_pose, const Vector6d &f_ext_w,
                       double mu_env, double mu_mnp) {

  if (f_ext_w.norm() == 0) {
    return true;
  }
  if (mnps.size() + envs.size() == 0) {
    return false;
  }

  Vector6d f_ext_o;
  {
    Matrix4d T = pose2SE3(object_pose);
    Matrix4d T_;
    T_.setIdentity();
    T_.block(0, 0, 3, 3) = T.block(0, 0, 3, 3);
    f_ext_o = SE32Adj(T_).transpose() * f_ext_w;
  }

  ContactConstraints cons(2);

  Eigen::Matrix<double, 3, 6> basis;
  basis.setZero();
  for (int i = 0; i < 3; i++) {
    basis(i, i) = 1;
  }

  double thr = 1e-4;

  MatrixXd A(6, 3 * (envs.size() + mnps.size()));
  Vector6d b;
  MatrixXd G(3 * (envs.size() + mnps.size()), 3 * (envs.size() + mnps.size()));
  MatrixXd G_fc(4 * (envs.size() + mnps.size()),
                3 * (envs.size() + mnps.size())); // friction constraints

  A.setZero();
  b = -f_ext_o;
  G.setIdentity();

  int n_pts = 0;
  int fc_counter = 0;

  MatrixXd pyramid_env(4, 3);
  pyramid_env << 1, 1, mu_env, -1, 1, mu_env, 1, -1, mu_env, -1, -1, mu_env;

  MatrixXd sliding_cone_env(2, 3);
  sliding_cone_env << 
  -1, -1, 1.01 * mu_env, //
      1, 1, -0.99 * mu_env;                  //

  MatrixXd pyramid_mnp(4, 3);
  pyramid_mnp << 1, 1, mu_mnp, //
      -1, 1, mu_mnp,           //
      1, -1, mu_mnp,           //
      -1, -1, mu_mnp;          //

  for (auto pt : envs) {

    // calculate contact velocity
    Matrix6d Adgco = contact_jacobian(pt.p, pt.n);
    Eigen::Matrix<double, 1, 6> N = basis.row(2) * Adgco;
    Eigen::Matrix<double, 2, 6> T = basis.block<2, 6>(0, 0) * Adgco;

    // skip this contact if its normal velocity > thr
    double vn = N * v;
    if (vn > thr) {
      continue;
    }

    Vector6d vt;
    vt.setZero();
    vt.block(0, 0, 2, 1) = T * v;

    Eigen::Matrix<double, 2, 6> T_cone;

    if (vt.norm() < thr) {
      // sticking contact
      T_cone = basis.block<2, 6>(0, 0);
      G(3 * n_pts, 3 * n_pts) = 0;
      G(3 * n_pts + 1, 3 * n_pts + 1) = 0;
      G_fc.block(fc_counter, 3 * n_pts, 4, 3) = pyramid_env;
      fc_counter += 4;
    } else {
      // sliding contact
      // compute its ss mode
      VectorXd vt_dir = cons.friction_cone->D * vt;
      VectorXi ss_mode(vt_dir.size());
      for (int i = 0; i < vt_dir.size(); ++i) {
        if (vt_dir[i] > thr) {
          ss_mode[i] = 1;
        } else if (vt_dir[i] < -thr) {
          ss_mode[i] = -1;
        } else {
          ss_mode[i] = 0;
        }
      }

      T_cone = -cons.friction_cone->getVectorsbyMode(ss_mode) * Adgco;
      G_fc.block(fc_counter, 3 * n_pts, 2, 3) = sliding_cone_env;
      fc_counter += 2;
    }

    // add this contact to
    A.block<6, 2>(0, 3 * n_pts) = T_cone.transpose();
    A.block<6, 1>(0, 3 * n_pts + 2) = N.transpose();

    n_pts++;
  }

  for (auto pt : mnps) {
    Matrix6d Adgco = contact_jacobian(pt.p, pt.n);
    A.block<6, 3>(0, 3 * n_pts) = (basis * Adgco).transpose();

    G_fc.block(fc_counter, 3 * n_pts, 4, 3) = pyramid_mnp;
    fc_counter += 4;

    G(3 * n_pts, 3 * n_pts) = 0;
    G(3 * n_pts + 1, 3 * n_pts + 1) = 0;
    n_pts++;
  }

  A.conservativeResize(6, 3 * n_pts);
  G.conservativeResize(3 * n_pts, 3 * n_pts);
  G_fc.conservativeResize(fc_counter, 3 * n_pts);

  MatrixXd G_all(G_fc.rows() + G.rows(), 3 * n_pts);
  G_all.block(0, 0, G.rows(), G.cols()) = G;
  G_all.block(G.rows(), 0, G_fc.rows(), G_fc.cols()) = G_fc;

  VectorXd h(G_all.rows());
  h.setZero();

  deleteZeroRows(G_all, h, &G_all, &h);

  int n_var = 3 * n_pts;

  VectorXd
      xl; // = VectorXd::Constant(n_var, std::numeric_limits<double>::min());
  VectorXd
      xu; // = VectorXd::Constant(n_var, std::numeric_limits<double>::max());

  VectorXd C = VectorXd::Constant(n_var, 0);
  VectorXd x(n_var);
  x.setZero();
  double optimal_cost;

  bool result = lp(C, -G_all, -h, A, b, xl, xu, &x, &optimal_cost);

  return result;
}

bool quasistatic_check_2(const std::vector<Vector6d> &mnps,
                       const std::vector<Vector6d> &envs, const Vector6d v,
                       const Vector7d object_pose, const Vector6d &f_ext_w,
                       double mu_env, double mu_mnp) {

  VectorXi env_mode(envs.size()*3);

  ContactConstraints cons(2);

  Eigen::Matrix<double, 3, 6> basis;
  basis.setZero();
  for (int i = 0; i < 3; i++) {
    basis(i, i) = 1;
  }

  double thr = 1e-4;

  for (int k = 0; k < envs.size(); ++k) {

    ContactPoint pt(envs[k].head(3), envs[k].tail(3));

    // calculate contact velocity
    Matrix6d Adgco = contact_jacobian(pt.p, pt.n);
    Eigen::Matrix<double, 1, 6> N = basis.row(2) * Adgco;
    Eigen::Matrix<double, 2, 6> T = basis.block<2, 6>(0, 0) * Adgco;

    // skip this contact if its normal velocity > thr
    double vn = N * v;
    if (vn > thr) {
      env_mode[k] = 1;
      env_mode[envs.size() + 2*k] = 0;
      env_mode[envs.size() + 2*k+1] = 0;
    }

    env_mode[k] = 0;

    Vector6d vt;
    vt.setZero();
    vt.block(0, 0, 2, 1) = T * v;

    
    if (vt.norm() < thr) {
      // sticking contact
      env_mode[envs.size() + 2*k] = 0;
      env_mode[envs.size() + 2*k+1] = 0;
    } else {
      // sliding contact
      // compute its ss mode
      VectorXd vt_dir = cons.friction_cone->D * vt;
      VectorXi ss_mode(vt_dir.size());
      for (int i = 0; i < vt_dir.size(); ++i) {
        if (vt_dir[i] > thr) {
          ss_mode[i] = 1;
        } else if (vt_dir[i] < -thr) {
          ss_mode[i] = -1;
        } else {
          ss_mode[i] = 0;
        }
      }

      env_mode[envs.size() + 2*k] = ss_mode[0];
      env_mode[envs.size() + 2*k+1] = ss_mode[1];

    }
  }
  std::cout << env_mode.transpose() << std::endl;
  bool result = force_balance(mnps, envs, env_mode, f_ext_w, object_pose, mu_env, mu_mnp);

  return result;
}
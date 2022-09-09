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

  VectorXd sigma = VectorXd::Constant(b.size(), 1e-4);
  MatrixXd A_relax;
  MatrixXd G_relax(G.rows() + 2 * A.rows(), n_var);
  VectorXd b_relax;
  VectorXd h_relax(G.rows() + 2 * A.rows());
  h_relax.setZero();
  G_relax.block(0, 0, G.rows(), n_var) = G;
  G_relax.block(G.rows(), 0, A.rows(), n_var) = A;
  G_relax.block(G.rows() + A.rows(), 0, A.rows(), n_var) = -A;
  h_relax.block(G.rows(), 0, A.rows(), 1) = b - sigma;
  h_relax.block(G.rows() + A.rows(), 0, A.rows(), 1) = -(b + sigma);

  bool result_relax =
      lp(C, -G_relax, -h_relax, A_relax, b_relax, xl, xu, &x, &optimal_cost);

  return result_relax;
}
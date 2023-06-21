#include "force_check.h"
#include "../mechanics/utilities/eiquadprog.hpp"

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


bool isQuasistatic(const std::vector<ContactPoint> &mnps,
                   const std::vector<ContactPoint> &envs,
                   const VectorXi &env_mode, const Vector6d &f_ext_w,
                   const Vector7d object_pose, double mu_env, double mu_mnp,
                   ContactConstraints *cons) {

  // force check

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

  MatrixXd A_env;
  MatrixXd G_env;
  VectorXd b_env;
  VectorXd h_env;

  cons->ModeConstraints(envs, env_mode, mu_env, f_ext_o, &A_env, &b_env, &G_env,
                        &h_env);

  MatrixXd A;
  MatrixXd G;
  VectorXd b;
  VectorXd h;

  if (mnps.size() > 0) {

    MatrixXd A_mnp;
    MatrixXd G_mnp;
    VectorXd b_mnp;
    VectorXd h_mnp;

    VectorXi mmode;
    mmode.resize(3 * mnps.size());
    mmode.setZero(); // all fixed manipulator contacts

    cons->ModeConstraints(mnps, mmode, mu_mnp, f_ext_o, &A_mnp, &b_mnp, &G_mnp,
                          &h_mnp);

    mergeManipulatorandEnvironmentConstraints(
        A_mnp, b_mnp, G_mnp, h_mnp, A_env, b_env, G_env, h_env, &A, &b, &G, &h);
  } else {
    A = A_env;
    b = b_env;
    G = G_env;
    h = h_env;
  }

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
}

bool isQuasidynamic(const Vector6d &v_b, const std::vector<ContactPoint> &mnps,
                    const std::vector<ContactPoint> &envs,
                    const VectorXi &env_mode, const Vector6d &f_ext_w,
                    const Matrix6d &object_inertia, const Vector7d object_pose,
                    double mu_env, double mu_mnp, double wa, double wt,
                    double h_time, ContactConstraints *cons, double thr) {

  // Will get strange numerical errors if the precision of envs.p is too high. Fix this by rounding the values to 6 decimal places.
  std::vector<ContactPoint> envs_round = envs;
  for (int i = 0; i < envs.size(); i++) {
    for(int k =0; k < 3; k++){
      envs_round[i].p[k] = round(envs_round[i].p[k] * 1000000) / 1000000;
    }
  }

  MatrixXd A_env;
  MatrixXd G_env;
  VectorXd b_env;
  VectorXd h_env;

  Vector6d f_ext_o;
  {
    Matrix4d T = pose2SE3(object_pose);
    Matrix4d T_;
    T_.setIdentity();
    T_.block(0, 0, 3, 3) = T.block(0, 0, 3, 3);
    f_ext_o = SE32Adj(T_).transpose() * f_ext_w;
  }

  cons->ModeConstraints(envs_round, env_mode, mu_env, f_ext_o, &A_env, &b_env, &G_env,
                        &h_env);

  MatrixXd A_mnp;
  MatrixXd G_mnp;
  VectorXd b_mnp;
  VectorXd h_mnp;

  VectorXi mmode;
  mmode.resize(3 * mnps.size());
  mmode.setZero(); // all fixed manipulator contacts

  cons->ModeConstraints(mnps, mmode, mu_mnp, f_ext_o, &A_mnp, &b_mnp, &G_mnp,
                        &h_mnp);

  MatrixXd A;
  MatrixXd G;
  VectorXd b;
  VectorXd h;

  mergeManipulatorandEnvironmentConstraints_relax(
      A_mnp, b_mnp, G_mnp, h_mnp, A_env, b_env, G_env, h_env, &A, &b, &G, &h);

  // add quasidynamic condition

  int n_var = A.cols();

  int G_rows = G.rows();
  if (G_rows < 12) {

    G.conservativeResize(12 + G_rows, n_var);
    h.conservativeResize(12 + G_rows);

    G.block(G_rows, 0, 12, n_var).setZero();
    h.block(G_rows, 0, 12, 1).setZero();

    VectorXd sigma = VectorXd::Constant(f_ext_o.size(), 1e-4);
    h.block(G_rows, 0, 6, 1) = -f_ext_o - sigma;
    h.block(G_rows + 6, 0, 6, 1) = -(-f_ext_o + sigma);
  }

  G.block(G.rows() - 12, 0, 6, 6) =
      -object_inertia * 1 / (h_time); // MI: object inertia
  G.block(G.rows() - 6, 0, 6, 6) = object_inertia * 1 / (h_time);

  // ------

  MatrixXd P(n_var, n_var);
  VectorXd p(n_var);
  p.setZero();
  P.setIdentity();
  P.block(0, 0, 3, 3) = wt * P.block(0, 0, 3, 3);
  P.block(3, 3, 3, 3) = wa * P.block(3, 3, 3, 3);
  P.block(6, 6, n_var - 6, n_var - 6) =
      0.01 * P.block(6, 6, n_var - 6, n_var - 6);
  p.block(0, 0, 6, 1) = -v_b;

  VectorXd x(n_var);
  x.setZero();

  Vector6d x_v;

  // std::cout << "f: " <<

  double f = solve_quadprog(P, p, A.transpose(), -b, G.transpose(), -h, x);

  if (std::isinf(f)) { // if fail to solve the problem
    return false;
  } else if (!ifConstraintsSatisfied(x, A, b, G, h)) {
    std::cout << " Constraints not satisfied for qp! " << std::endl;
    return false;
  } else {
    x_v = x.block(0, 0, 6, 1);
  }
  if (v_b.norm() < 1e-6) {
    if (x_v.norm() < 1e-3){
      return true;
    } 
  }
  Vector6d x_v_normalized = x_v.normalized();
  Vector6d v_b_normalized = v_b.normalized();
  double theta = x_v_normalized.transpose() * v_b_normalized;

  if (theta < thr) {
    // std::cout << "Solved v too large error for quasidynamic verification! Current theta: " << theta 
              // << std::endl;
    // std::cout << "x_v: " << x_v.transpose() << std::endl;
    // std::cout << "v_b: " << v_b.transpose() << std::endl;
    return false;
  }
  return true;
}

bool is_force_closure(Vector7d x, const std::vector<ContactPoint> &mnps,
                   double friction_coeff) {
  Matrix4d T_x = pose2SE3(x);

  Matrix3d R_wo = T_x.block(0, 0, 3, 3);

  // define polyhedral friction cone
  VectorXd f1_c(6, 1);
  f1_c << 0, -friction_coeff, 1, 0, 0, 0;
  VectorXd f2_c(6, 1);
  f2_c << 0, friction_coeff, 1, 0, 0, 0;
  VectorXd f3_c(6, 1);
  f3_c << -friction_coeff, 0, 1, 0, 0, 0;
  VectorXd f4_c(6, 1);
  f4_c << friction_coeff, 0, 1, 0, 0, 0;

  int num_of_contacts = mnps.size();

  // define the friction cone in contact frame and in object frame
  MatrixXd w_c(6, 4 * num_of_contacts);

  for (int j = 0; j < num_of_contacts; j++) {

    // find the finger position and norm from surface
    Vector3d fp_o;
    Vector3d fn_o;
    fp_o = mnps[j].p;
    fn_o = mnps[j].n;

    Vector3d fp_w = R_wo * fp_o + x.head(3);

    // get contact kinematics and transfer force in contact frame to object
    // frame

    MatrixXd adgco = contact_jacobian(fp_o, fn_o);

    VectorXd f1_o = adgco.transpose() * f1_c;
    VectorXd f2_o = adgco.transpose() * f2_c;
    VectorXd f3_o = adgco.transpose() * f3_c;
    VectorXd f4_o = adgco.transpose() * f4_c;

    w_c.col(j * 4) = f1_o;
    w_c.col(j * 4 + 1) = f2_o;
    w_c.col(j * 4 + 2) = f3_o;
    w_c.col(j * 4 + 3) = f4_o;
  }

  // std::cout << "w_c: "<< w_c << std::endl;

  // solve it by LP
  VectorXd sum_2_w_c = w_c.rowwise().sum();  // sum(w_c, 2)
  VectorXd avg_w_c = sum_2_w_c / w_c.cols(); // get average
  VectorXd T_0 = -avg_w_c;
  MatrixXd T(6, 4 * num_of_contacts);
  for (int i = 0; i < w_c.cols(); i++) {
    T.col(i) = w_c.col(i) - avg_w_c;
  }

  // check if T is full rank
  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(T);
  int rank_T = lu_decomp.rank();
  if (rank_T < 6) {
    // std::cout << "T is not full rank" << std::endl;
    return false;
  }

  // set up LP
  VectorXd C = -T_0;
  MatrixXd A = T.transpose();
  VectorXd b(w_c.cols());
  b.fill(1);

  MatrixXd Ae(w_c.cols(), 6);
  VectorXd be(w_c.cols());
  VectorXd xl(6);
  xl << std::numeric_limits<double>::min(), std::numeric_limits<double>::min(),
      std::numeric_limits<double>::min(), std::numeric_limits<double>::min(),
      std::numeric_limits<double>::min(), std::numeric_limits<double>::min();
  VectorXd xu(6);
  xu << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max();
  VectorXd xs(6);
  double optimal_cost;

  bool result = lp(C, A, b, Ae, be, xl, xu, &xs, &optimal_cost);

  if (-optimal_cost <= 1) {
    return true;
  } else {
    return false;
  }
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

  deleteZeroRows(A_env.block(0, 0, A_env.rows(), 6), b_env, &A, &b);
  deleteZeroRows(G_env.block(0, 0, G_env.rows(), 6), h_env, &G, &h);

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

Vector6d EnvironmentConstrainedVelocity_CSModeOnly(
    const Vector6d &v_goal, const std::vector<ContactPoint> &envs,
    const VectorXi &mode, ContactConstraints &cons) {

  int n_pts = envs.size();
  const int n = cons.friction_cone->number_of_sliding_planes;

  int n_var = 6;
  int n_sep = 0;
  int n_con = 0;
  for (int i = 0; i < n_pts; i++) {
    (mode[i] == 0) ? n_con++ : n_sep++;
  }

  MatrixXd A(n_con, n_var);
  MatrixXd G(n_sep, n_var);

  VectorXd b(n_con);
  VectorXd h(n_sep);
  b.setZero();
  h.setZero();

  int counter_G = 0;
  int counter_A = 0;

  for (int i = 0; i < n_pts; i++) {

    int cs_mode = mode[i];

    Matrix6d Adgco = contact_jacobian(envs[i].p, envs[i].n);

    // std::cout << "Adgco\n" << Adgco << std::endl;

    if (cs_mode == 1) { // separate
      G.block(counter_G, 0, 1, 6) = cons.basis.row(2) * Adgco;
      counter_G += 1;
    } else { // contacting
      A.block(counter_A, 0, 1, 6) = cons.basis.row(2) * Adgco;
      counter_A += 1;
    }
  }

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


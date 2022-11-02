#include "cmg_task.h"
#include "../mechanics/contacts/contact_kinematics.h"
#include "../mechanics/utilities/eiquadprog.hpp"

// ---------------------------
// Utility functions
#define MODE_TYPE_CS 0
#define MODE_TYPE_FULL 1

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

double dist_vel(const Vector6d &v, const Vector6d &v0, double wt, double wa)
{
  double d = wt * (v - v0).block(0, 0, 3, 1).norm() +
             wa * (v - v0).block(3, 0, 3, 1).norm();
  return d;
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

void copy_pts(const std::vector<ContactPoint> &pts,
              std::vector<ContactPoint> *pts_new)
{
  for (auto &pt : pts)
  {
    pts_new->push_back(pt);
  }
}

Vector7d steer_config(Vector7d x_near, Vector7d x_rand,
                      double epsilon_translation, double epsilon_angle)
{

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
                            const VectorXd &h)
{

  double Ax_b = (A * x - b).cwiseAbs().sum();
  if (Ax_b > 1e-3)
  {
    return false;
  }

  VectorXd g = G * x - h;
  for (int i = 0; i < g.size(); i++)
  {
    if (g[i] < -1e-3)
    {
      return false;
    }
  }

  return true;
}

void steer_velocity(Vector6d &x, double h, double cl = 1.0)
{

  Vector6d xx = x;
  xx.head(3) = xx.head(3) * cl;
  if (xx.norm() > h)
  {
    xx = (h / xx.norm()) * xx;
    double xx_norm = xx.tail(3).norm();
    double x_norm = x.tail(3).norm();
    if (xx_norm > 1e-4 && x_norm > 1e-4)
    {
      x = x * (xx.tail(3).norm() / x.tail(3).norm());
    }
    else
    {
      x = xx / cl;
    }
  }
  return;
}

bool ifCollide(const std::vector<ContactPoint> &pts)
{
  double thr = -0.04;
  for (const auto &pt : pts)
  {
    if (pt.d < thr)
    {
      return true;
    }
  }
  return false;
}

VectorXi deleteSeparatingMode(const VectorXi &mode)
{

  VectorXi m(mode.size());
  int n_cpts = 0;
  int n_pts = mode.size() / 3;

  for (int i = 0; i < n_pts; i++)
  {
    int cs_mode = mode(i);
    if (cs_mode == 0)
    { // contacting
      n_cpts += 1;
    }
  }
  int k = 0;
  for (int i = 0; i < n_pts; i++)
  {
    int cs_mode = mode(i);
    if (cs_mode == 0)
    { // contacting
      m(k) = cs_mode;
      m.block(n_cpts + 2 * k, 0, 2, 1) = mode.block(n_pts + 2 * i, 0, 2, 1);
      k += 1;
    }
  }
  m.conservativeResize(3 * n_cpts);
  return m;
}

VectorXi deleteModebyRemainIndex(const VectorXi &mode,
                                 const VectorXi &remain_idx, int mode_type)
{
  int n_cpts = remain_idx.size();
  int n_pts;

  if (mode_type == MODE_TYPE_CS)
  {
    n_pts = mode.size();
    VectorXi m(n_cpts);
    for (int i = 0; i < n_cpts; i++)
    {
      m[i] = mode[remain_idx[i]];
    }
    return m;
  }
  if (mode_type == MODE_TYPE_FULL)
  {
    n_pts = int(mode.size() / 3);

    VectorXi m(3 * n_cpts);
    for (int i = 0; i < n_cpts; i++)
    {
      m[i] = mode[remain_idx[i]];
      m.block(n_cpts + 2 * i, 0, 2, 1) =
          mode.block(n_pts + 2 * remain_idx[i], 0, 2, 1);
    }
    return m;
  }
}

bool ifContactingModeDeleted(const VectorXi &mode, const VectorXi &remain_idx)
{
  for (int i = 0; i < int(mode.size() / 3); i++)
  {
    if (mode[i] == 0)
    {
      bool ifremained = false;
      for (int k = 0; k < remain_idx.size(); k++)
      {
        if (i == remain_idx[k])
        {
          ifremained = true;
          break;
        }
      }
      if (!ifremained)
      {
        return true;
      }
    }
  }
  return false;
}

double CollisionInterpolation(const Vector6d &v,
                              const std::vector<ContactPoint> &pts)
{

  double d_min = 0;
  Vector3d p;
  Vector3d n;

  for (const auto &pt : pts)
  {
    if (abs(pt.d) > abs(d_min))
    {
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
  if (std::abs(a) >= std::abs(d_min))
  {
    if (d_min > 0)
    {
      k = 1 + (std::abs(d_min) - 0.005) / std::abs(a);
    }
    else
    {
      k = 1 - (std::abs(d_min) - 0.005) / std::abs(a);
    }
  }
  return k;
}

bool ifNeedVelocityCorrection(VectorXi mode,
                              const std::vector<ContactPoint> &pts)
{
  double thr = 0.03;
  for (int i = 0; i < pts.size(); i++)
  {

    if ((abs(pts[i].d) > thr) && mode[i] == 0)
    {
      return true;
    }
  }
  return false;
}

Vector6d VelocityCorrection(const std::vector<ContactPoint> &pts)
{

  int n_pts = pts.size();
  double violation = -1e-4;

  Vector6d z_axis;
  z_axis << 0, 0, 1, 0, 0, 0;
  MatrixXd N(n_pts, 6);
  VectorXd d(n_pts);

  for (int i = 0; i < n_pts; i++)
  {
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

bool contactTrack(ContactPoint pt0, ContactPoint pt1,
                  double normal_product = 0.85)
{
  if (((pt0.p - pt1.p).norm() < 0.1) &&
      ((pt0.n.transpose() * pt1.n)[0] > normal_product))
  {
    return true;
  }
  else
  {
    // std::cout << "d: " << (pt0.p - pt1.p).norm() << " ";
    return false;
  }
}

VectorXi track_contacts_remain(const std::vector<ContactPoint> &pts,
                               const std::vector<ContactPoint> &pts_new,
                               double normal_product = 0.85)
{
  VectorXi remain_idx(pts_new.size());
  int i = 0;
  int j = 0;
  while ((i < pts.size()) && (j < pts_new.size()))
  {
    if (contactTrack(pts[i], pts_new[j]), normal_product)
    {
      remain_idx[j] = i;
      j++;
    }
    i++;
  }
  if (j < pts_new.size())
  {
    VectorXi empty_idx(0);
    return empty_idx;
  }
  return remain_idx;
}

void deleteExtraContacts(const std::vector<ContactPoint> &pts0,
                         std::vector<ContactPoint> &pts)
{
  std::vector<ContactPoint> pts2;
  VectorXi track(pts0.size());
  track.setZero();

  for (auto &pt : pts)
  {
    bool iftracked = false;
    for (int i = 0; i < pts0.size(); i++)
    {
      if (track[i] == 1)
        continue;
      ContactPoint pt0 = pts0[i];
      iftracked = contactTrack(pt0, pt);
      if (iftracked)
      {
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
                            std::vector<ContactPoint> *pts_update)
{

  if (pts.size() <= 2)
  {
    return false;
  }

  double thr = 5e-2;
  Vector3d vec = pts[0].p - pts[1].p;
  vec = vec / vec.norm();
  int idx = 1;
  double d = vec.norm();
  // check if pts are in the same line
  for (int i = 2; i < pts.size(); i++)
  {
    Vector3d vec1 = pts[i].p - pts[0].p;
    vec1 = vec1 / vec1.norm();
    double cross_product = vec.cross(vec1).norm();
    if (cross_product > thr)
    {
      // not in the same line
      return false;
    }
    else
    {
      double dd = (pts[i].p - pts[0].p).norm();
      if (dd > d)
      {
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
                      std::vector<ContactPoint> *pts_update)
{

  if (pts.size() != 2)
  {
    return false;
  }

  double thr = 1e-3;
  Vector3d vec = pts[0].p - pts[1].p;
  double d = vec.norm();
  ContactPoint pt = pts[1];
  // check if pts_new are in the same line
  for (int i = 0; i < pts_new.size(); i++)
  {
    if ((vec.cross(pts_new[i].p - pts[0].p)).norm() > thr)
    {
      // not in the same line
      return false;
    }
    else
    {
      double dd = (pts_new[i].p - pts[0].p).norm();
      if (dd > d)
      {
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
                                   const VectorXi &remain_idx)
{
  std::vector<ContactPoint> envs;
  for (int i = 0; i < int(mode.size() / 3); i++)
  {
    if (mode[i] == 0)
    {
      bool ifremained = false;
      for (int k = 0; k < remain_idx.size(); k++)
      {
        if (i == remain_idx[k])
        {
          ifremained = true;
        }
      }
      if (!ifremained)
      {
        envs.push_back(pts[i]);
        envs.back().d = 0.042;
      }
    }
  }
  for (int k = 0; k < remain_idx.size(); k++)
  {
    envs.push_back(pts[remain_idx[k]]);
  }
  return VelocityCorrection(envs);
}

Vector6d EnvironmentConstrainedVelocity(const Vector6d &v_goal,
                                        const std::vector<ContactPoint> &envs,
                                        const VectorXi &env_mode,
                                        ContactConstraints &cons)
{

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

  if (A.rows() > n_var)
  {
    FullPivLU<MatrixXd> lu_decomp(A.transpose());

    if (lu_decomp.rank() >= n_var)
    {
      // if A fully constrainted the velocity
      x.setZero();
      // double f = solve_quadprog(P, p, A.transpose(), -b,  G.transpose(), -h,
      // x);
      return x;
    }
    else
    {
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
                           x, A, b, G, h)))
  { // if fail to solve the problem
    x.setZero();
  }

  // if(f > 0.0){
  //     std::cout << "solve_quadprog error" << std::endl;
  // }

  return x;
}

Vector6d EnvironmentConstrainedVelocity_CSModeOnly(
    const Vector6d &v_goal, const std::vector<ContactPoint> &envs,
    const VectorXi &mode, ContactConstraints &cons)
{

  int n_pts = envs.size();
  const int n = cons.friction_cone->number_of_sliding_planes;

  int n_var = 6;
  int n_sep = 0;
  int n_con = 0;
  for (int i = 0; i < n_pts; i++)
  {
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

  for (int i = 0; i < n_pts; i++)
  {

    int cs_mode = mode[i];

    Matrix6d Adgco = contact_jacobian(envs[i].p, envs[i].n);

    // std::cout << "Adgco\n" << Adgco << std::endl;

    if (cs_mode == 1)
    { // separate
      G.block(counter_G, 0, 1, 6) = cons.basis.row(2) * Adgco;
      counter_G += 1;
    }
    else
    { // contacting
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

  if (A.rows() > n_var)
  {
    FullPivLU<MatrixXd> lu_decomp(A.transpose());

    if (lu_decomp.rank() >= n_var)
    {
      // if A fully constrainted the velocity
      x.setZero();
      // double f = solve_quadprog(P, p, A.transpose(), -b,  G.transpose(), -h,
      // x);
      return x;
    }
    else
    {
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
                           x, A, b, G, h)))
  { // if fail to solve the problem
    x.setZero();
  }

  // if(f > 0.0){
  //     std::cout << "solve_quadprog error" << std::endl;
  // }

  return x;
}

bool isQuasistatic(const std::vector<ContactPoint> &mnps,
                   const std::vector<ContactPoint> &envs,
                   const VectorXi &env_mode, const Vector6d &f_ext_w,
                   const Vector7d object_pose, double mu_env, double mu_mnp,
                   ContactConstraints *cons)
{

  // std::cout << env_mode << std::endl;

  if (mnps.size() + envs.size() == 0)
  {
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

  mergeManipulatorandEnvironmentConstraints(
      A_mnp, b_mnp, G_mnp, h_mnp, A_env, b_env, G_env, h_env, &A, &b, &G, &h);

  int n_var = A.cols() - 6;
  if (n_var == 0)
  {
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
bool isQuasistatic(const std::vector<ContactPoint> &mnps,
                   const std::vector<ContactPoint> &envs, const Vector6d &v,
                   const Vector6d &f_ext_w, const Vector7d object_pose,
                   double mu_env, double mu_mnp, ContactConstraints *cons)
{

  VectorXi env_mode(envs.size() * 3);

  Eigen::Matrix<double, 3, 6> basis;
  basis.setZero();
  for (int i = 0; i < 3; i++)
  {
    basis(i, i) = 1;
  }

  double thr = 1e-4;

  for (int k = 0; k < envs.size(); ++k)
  {

    ContactPoint pt = envs[k];

    // calculate contact velocity
    Matrix6d Adgco = contact_jacobian(pt.p, pt.n);
    Eigen::Matrix<double, 1, 6> N = basis.row(2) * Adgco;
    Eigen::Matrix<double, 2, 6> T = basis.block<2, 6>(0, 0) * Adgco;

    // skip this contact if its normal velocity > thr
    double vn = N * v;
    if (vn > thr)
    {
      env_mode[k] = 1;
      env_mode[envs.size() + 2 * k] = 0;
      env_mode[envs.size() + 2 * k + 1] = 0;
    }

    env_mode[k] = 0;

    Vector6d vt;
    vt.setZero();
    vt.block(0, 0, 2, 1) = T * v;

    if (vt.norm() < thr)
    {
      // sticking contact
      env_mode[envs.size() + 2 * k] = 0;
      env_mode[envs.size() + 2 * k + 1] = 0;
    }
    else
    {
      // sliding contact
      // compute its ss mode
      VectorXd vt_dir = cons->friction_cone->D * vt;
      VectorXi ss_mode(vt_dir.size());
      for (int i = 0; i < vt_dir.size(); ++i)
      {
        if (vt_dir[i] > thr)
        {
          ss_mode[i] = 1;
        }
        else if (vt_dir[i] < -thr)
        {
          ss_mode[i] = -1;
        }
        else
        {
          ss_mode[i] = 0;
        }
      }

      env_mode[envs.size() + 2 * k] = ss_mode[0];
      env_mode[envs.size() + 2 * k + 1] = ss_mode[1];
    }
  }

  bool result = isQuasistatic(mnps, envs, env_mode, f_ext_w, object_pose,
                              mu_env, mu_mnp, cons);

  return result;
}

bool isQuasidynamic(const Vector6d &v_b, const std::vector<ContactPoint> &mnps,
                    const std::vector<ContactPoint> &envs,
                    const VectorXi &env_mode, const Vector6d &f_ext_w,
                    const Matrix6d &object_inertia, const Vector7d object_pose,
                    double mu_env, double mu_mnp, double wa, double wt,
                    double h_time, ContactConstraints *cons)
{

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

  cons->ModeConstraints(envs, env_mode, mu_env, f_ext_o, &A_env, &b_env, &G_env,
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
  if (G_rows < 12)
  {

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

  if (std::isinf(f))
  { // if fail to solve the problem
    return false;
  }
  else if (!ifConstraintsSatisfied(x, A, b, G, h))
  {
    std::cout << " Constraints not satisfied for qp! " << std::endl;
    return false;
  }
  else
  {
    x_v = x.block(0, 0, 6, 1);
  }
  if ((x_v - v_b).norm() > 0.05 * v_b.norm())
  {
    std::cout << "solved v too large error for quasidynamic verification!"
              << std::endl;
    return false;
  }
  return true;
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

  int mode_type = (env_mode.size() == envs.size()) ? MODE_TYPE_CS : MODE_TYPE_FULL;

  Vector6d v_b_pre;
  v_b_pre.setZero();

  int counter;
  int delete_c = 0;

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
      EnvironmentConstrainedVelocity(v_star, envs, env_mode, *this->cons);
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

    steer_velocity(v_b, h, this->charac_len);

    // integrate v
    Vector7d x_new = SE32pose(T * se32SE3(v_b));

    this->m_world->updateObjectPose(x_new);

    // check penetration & interpolation (break may happen)
    envs.clear();
    this->m_world->getObjectContacts(&envs, x_new);

    // velocity correction
    int pre_env_size = (mode_type == MODE_TYPE_FULL) ? env_mode.size() / 3 : env_mode.size();

    if (envs.size() != 0 && envs.size() == pre_env_size &&
        (ifNeedVelocityCorrection(env_mode, envs)))
    {
      // std::cout << "velocity correction " << counter << std::endl;
      Vector6d v_corr = VelocityCorrection(envs);
      x_new = SE32pose(pose2SE3(x_new) * se32SE3(v_corr));
      envs.clear();
      this->m_world->getObjectContacts(&envs, x_new);
    }

    if (envs.size() > pre_env_size)
    {
      // Detects new contacts: project the object back to zero contact distance
      Vector6d v_corr = VelocityCorrection(envs);
      x_new = SE32pose(pose2SE3(x_new) * se32SE3(v_corr));
      path->push_back(x_new);
      x = x_new;
      envs.clear();
      this->m_world->getObjectContacts(&envs, x_new);
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
        printf("contact track fails.\n");
        break;
      }
      if (ifContactingModeDeleted(env_mode, remain_idx))
      {
        if (h < 0.004 / 3)
        {
          // TODO: need to fix this
          delete_c++;
          if (delete_c > 4)
            break;
          // printf("Collision Detection delelte contacting mode \n");
          Vector6d v_corr =
              recoverContactingContacts(envs_pre, env_mode, remain_idx);
          x_new = SE32pose(pose2SE3(x_new) * se32SE3(v_corr));
          envs.clear();
          this->m_world->getObjectContacts(&envs, x_new);
          if (envs.size() <= pre_env_size)
            continue;
          else
            break;
        }
        h = h / 1.5;
        envs = envs_pre;
        continue;
      }
      else
      {
        env_mode = deleteModebyRemainIndex(env_mode, remain_idx, mode_type);
      }
    }

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

void enumerate_cs_modes(ContactConstraints &cons,
                        const std::vector<ContactPoint> &envs,
                        std::vector<VectorXi> *modes)
{
  // contacting-separating mode enumeration
  MatrixXd A;
  VectorXd b;
  MatrixXd D;
  VectorXd d;
  cons.NormalVelocityConstraints(envs, &A, &b);
  cons.TangentVelocityConstraints(envs, &D, &d);

  if (envs.size() == 0)
  {
    VectorXi m(0);
    modes->push_back(m);
  }
  else
  {
    cs_mode_enumeration(A, modes);
  }
}

void enumerate_ss_modes(ContactConstraints &cons,
                        const std::vector<ContactPoint> &envs,
                        const VectorXi &cs_mode,
                        std::vector<VectorXi> *ss_modes)
{

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

void CMGTASK::initialize(const Vector7d &start_object_pose,
                         const Vector7d &goal_object_pose, double goal_thr,
                         double wa, double wt, double charac_len, double mu_env,
                         double mu_mnp, Matrix6d object_inertia,
                         Vector6d f_gravity,
                         std::shared_ptr<WorldTemplate> world,
                         int n_robot_contacts,
                         int dynamic_type,
                         const SearchOptions &options,
                         bool if_refine, double refine_dist)
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
  this->if_refine = if_refine;
  this->refine_dist = refine_dist;

  // initialize the shared RRT
  shared_rrt =
      std::make_shared<ReusableRRT>(this->wa, this->charac_len * this->wt);
  ReusableRRT::Node start_node(start_object_pose);

  this->m_world->getObjectContacts(&(start_node.envs), start_node.config);
  enumerate_cs_modes(*this->cons.get(), start_node.envs, &start_node.modes);
  shared_rrt->initial_node(&start_node);
}

CMGTASK::State CMGTASK::generate_state(const Vector7d &object_pose) const
{
  CMGTASK::State state_;
  state_.m_pose = object_pose;
  this->m_world->getObjectContacts(&state_.envs, object_pose);
  enumerate_cs_modes(*this->cons.get(), state_.envs, &state_.modes);

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
  bool if_extend_root_to_goal = false;
  if (!shared_rrt->nodes[root_node_idx].has_been_root)
  {
    if_extend_root_to_goal = true;
    shared_rrt->nodes[root_node_idx].has_been_root = true;
    shared_rrt->nodes[root_node_idx].is_extended_to_goal = true;
  }

  for (int kk = 0; kk < this->search_options.max_samples; kk++)
  {

    // std::cout << "rrt iter: " << kk << std::endl;

    // bias sample toward the goal
    Vector7d x_rand;
    int near_idx;

    if (randd() > this->search_options.goal_biased_prob)
    {
      Vector3d p_rand;
      Quaterniond q_rand;
      p_rand =
          sample_position(this->search_options.x_ub, this->search_options.x_lb);

      q_rand = (this->search_options.sampleSO3)
                   ? generate_unit_quaternion()
                   : sample_rotation(this->search_options.sample_rotation_axis);

      x_rand << p_rand[0], p_rand[1], p_rand[2], q_rand.x(), q_rand.y(),
          q_rand.z(), q_rand.w();
      // near_idx = shared_rrt->nearest_neighbor(x_rand);
      near_idx = shared_rrt->nearest_neighbor_subtree(x_rand, root_node_idx,
                                                      subtree, false, true);
    }
    else
    {
      x_rand = this->goal_object_pose;
      if (if_extend_root_to_goal)
      {
        near_idx = root_node_idx;
        if_extend_root_to_goal = false;
      }
      else
      {
        near_idx = shared_rrt->nearest_neighbor_subtree(x_rand, root_node_idx,
                                                        subtree, true, true);
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

    Vector6d v_star =
        compute_rbvel_body(shared_rrt->nodes[near_idx].config, x_rand);

    Vector6d f_o =
        weight_w2o(shared_rrt->nodes[near_idx].config, this->f_gravity);

    // contact mode enumeration
    if (shared_rrt->nodes[near_idx].modes.size() == 0)
    {
      this->m_world->getObjectContacts(&(shared_rrt->nodes[near_idx].envs),
                                       shared_rrt->nodes[near_idx].config);
      enumerate_cs_modes(*this->cons.get(), shared_rrt->nodes[near_idx].envs,
                         &shared_rrt->nodes[near_idx].modes);
    }

    // for every mode do forward integration
    Vector6d v_zero = Vector6d::Zero();
    // move
    double d_zero = dist_vel(v_zero, v_star, shared_rrt->translation_weight,
                             shared_rrt->angle_weight);

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

      // std::cout << "cs mode " << cs_mode.transpose() << std::endl;

      // extend cs_mode with all sticking mode and free sliding mode
      std::vector<VectorXi> mode_to_extend;
      {
        VectorXi all_sticking_mode(3 * cs_mode.size());
        all_sticking_mode.setZero();
        all_sticking_mode.block(0, 0, cs_mode.size(), 1) = cs_mode;

        Vector6d v = EnvironmentConstrainedVelocity(
            v_star, shared_rrt->nodes[near_idx].envs, all_sticking_mode,
            *this->cons);

        if (dist_vel(v, v_star, shared_rrt->translation_weight,
                     shared_rrt->angle_weight) < d_zero)
        {
          mode_to_extend.push_back(all_sticking_mode);
        }
      }
      mode_to_extend.push_back(
          cs_mode); // extend this cs_mode in free sliding way

      /// choose sliding mode end

      for (const auto &mode : mode_to_extend)
      {

        // std::cout << "Extend mode: " << mode.transpose() << std::endl;

        std::vector<Vector7d> path;

        // move
        this->forward_integration(shared_rrt->nodes[near_idx].config, x_rand,
                                  shared_rrt->nodes[near_idx].envs, mode,
                                  &path);

        // if integration is successful
        if (path.size() > 1)
        {

          if (shared_rrt->find_node(path.back(), mode, root_node_idx) != -1)
          {
            // printf("This node is already in the tree!\n");
            continue;
          }
          else
          {

            ReusableRRT::Node new_node(path.back());
            ReusableRRT::Edge new_edge(mode, path);

            shared_rrt->add_node(&new_node, near_idx, &new_edge);

            if (near_idx == root_node_idx)
            {
              // for the nodes expaned from the root node, we need to check if
              // the mode is the desired one
              if ((cs_mode - start_state.modes[start_state.m_mode_idx])
                      .norm() == 0)
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

  int finger_idx = state.finger_index;

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
        if (idx > 0)
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

  if (this->task_dynamics_type == CMG_QUASISTATIC)
  {

    Vector6d v = compute_rbvel_body(x_object, x_object_now);
    dynamic_feasibility =
        isQuasistatic(mnps, this->saved_object_trajectory[pre_timestep].envs, v,
                      this->f_gravity, x_object, this->mu_env, this->mu_mnp,
                      this->cons.get());
  }
  else if (this->task_dynamics_type == CMG_QUASIDYNAMIC)
  {
    double h_time = 0.01;
    Vector6d v = compute_rbvel_body(
        x_object, this->saved_object_trajectory[state.timestep].m_pose);

    // dynamic_feasibility = isQuasidynamic(
    //     v, mnps, this->saved_object_trajectory[pre_timestep].envs, ss_mode,
    //     this->f_gravity, this->object_inertia, x_object, this->mu_env,
    //     this->mu_mnp, this->wa, this->wt, h_time, this->cons.get());
    dynamic_feasibility = true; // TODO: implement quasidynamic with velocity
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
      else if (pre_fingertips[k] <= 0)
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

double CMGTASK::evaluate_path(const std::vector<State2> &path)
{

  if (!path.back().is_valid)
  {
    return 0.0;
  }

  double reward_finger_stay = 0.0;
  for (int k = 0; k < path.size() - 1; ++k)
  {
    reward_finger_stay +=
        double(this->number_of_robot_contacts -
               number_of_different_elements(
                   this->get_finger_locations(path[k].finger_index),
                   this->get_finger_locations(path[k + 1].finger_index))) /
        double(this->number_of_robot_contacts);
  }

  reward_finger_stay = reward_finger_stay / double(path.size());

  double reward_path_size = 1.0 / double(path.size());

  return reward_finger_stay + reward_path_size;
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
    return;
  }

  this->saved_object_trajectory.push_back(path[0]);

  for (int i = 1; i < path.size(); ++i)
  {
    State state = path[i];

    for (int k = 0; k < state.m_path.size() - 1; k++)
    {
      double cur_dist = this->shared_rrt->dist(this->saved_object_trajectory.back().m_pose,
                                               state.m_path[k]);
      if (cur_dist >= this->refine_dist)
      {
        // add a new state
        State new_state;
        new_state.m_pose = state.m_path[k];
        this->m_world->getObjectContacts(&new_state.envs, new_state.m_pose);
        this->saved_object_trajectory.push_back(new_state);
      }
    }

    this->saved_object_trajectory.push_back(state);
  }
}

long int factorial(int n, int m)
{
  long int result = 1;
  for (int i = n; i > n - m; i--)
  {
    result *= i;
  }
  return result;
}

std::vector<int> CMGTASK::get_finger_locations(int finger_location_index)
{

  // obtain finger location idxes from the single location idx

  int N = this->object_surface_pts.size();
  int n = this->number_of_robot_contacts;
  int x = finger_location_index;

  std::vector<int> finger_locations;
  if (x == -1){
    for (int i = 0; i < n; ++i)
    {
      finger_locations.push_back(0);
    }
    return finger_locations;
  }
  for (int k = 0; k < n; ++k)
  {
    int divisor = factorial(N - k - 1, n - k -1);
    int a = int(x / divisor);
    x -= a * divisor;
    for (auto p : finger_locations)
    {
      if (p == a)
      {
        a += 1;
      }
    }
    finger_locations.push_back(a);
  }

  // for (int k = 0; k < n; ++k)
  // {
  //   int a = int(x / pow(N, (n - k - 1)));
  //   x -= a * (int)pow(N, (n - k - 1));

  //   finger_locations.push_back(a);
  // }
  return finger_locations;
}

int CMGTASK::get_number_of_robot_actions(const CMGTASK::State2 &state)
{
  // // return combination of fingers for now
  if (this->n_finger_combinations == -1)
  {
    this->n_finger_combinations = factorial(this->object_surface_pts.size(), this->number_of_robot_contacts);
  }

  return this->n_finger_combinations;
  // return pow(this->object_surface_pts.size(), this->number_of_robot_contacts);
}
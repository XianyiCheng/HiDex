#include "cmg_task.h"
#include "../mechanics/contacts/contact_kinematics.h"
#include "../mechanics/utilities/eiquadprog.hpp"
#include "../mechanics/utilities/combinatorics.h"

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

bool ifContactingModeDeleted(const VectorXi &mode, const VectorXi &remain_idx,
                             int n_pts)
{
  for (int i = 0; i < n_pts; i++)
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
  double thr = 0.02;
  for (int i = 0; i < pts.size(); i++)
  {

    if ((abs(pts[i].d) > thr) && mode[i] == 0)
    {
      return true;
    }
  }
  return false;
}

bool is_penetrate(const std::vector<ContactPoint> &pts)
{
  double thr = 0.05;
  for (int i = 0; i < pts.size(); i++)
  {
    if ((abs(pts[i].d) > thr))
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
                  double normal_product = 0.85, double thr = 0.1)
{
  if (((pt0.p - pt1.p).norm() < thr) &&
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
                               double normal_product = 0.85, double thr = 0.1)
{
  std::vector<int> remain_idxes;
  for (int i = 0; i < pts_new.size(); i++)
  {
    for (int j = 0; j < pts.size(); j++)
    {
      if (contactTrack(pts[j], pts_new[i], normal_product, thr))
      {
        remain_idxes.push_back(j);
        break;
      }
    }
  }

  if (remain_idxes.size() < pts_new.size())
  {
    VectorXi empty_idx(0);
    return empty_idx;
  }

  VectorXi remain_idx(remain_idxes.size());
  for (int i = 0; i < remain_idxes.size(); i++)
  {
    remain_idx[i] = remain_idxes[i];
  }

  return remain_idx;
}

VectorXi cs_mode_from_contacts(const std::vector<ContactPoint> &pts,
                               const std::vector<ContactPoint> &pts_new)
{
  // conservative estimation, do not consider the normal direction rotating for
  // 90 degree

  VectorXi remain_idx = track_contacts_remain(pts, pts_new, 0.3, 0.25);

  VectorXi mode(pts.size());

  mode.setOnes();

  for (int i = 0; i < remain_idx.size(); ++i)
  {
    mode[remain_idx[i]] = 0;
  }

  return mode;
}

VectorXi conservative_cs_mode(const VectorXi &m1, const VectorXi &m2)
{
  VectorXi mode(m1.size());
  mode.setZero();
  for (int i = 0; i < m1.size(); ++i)
  {
    if (m1[i] == 1 && m2[i] == 1)
    {
      mode[i] = 1;
    }
  }
  return mode;
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
  for (int i = 0; i < pts.size(); i++)
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

  // force check

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

  MatrixXd A;
  MatrixXd G;
  VectorXd b;
  VectorXd h;

  if (mnps.size() > 0)
  {

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
  }
  else
  {
    A = A_env;
    b = b_env;
    G = G_env;
    h = h_env;
  }

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
}

VectorXi mode_from_velocity(const Vector6d &v,
                            const std::vector<ContactPoint> &envs,
                            ContactConstraints *cons)
{
  VectorXi env_mode(envs.size() * 3);

  Eigen::Matrix<double, 3, 6> basis;
  basis.setZero();
  for (int i = 0; i < 3; i++)
  {
    basis(i, i) = 1;
  }

  double thr = 1e-3;

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
      continue;
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
  return env_mode;
}

bool isQuasistatic(const std::vector<ContactPoint> &mnps,
                   const std::vector<ContactPoint> &envs,
                   const VectorXi &ref_cs_mode, const Vector6d &v,
                   const Vector6d &f_ext_w, const Vector7d object_pose,
                   double mu_env, double mu_mnp, ContactConstraints *cons)
{

  // force check

  VectorXi env_mode = mode_from_velocity(v, envs, cons);

  for (int i = 0; i < ref_cs_mode.size(); ++i)
  {
    if (env_mode[i] == 1)
    {
      if (ref_cs_mode[i] == 0)
      {
        env_mode[i] = 0;
      }
    }
  }

  bool result = isQuasistatic(mnps, envs, env_mode, f_ext_w, object_pose,
                              mu_env, mu_mnp, cons);
  return result;
}

bool isQuasistatic(const std::vector<ContactPoint> &mnps,
                   const std::vector<ContactPoint> &envs, const Vector6d &v,
                   const Vector6d &f_ext_w, const Vector7d object_pose,
                   double mu_env, double mu_mnp, ContactConstraints *cons)
{

  // force check

  VectorXi env_mode = mode_from_velocity(v, envs, cons);

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
  enumerate_cs_modes(*this->cons.get(), start_node.envs, &start_node.modes);
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

  // std::cout << "Search a new path with shared rrt" << std::endl;

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

    // std::cout << "rrt iter: " << kk << std::endl;

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
        // std::cout << "sampled random state" << std::endl;
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
      // std::cout << "sampled idx to extend to goal" << std::endl;
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
        // std::cout << "sampled goal state" << std::endl;
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

    // std::cout << "x_rand " << x_rand.transpose() << std::endl;

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

        // if ((dist_vel(v, v_star, shared_rrt->translation_weight,
        //               shared_rrt->angle_weight) < d_zero) &&
        //     (v.norm() > 1e-6))
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

        // if ((dist_vel(v, v_star, shared_rrt->translation_weight,
        //               shared_rrt->angle_weight) < d_zero) &&
        //     (v.norm() > 1e-6))
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

        // std::cout << "Extend mode: " << mode.transpose() << std::endl;

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

            enumerate_cs_modes(*this->cons.get(), new_node.envs,
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

          // std::cout << "New node idx: " << this->shared_rrt->nodes.size()
          //           << ", Parent: " << near_idx << ", "
          //           << path.back().transpose() << std::endl;

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
    // std::cout << "Pose " << s.m_pose.transpose() << std::endl;
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

  double total_finger_changes = this->total_finger_change_ratio(
      path);

  // double reward = double(node->m_state.t_max) + 1.0 - total_finger_changes;

  double x = total_finger_changes / double(path.back().t_max);
  double y = 10.80772595 * x + -4.59511985;
  double reward = 1.0 / (1.0 + std::exp(y));

  if (this->grasp_measure_charac_length <= 0.0) {
    return reward;
  } else {
    reward *= 0.5;

    double avg_grasp_d = 0.0;
    for (auto s2 : path) {
      if (s2.finger_index == -1 || s2.timestep == -1) {
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
      this->object_surface_pts.size();         // N: number of surface contact points
  int n = this->number_of_robot_contacts;      // n: number of fingers
  long int action_idx = finger_location_index; // action_idx: index of the action

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
    dynamic_feasibility = true; // TODO: implement quasidynamic
  }

  return dynamic_feasibility;
}

long int CMGTASK::pruning_check(const Vector7d &x, const Vector6d &v,
                           const std::vector<ContactPoint> &envs)
{

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

      dynamic_feasibility =
          isQuasistatic(mnps, envs, v, this->f_gravity, x, this->mu_env,
                        this->mu_mnp, this->cons.get());
      // if (dynamic_feasibility) {
      //   std::cout << "Passed pruning check" << std::endl;
      //   std::cout << "x " << x.transpose() << std::endl;
      //   std::cout << "v " << v.transpose() << std::endl;
      //   std::cout << "fingertips ";
      //   for (auto i : fingertip_idx) {
      //     std::cout << i << " ";
      //   }
      //   std::cout << std::endl;
      // }
    }
    else if (this->task_dynamics_type == CMG_QUASIDYNAMIC)
    {
      dynamic_feasibility = true; // TODO: implement quasidynamic
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
      dynamic_feasibility = true; // TODO: implement quasidynamic
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

  if (this->task_dynamics_type == CMG_QUASISTATIC)
  {
    dynamic_feasibility =
        isQuasistatic(mnps, this->saved_object_trajectory[timestep].envs,
                      reference_cs_mode, v, this->f_gravity, x_object,
                      this->mu_env, this->mu_mnp, this->cons.get());
  }
  else if (this->task_dynamics_type == CMG_QUASIDYNAMIC)
  {
    double h_time = 0.01;

    // dynamic_feasibility = isQuasidynamic(
    //     v, mnps, this->saved_object_trajectory[pre_timestep].envs,
    //     ss_mode, this->f_gravity, this->object_inertia, x_object,
    //     this->mu_env, this->mu_mnp, this->wa, this->wt, h_time,
    //     this->cons.get());
    dynamic_feasibility = true; // TODO: implement quasidynamic with velocity
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
    bool dynamic_feasibility = isQuasistatic(
        remain_mnps, this->saved_object_trajectory[state.timestep].envs,
        ss_mode_relocate, this->f_gravity,
        this->saved_object_trajectory[state.timestep].m_pose, this->mu_env,
        this->mu_mnp, this->cons.get());
    return dynamic_feasibility;
  }
  return true;
}
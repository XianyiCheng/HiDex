
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
  return VectorXi(0);
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
  x = G.inverse() * g0 / 2;

  // double f = solve_quadprog(G, g0, A, b,  C, e, x);

  // if (f > 0.0){
  //     x.setZero();
  // }

  return x;
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
#include "mode_utils.h"

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

bool contactTrack(ContactPoint pt0, ContactPoint pt1,
                  double normal_product, double thr)
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
                               double normal_product, double thr)
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

VectorXi conservative_mode_from_velocity( const std::vector<ContactPoint> &envs,
                   const VectorXi &ref_cs_mode, const Vector6d &v, ContactConstraints *cons){
    
  VectorXi env_mode = mode_from_velocity(v, envs, cons);

  for (int i = 0; i < ref_cs_mode.size(); ++i) {
    if (env_mode[i] == 1) {
      if (ref_cs_mode[i] == 0) {
        env_mode[i] = 0;
      }
    }
  }
  return env_mode;
}
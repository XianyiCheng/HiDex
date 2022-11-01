#include "../mechanics/mechanics.h"

#include <algorithm>
#include <iostream>
#include <math.h>
#include <mcts.h>
#include <string>

void compare()
{

  std::vector<Vector6d> mnps;
  std::vector<Vector6d> envs;

  // {
  //   Vector6d cp;
  //   cp << -1,0,0.2,1,0,0;
  //   mnps.push_back(cp);
  // }

  // {
  //   Vector6d cp;
  //   cp << 1,0,0.2,-1,0,0;
  //   mnps.push_back(cp);
  // }
  {
    Vector6d cp;
    cp << 1, 0, 0, -1, 0, 0;
    mnps.push_back(cp);
  }
  // envs
  {
    {
      Vector6d cp;
      cp << 1, 1, -1, 0, 0, 1;
      envs.push_back(cp);
    }
    {
      Vector6d cp;
      cp << -1, 1, -1, 0, 0, 1;
      envs.push_back(cp);
    }
    {
      Vector6d cp;
      cp << -1, -1, -1, 0, 0, 1;
      envs.push_back(cp);
    }
    {
      Vector6d cp;
      cp << 1, -1, -1, 0, 0, 1;
      envs.push_back(cp);
    }
  }

  std::vector<ContactPoint> mnp_contacts;
  std::vector<ContactPoint> env_contacts;

  for (auto pt : mnps)
  {
    ContactPoint cp(pt.head(3), pt.tail(3));
    mnp_contacts.push_back(cp);
  }

  for (auto pt : envs)
  {
    ContactPoint cp(pt.head(3), pt.tail(3));
    env_contacts.push_back(cp);
  }

  Vector6d f_ext_w;
  f_ext_w << 0, 0, -1, 0, 0, 0;

  Vector7d pose;
  pose << 0, 0, 0, 0, 0, 0, 1;

  Vector6d v;
  v << 1, -0.1, 0, 0, 0, 0;

  Eigen::VectorXi env_mode(12);
  // env_mode << 0, 0, 0, 0, -1, 1, -1, 1, -1, 1, -1, 1;
  env_mode << 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0;

  bool is_balance =
      force_balance(mnps, envs, env_mode, f_ext_w, pose, 0.3, 0.6);

  bool is_quasistatic = quasistatic_check_2(mnps, envs, v, pose, f_ext_w, 0.3, 0.6);

  std::cout << "Force balance with mode " << is_balance << std::endl;
  std::cout << "Force balance with complementarity " << is_quasistatic << std::endl;
}

void grasp()
{
  std::vector<Vector6d> mnps;
  std::vector<Vector6d> envs;

  {
    Vector6d cp;
    cp << -1, 0, 0.2, 1, 0, 0;
    mnps.push_back(cp);
  }

  {
    Vector6d cp;
    cp << 1, 0, 0.2, -1, 0, 0;
    mnps.push_back(cp);
  }
  Vector7d pose;
  pose << 0, 0, 0, 0, 0, 0, 1;
  Vector6d v;
  v << 0.1, 0, 0.1, 0.1, 0, 0.1;
  Vector6d f_ext_w;
  f_ext_w << 0, 0, -1, 0, 0, 0;

  bool is_quasistatic = quasistatic_check_2(mnps, envs, v, pose, f_ext_w, 0.3, 0.6);

  std::cout << "Force balance with complementarity " << is_quasistatic << std::endl;
}

int main()
{
  grasp();
}

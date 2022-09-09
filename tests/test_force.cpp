#include "../mechanics/mechanics.h"

#include <algorithm>
#include <iostream>
#include <math.h>
#include <mcts.h>
#include <string>

int main() {

  std::vector<Vector6d> mnps;
  std::vector<Vector6d> envs;

  {
    Vector6d cp;
    cp << -1, 0, 0, 1, 0, 0;
    mnps.push_back(cp);
  }
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

  Vector6d f_ext_w;
  f_ext_w << 0, 0, -1, 0, 0, 0;

  Vector7d pose;
  pose << 0, 0, 0, 0, 0, 0, 1;

  Eigen::VectorXi env_mode(12);
  env_mode << 0,0,0,0,1,0,1,0,1,0,1,0;

  bool is_balance =
      force_balance(mnps, envs, env_mode, f_ext_w, pose, 0.3, 0.8);

  std::cout << is_balance << std::endl;
}
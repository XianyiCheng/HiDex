
#include "../mechanics/contacts/contact_kinematics.h"
#include "../mechanics/utilities/combinatorics.h"
#include "../mechanics/utilities/eiquadprog.hpp"
#include "../tasks/cmg_task.h"

void drop() {

  Vector6d v_b = Vector6d::Zero();
  v_b[2] = -0.1;
  std::vector<ContactPoint> mnps;
  std::vector<ContactPoint> envs;
  VectorXi env_mode(0);
  Vector6d f_ext_w = Vector6d::Zero();
  f_ext_w[2] = -9.81;
  Matrix6d object_inertia = Matrix6d::Identity();
  Vector7d object_pose;
  object_pose << 0, 0, 0, 0, 0, 0, 1;
  double mu_env = 0.5;
  double mu_mnp = 0.5;
  double wa = 1;
  double wt = 1;
  double h_time = 0.01;
  ContactConstraints cons(2);

  bool result =
      isQuasidynamic(v_b, mnps, envs, env_mode, f_ext_w, object_inertia,
                     object_pose, mu_env, mu_mnp, wa, wt, h_time, &cons);

  std::cout << "drop result: " << result << std::endl;
}

void rotate() {

  Vector6d v_goal = Vector6d::Zero();
  v_goal[2] = -0.1;

  std::vector<ContactPoint> mnps;
  std::vector<ContactPoint> envs;
  envs.push_back(ContactPoint(Vector3d(-1, 1, -1), Vector3d(0, 0, 1)));
  envs.push_back(ContactPoint(Vector3d(-1, -1, -1), Vector3d(0, 0, 1)));
  VectorXi env_mode(6);
  env_mode << 0, 0, 0, 0, 0, 0;

  Vector6d f_ext_w = Vector6d::Zero();
  f_ext_w << -4, 0, -8, 0, 0, 0;
  Matrix6d object_inertia = Matrix6d::Identity();
  Vector7d object_pose;
  object_pose << 0, 0, 0, 0, 0, 0, 1;
  double mu_env = 0.8;
  double mu_mnp = 0.5;
  double wa = 1;
  double wt = 1;
  double h_time = 0.01;
  ContactConstraints cons(2);

  Vector6d v_b = EnvironmentConstrainedVelocity(v_goal, envs, env_mode, cons);
 
  std::cout << "v_b "<< v_b.transpose() << std::endl;
  Vector6d v_zero = Vector6d::Zero();
  std::cout << v_b.normalized().transpose()*v_zero.normalized()<< std::endl;
  bool result =
      isQuasidynamic(v_b, mnps, envs, env_mode, f_ext_w, object_inertia,
                     object_pose, mu_env, mu_mnp, wa, wt, h_time, &cons);

  std::cout << "rotate result: " << result << std::endl;
}


int main() {
  drop();
  rotate();
}

#include "../src/mechanics/contacts/contact_kinematics.h"
#include "../src/mechanics/force_check.h"
#include "../src/mechanics/mode_utils.h"
#include "../src/mechanics/utilities/combinatorics.h"
#include "../src/mechanics/utilities/eiquadprog.hpp"
#include "../src/mechanics/manipulators/DartPointManipulator.h"

void drop()
{
  std::cout << "Test drop" << std::endl;
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
                     object_pose, mu_env, mu_mnp, wa, wt, h_time, &cons, 0.5);

  std::cout << "drop result: " << result << std::endl;
}

void rotate()
{
  std::cout << "Test rotate" << std::endl;
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

  std::cout << "v_b " << v_b.transpose() << std::endl;

  bool result =
      isQuasidynamic(v_b, mnps, envs, env_mode, f_ext_w, object_inertia,
                     object_pose, mu_env, mu_mnp, wa, wt, h_time, &cons, 0.5);

  std::cout << "rotate result: " << result << std::endl;
}

void slide()
{

  Vector6d v;
  v << 0.01, 0, 0, 0, 0, 0;

  DartPointManipulator robot(1,0.1);
  robot.is_patch_contact = true;

  
  std::vector<ContactPoint> fingertips;
  fingertips.push_back(ContactPoint(Vector3d(-1, 0.2,0.1), Vector3d(1, 0, 0)));
  std::vector<ContactPoint> mnps;
  robot.Fingertips2PointContacts(fingertips, &mnps);
  std::vector<ContactPoint> envs;
  envs.push_back(ContactPoint(Vector3d(-1, 1, -1), Vector3d(0, 0, 1)));
  envs.push_back(ContactPoint(Vector3d(-1, -1, -1), Vector3d(0, 0, 1)));
  envs.push_back(ContactPoint(Vector3d(1, 1, -1), Vector3d(0, 0, 1)));
  envs.push_back(ContactPoint(Vector3d(1, -1, -1), Vector3d(0, 0, 1)));
  VectorXi cs_mode(4);
  cs_mode << 0, 0, 0, 0;

  Vector6d f_ext_w = Vector6d::Zero();
  f_ext_w[2] = -9.81;
  Matrix6d object_inertia = Matrix6d::Identity();
  object_inertia(3,3) = 0.167;
  object_inertia(4,4) = 0.167;
  object_inertia(5,5) = 0.167;
  Vector7d object_pose;
  object_pose << 0, 0, 0, 0, 0, 0, 1;
  double mu_env = 0.4;
  double mu_mnp = 0.8;
  double wa = 1;
  double wt = 1;
  double h_time = 10;
  ContactConstraints cons(2);

  VectorXi env_mode = mode_from_velocity(v, envs, &cons);
  env_mode.head(envs.size()) = cs_mode;

  bool result =
      isQuasidynamic(v, mnps, envs, env_mode, f_ext_w, object_inertia,
                     object_pose, mu_env, mu_mnp, wa, wt, h_time, &cons, 0.5);

  bool result_quasistatic = isQuasistatic(mnps, envs, env_mode, f_ext_w,
                                          object_pose, mu_env,
                                          mu_mnp, &cons);

  std::cout << "sliding result: " << result << std::endl;
  std::cout << "sliding quasistatic result: " << result_quasistatic << std::endl;
}

int main()
{
  // drop();
  // rotate();
  slide();
}
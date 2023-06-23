
#include "../src/mechanics/contacts/contact_kinematics.h"
#include "../src/mechanics/force_check.h"
#include "../src/mechanics/manipulators/DartPointManipulator.h"
#include "../src/mechanics/mode_utils.h"
#include "../src/mechanics/utilities/combinatorics.h"
#include "../src/mechanics/utilities/eiquadprog.hpp"

void drop() {
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
  bool result_quasidynamic_lp = isQuasidynamic_LP(
      v_b, mnps, envs, env_mode, f_ext_w, object_inertia, object_pose, mu_env,
      mu_mnp, wa, wt, h_time, &cons, 0.5);
  std::cout << "result_quasidynamic_lp: " << result_quasidynamic_lp
            << std::endl;

  std::cout << "drop result: " << result << std::endl;
}

void rotate() {
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

void slide() {

  Vector6d v;
  v << 0.7, 0, 0, 0, 0, 0;

  DartPointManipulator robot(1, 0.1);
  robot.is_patch_contact = true;

  // std::vector<ContactPoint> fingertips;
  // fingertips.push_back(ContactPoint(Vector3d(-1, 0.2, 0.1), Vector3d(1, 0,
  // 0))); std::vector<ContactPoint> mnps;
  // robot.Fingertips2PointContacts(fingertips, &mnps);

  std::vector<ContactPoint> mnps;
  mnps.push_back(ContactPoint(Vector3d(-0.4, 0, 0), Vector3d(0, 0, 1)));
  mnps.push_back(ContactPoint(Vector3d(-0.55, 0.0865, 0), Vector3d(0, 0, 1)));
  mnps.push_back(ContactPoint(Vector3d(-0.55, -0.0865, 0), Vector3d(0, 0, 1)));

  std::vector<ContactPoint> envs;
  envs.push_back(
      ContactPoint(Vector3d(0.5, 0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(
      ContactPoint(Vector3d(0.5, -0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(
      ContactPoint(Vector3d(-0.5, -0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(
      ContactPoint(Vector3d(-0.5, 0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));

  VectorXi cs_mode(4);
  cs_mode << 0, 0, 0, 0;

  Vector6d f_ext_w = Vector6d::Zero();
  f_ext_w[2] = -1;
  Matrix6d object_inertia = Matrix6d::Identity();
  object_inertia(3, 3) = 0.167;
  object_inertia(4, 4) = 0.167;
  object_inertia(5, 5) = 0.167;
  Vector7d object_pose;
  object_pose << -0.1, 0, 0.5, 0, 0, 0, 1;
  double mu_env = 0.4;
  double mu_mnp = 0.8;
  double wa = 1;
  double wt = 1;
  double h_time = 10;
  ContactConstraints cons(2);

  /* v: 0.7   0   0   0   0   0
  Envs
  ContactPoint(Vector3d( 0.5  0.5 -0.5),Vector3d(0 0 1), 9.99868e-05))
  ContactPoint(Vector3d( 0.5 -0.5 -0.5),Vector3d(0 0 1), 9.99868e-05))
  ContactPoint(Vector3d(-0.5 -0.5 -0.5),Vector3d(0 0 1), 9.99868e-05))
  ContactPoint(Vector3d(-0.5  0.5 -0.5),Vector3d(0 0 1), 9.99868e-05))
  Mnps
  ContactPoint(Vector3d(-0.4    0    0),Vector3d(0 0 1), 0))
  ContactPoint(Vector3d( -0.55 0.0865      0),Vector3d(0 0 1), 0))
  ContactPoint(Vector3d(  -0.55 -0.0865       0),Vector3d(0 0 1), 0))
  env_mode: 0 0 0 0 1 0 1 0 1 0 1 0
  f_gravity 0  0 -1  0  0  0
  object_inertia    1     0     0     0     0     0
      0     1     0     0     0     0
      0     0     1     0     0     0
      0     0     0 0.167     0     0
      0     0     0     0 0.167     0
      0     0     0     0     0 0.167
  x: -0.1    0  0.5    0    0    0    1
  mu_env: 0.4
  mu_mnp: 0.8
  wa: 1
  wt: 1
  */

  // VectorXi env_mode = mode_from_velocity(v, envs, &cons);
  // env_mode.head(envs.size()) = cs_mode;
  // std::cout << "env_mode: " << env_mode.transpose() << std::endl;
  // 0 0 0 0 1 0 1 0 1 0 1 0
  VectorXi env_mode(12);
  env_mode << 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0;

  bool result =
      isQuasidynamic(v, mnps, envs, env_mode, f_ext_w, object_inertia,
                     object_pose, mu_env, mu_mnp, wa, wt, h_time, &cons, 0.3);

  bool result_quasidynamic_lp = isQuasidynamic_LP(
      v, mnps, envs, env_mode, f_ext_w, object_inertia, object_pose, mu_env,
      mu_mnp, wa, wt, h_time, &cons, 0.3);
  std::cout << "result_quasidynamic_lp: " << result_quasidynamic_lp
            << std::endl;

  bool result_quasistatic = isQuasistatic(mnps, envs, env_mode, f_ext_w,
                                          object_pose, mu_env, mu_mnp, &cons);

  std::cout << "sliding result: " << result << std::endl;
  std::cout << "sliding quasistatic result: " << result_quasistatic
            << std::endl;
}

void pivot() {

  Vector6d v1; // feasible
  v1 << 0.182365, 1.44412e-19, 0.526385, -1.11022e-16, 1.18509, -2.08167e-17;
  Vector6d v2; // shoud be feasible, but not now
  v2 << 0.693052, 8.53208e-19, 0.599891, -1.38778e-16, 1.1394, -4.16334e-17;

  DartPointManipulator robot(1, 0.1);
  robot.is_patch_contact = true;

  std::vector<ContactPoint> fingertips;
  fingertips.push_back(
      ContactPoint(Vector3d(-0.5, 0.0489681, 0.45935144), Vector3d(1, 0, 0)));
  std::vector<ContactPoint> mnps;
  robot.Fingertips2PointContacts(fingertips, &mnps);

  std::vector<ContactPoint> envs;
  envs.push_back(
      ContactPoint(Vector3d(0.5, 0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(
      ContactPoint(Vector3d(0.5, -0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(
      ContactPoint(Vector3d(-0.5, -0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(
      ContactPoint(Vector3d(-0.5, 0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));

  VectorXi cs_mode(4);
  cs_mode << 0, 0, 1, 1;

  Vector6d f_ext_w = Vector6d::Zero();
  f_ext_w[2] = -1;
  Matrix6d object_inertia = Matrix6d::Identity();
  object_inertia(3, 3) = 0.167;
  object_inertia(4, 4) = 0.167;
  object_inertia(5, 5) = 0.167;
  Vector7d object_pose;
  object_pose << -0.1, 0, 0.5, 0, 0, 0, 1;
  double mu_env = 0.8;
  double mu_mnp = 0.8;
  double wa = 1;
  double wt = 1;
  double h_time = 10;
  ContactConstraints cons(2);

  VectorXi env_mode_1 = mode_from_velocity(v1, envs, &cons);
  env_mode_1.head(envs.size()) = cs_mode;
  std::cout << "env_mode_1: " << env_mode_1.transpose() << std::endl;

  VectorXi env_mode_2 = mode_from_velocity(v2, envs, &cons);
  env_mode_2.head(envs.size()) = cs_mode;
  std::cout << "env_mode_2: " << env_mode_2.transpose() << std::endl;

  bool result_1 =
      isQuasidynamic(v1, mnps, envs, env_mode_2, f_ext_w, object_inertia,
                     object_pose, mu_env, mu_mnp, wa, wt, h_time, &cons, 0.3);

  bool result_2 =
      isQuasidynamic(v2, mnps, envs, env_mode_2, f_ext_w, object_inertia,
                     object_pose, mu_env, mu_mnp, wa, wt, h_time, &cons, 0.3);

  bool result_quasidynamic_lp1 = isQuasidynamic_LP(
      v1, mnps, envs, env_mode_1, f_ext_w, object_inertia, object_pose, mu_env,
      mu_mnp, wa, wt, h_time, &cons, 0.3);
  std::cout << "result_quasidynamic_lp_1: " << result_quasidynamic_lp1
            << std::endl;

  bool result_quasidynamic_lp2 = isQuasidynamic_LP(
      v2, mnps, envs, env_mode_2, f_ext_w, object_inertia, object_pose, mu_env,
      mu_mnp, wa, wt, h_time, &cons, 0.3);
  std::cout << "result_quasidynamic_lp_2: " << result_quasidynamic_lp2
            << std::endl;
  std::cout << "pivot result_1: " << result_1 << std::endl;
  std::cout << "pivot result_2: " << result_2 << std::endl;
}

void push_up() {
  std::vector<ContactPoint> envs;
  envs.push_back(
      ContactPoint(Vector3d(0.5, 0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(
      ContactPoint(Vector3d(0.5, -0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(
      ContactPoint(Vector3d(-0.5, -0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(
      ContactPoint(Vector3d(-0.5, 0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
  envs.push_back(ContactPoint(Vector3d(-0.5, 0.5, 0.5), Vector3d(1, 0, 0), -0));
  envs.push_back(
      ContactPoint(Vector3d(-0.5, 0.5, -0.5), Vector3d(1, 0, 0), -0));
  envs.push_back(
      ContactPoint(Vector3d(-0.5, -0.5, -0.5), Vector3d(1, 0, 0), -0));
  envs.push_back(
      ContactPoint(Vector3d(-0.5, -0.5, 0.5), Vector3d(1, 0, 0), -0));

  DartPointManipulator robot(1, 0.1);
  robot.is_patch_contact = true;

  std::vector<ContactPoint> fingertips;
  fingertips.push_back(ContactPoint(Vector3d(0.5, 0, 0), Vector3d(-1, 0, 0)));
  std::vector<ContactPoint> mnps;
  robot.Fingertips2PointContacts(fingertips, &mnps);

  Vector6d v;
  v << 0, 0, 0.1, 0, 0, 0;
  //   v << 0,0,0,0,0,0;

  VectorXi cs_mode(8);
  cs_mode << 0, 0, 0, 0, 1, 1, 1, 1;

  Vector6d f_ext_w = Vector6d::Zero();
  f_ext_w[2] = -1;
  Matrix6d object_inertia = Matrix6d::Identity();
  object_inertia(3, 3) = 0.167;
  object_inertia(4, 4) = 0.167;
  object_inertia(5, 5) = 0.167;
  Vector7d object_pose;
  object_pose << -4.5, 0, 0.5, 0, 0, 0, 1;
  double mu_env = 0.3;
  double mu_mnp = 0.8;
  double wa = 1;
  double wt = 1;
  double h_time = 10;
  ContactConstraints cons(2);

  VectorXi env_mode = mode_from_velocity(v, envs, &cons);
  env_mode.head(envs.size()) = cs_mode;

  bool result_quasidynamic =
      isQuasidynamic(v, mnps, envs, env_mode, f_ext_w, object_inertia,
                     object_pose, mu_env, mu_mnp, wa, wt, h_time, &cons, 0.3);

  bool result_quasidynamic_lp = isQuasidynamic_LP(
      v, mnps, envs, env_mode, f_ext_w, object_inertia, object_pose, mu_env,
      mu_mnp, wa, wt, h_time, &cons, 0.3);
  std::cout << "result_quasidynamic_lp: " << result_quasidynamic_lp
            << std::endl;

  bool result_static = isQuasistatic(mnps, envs, env_mode, f_ext_w, object_pose,
                                     mu_env, mu_mnp, &cons);
  std::cout << "push_up result_quasidynamic: " << result_quasidynamic
            << std::endl;
  std::cout << "push_up result_quasidynamic_lp: " << result_quasidynamic_lp
            << std::endl;
  std::cout << "push_up result_static: " << result_static << std::endl;
}
int main() {
  drop();
  rotate();
  slide();
  pivot();
  push_up();
  return 0;
}
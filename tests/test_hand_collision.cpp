

#include "../search/level1.h"

#include "../tasks/cmg_task.h"

#include "../mechanics/utilities/utilities.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

void pivoting(std::shared_ptr<CMGTASK> task) {
  // Test with two fingers and one finger quasidynamics

  double box_length = 0.05;

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", box_length * Vector3d(1, 1, 1));
  SkeletonPtr env1 =
      createFixedBox("ground", Vector3d(2, 2, 0.2), Vector3d(0, 0, -0.1));

  world->addObject(object);
  world->addEnvironmentComponent(env1);

  int n_robot_contacts = 2;
  DartPointManipulator *rpt =
      new DartPointManipulator(n_robot_contacts, box_length * 0.33);
  world->addRobot(rpt);

  // set the task parameters, start, goal, object inertial, etc....

  Vector7d x_start;
  Vector7d x_goal;
  x_start << 0, 0, box_length / 2 * 0.9999, 0, 0, 0, 1;

  // goal: rotate around y axis for 90 degrees
  x_goal << 0.2, 0, box_length / 2 * 0.9999, 0, 0.7071, 0, 0.7071;

  double goal_thr = box_length * 3.14 * 30 / 180;

  double wa = 0.3;
  double wt = 10;

  double mu_env = 0.3;
  double mu_mnp = 0.6;

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  CMGTASK::SearchOptions rrt_options;

  rrt_options.x_ub << 0.5, 1, 0.1;
  rrt_options.x_lb << -0.5, -1, 0.0;

  rrt_options.eps_trans = 0.15;
  rrt_options.eps_angle = 3.14 * 95 / 180;
  rrt_options.max_samples = 20;

  rrt_options.goal_biased_prob = 0.7;

  // pass the world and task parameters to the task through task->initialize
  task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                   mu_mnp, oi, f_g, world, rrt_options);

  // read surface point, add robot contacts

  std::ifstream f(std::string(SRC_DIR) +
                  "/data/test_cmg_cube_reorient90/surface_contacts.csv");
  aria::csv::CsvParser parser(f);

  for (auto &row : parser) {
    int n_cols = row.size();
    assert(n_cols == 6);

    Vector6d v;
    for (int j = 0; j < 6; ++j) {
      v(j) = std::stod(row[j]);
    }
    ContactPoint p(box_length / 2 * v.head(3), v.tail(3));
    task->object_surface_pts.push_back(p);
  }
  // task->task_dynamics_type = CMGTASK::CMG_QUASIDYNAMIC;
  task->number_of_robot_contacts = n_robot_contacts;
}

int main(int argc, char *argv[]) {
  std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

  pivoting(task);
  double box_length = 0.05;

  VectorXd mnp_config(12);
  mnp_config << -0.5*box_length / 2 ,0,1*box_length / 2 ,0,0,-1,-0.5*box_length / 2 ,0,-1*box_length / 2 ,0,0,1;
  Vector7d x_object_now;
  x_object_now << 0.2, -6.0426e-19, 0.024895, 5.12553e-19, 0.707107,
      6.89933e-18, 0.707107;
  Vector7d x_object;
  x_object << 0.15, 0, 0.0249975, 0, 0, 0, 1;

  // update object pose
  task->m_world->updateObjectPose(x_object_now);

  bool ik_now =
      task->m_world->getRobot()->ifIKsolution(mnp_config, x_object_now); // should be 1
  bool collide_now = task->m_world->isRobotCollide(mnp_config); // 0

  task->m_world->updateObjectPose(x_object);

  bool ik_prev = task->m_world->getRobot()->ifIKsolution(mnp_config, x_object); // 1
  bool collide_prev = task->m_world->isRobotCollide(mnp_config); // 1

  std::cout << "ik_now " << ik_now << ", collide_now " << collide_now
            << ", ik_prev " << ik_prev << ", collide_prev " << collide_prev
            << std::endl;
}

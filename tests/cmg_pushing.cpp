

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

void pushing(std::shared_ptr<CMGTASK> task) {
  // create world, create environment, an object sliding on the table

  double box_length = 0.05;

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", box_length * Vector3d(1, 1, 1));
  SkeletonPtr env1 =
      createFixedBox("ground", Vector3d(2, 2, 0.2), Vector3d(0, 0, -0.1));

  world->addObject(object);
  world->addEnvironmentComponent(env1);

  int n_robot_contacts = 1;
  DartPointManipulator *rpt =
      new DartPointManipulator(n_robot_contacts, box_length * 0.2);
  world->addRobot(rpt);

  // set the task parameters, start, goal, object inertial, etc....

  Vector7d x_start;
  Vector7d x_goal;
  x_start << 0, 0, box_length / 2 * 0.9999, 0, 0, 0, 1;
  x_goal << 0.48, 0, box_length / 2 * 0.9999, 0, 0, 0, 1;
  // x_goal << 0.1, 0, box_length/2 * 0.9999, 0, 0.7071, 0, 0.7071;

  double goal_thr = box_length * 3.14 * 30 / 180;

  double wa = 0.3;
  double wt = 10;

  double mu_env = 0.4;
  double mu_mnp = 0.8;

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  CMGTASK::SearchOptions rrt_options;

  rrt_options.x_ub << 1, 1, 0.1;
  rrt_options.x_lb << -1, -1, 0.0;

  rrt_options.eps_trans = 0.15;
  rrt_options.eps_angle = 3.14 * 95 / 180;
  rrt_options.max_samples = 20;

  rrt_options.goal_biased_prob = 0.7;

  // pass the world and task parameters to the task through task->initialize
  task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                   mu_mnp, oi, f_g, world, n_robot_contacts, CMG_QUASISTATIC,
                   rrt_options);

  // read surface point, add robot contacts

  std::ifstream f(std::string(SRC_DIR) +
                  "/data/test_cmg_pushing/surface_contacts.csv");
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
}

int main(int argc, char *argv[]) {
  std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

  pushing(task);

  CMGTASK::State start_state = task->get_start_state();

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2,
                  CMGTASK>::HierarchicalComputeOptions compute_options;

  compute_options.l1_1st.max_iterations = 100;
  compute_options.l1.max_iterations = 20;
  compute_options.l2_1st.max_iterations = 1000;
  compute_options.l2.max_iterations = 300;
  compute_options.final_l2_1st.max_iterations = 10000;
  compute_options.final_l2.max_iterations = 3000;

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2, CMGTASK> tree(
      task, start_state, compute_options);

  HMP::Node<CMGTASK::State> *current_node = tree.search_tree();

  std::vector<CMGTASK::State> object_trajectory;
  std::vector<CMGTASK::State2> action_trajectory;
  tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

  std::vector<Vector7d> object_traj;

  std::cout << "Best value " << current_node->m_value << std::endl;

  for (int kk = 0; kk < object_trajectory.size(); ++kk) {
    std::cout << "Timestep " << kk << std::endl;
    std::cout << "Pose " << object_trajectory[kk].m_pose.transpose()
              << std::endl;
    std::cout << "Fingers ";
    for (int jj :
         task->get_finger_locations(action_trajectory[kk].finger_index)) {
      std::cout << jj << " ";
    }
    std::cout << std::endl;
    // object_traj.push_back(object_trajectory[kk].m_pose);
    object_traj.insert(object_traj.end(), object_trajectory[kk].m_path.begin(),
                       object_trajectory[kk].m_path.end());
  }

  std::cout << "Total level 1 tree nodes " << tree.count_total_nodes()
            << std::endl;

  std::cout << "Total shared rrt nodes " << tree.m_task->total_rrt_nodes()
            << std::endl;

  // world->setObjectTrajectory(object_traj);
  // world->startWindow(&argc, argv);
}

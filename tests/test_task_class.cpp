

#include "../search/level1mcts.h"

#include "../tasks/task.h"

#include "../mechanics/utilities/utilities.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

#include "visualization.h"

const TASK::State2::Action TASK::State2::no_action = TASK::State2::Action(-1, -1);
const TASK::State::Action TASK::State::no_action = -1;

void card(std::shared_ptr<TASK> task) {
  // Test with two fingers and one finger quasidynamics

  double box_length = 2.0;
  double box_height = 0.5;

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", Vector3d(box_length, box_length, box_height));
  SkeletonPtr env1 =
      createFixedBox("ground", Vector3d(4, 4, 0.2), Vector3d(0, 0, -0.1));

  world->addObject(object);
  world->addEnvironmentComponent(env1);

  int n_robot_contacts = 2;
  DartPointManipulator *rpt = new DartPointManipulator(n_robot_contacts, 0.15);
  rpt->is_patch_contact = true;
  world->addRobot(rpt);

  // set the task parameters, start, goal, object inertial, etc....

  Vector7d x_start;
  Vector7d x_goal;
  x_start << 0, 0, box_height / 2 * 0.9999, 0, 0, 0, 1;
  // x_start << 0.272219, -0.759927, 0.249975, -3.54756e-17, -1.54323e-16,
  //     -0.427257, 0.90413;

  // goal: rotate around y axis for 90 degrees
  x_goal << 2.5, 0, box_length/2 * 1.5, 0, -0.7071, 0, 0.7071;
  // x_goal << 0.5, 0, box_length / 2 * 1.5, 0, -0.7071, 0, 0.7071;

  double goal_thr = box_length * 3.14 * 10 / 180;

  double wa = 0.4;
  double wt = 1.0;

  double mu_env = 0.2;
  double mu_mnp = 0.7;

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  TASK::SearchOptions rrt_options;

  rrt_options.x_ub << 5.0, 1.0, box_height;
  rrt_options.x_lb << -1.0, -1.0, 0.0;

  rrt_options.eps_trans = 0.75;
  // rrt_options.eps_angle = 3.14 * 95 / 180;
  // rrt_options.eps_trans = 0.10;
  rrt_options.eps_angle = 3.14 * 35 / 180;
  rrt_options.max_samples = 500;

  rrt_options.goal_biased_prob = 0.5;

  bool if_refine = false;
  bool refine_dist = 0.5;

  std::vector<ContactPoint> surface_pts;

  // read surface point, add robot contacts

  std::ifstream f(std::string(SRC_DIR) +
                  "/data/test_cmg_pick_card/surface_contacts.csv");
  aria::csv::CsvParser parser(f);

  for (auto &row : parser) {
    int n_cols = row.size();
    assert(n_cols == 6);

    Vector6d v;
    for (int j = 0; j < 6; ++j) {
      v(j) = std::stod(row[j]);
    }
    ContactPoint p(v.head(3), v.tail(3));
    surface_pts.push_back(p);
  }

  // pass the world and task parameters to the task through task->initialize
  task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                   mu_mnp, oi, f_g, world, n_robot_contacts, "quasistatic",
                   surface_pts, rrt_options, if_refine, refine_dist);
}

int main(int argc, char *argv[]) {
  std::shared_ptr<TASK> task = std::make_shared<TASK>();

  card(task);

  TASK::State start_state = task->get_start_state();

  HMP::Level1MCTS<TASK::State, TASK::State2,
                  TASK>::HierarchicalComputeOptions compute_options;

  compute_options.l1_1st.max_iterations = 5;
  compute_options.l1.max_iterations = 3;
  compute_options.l2_1st.max_iterations = 20;
  compute_options.l2.max_iterations = 5;
  compute_options.final_l2_1st.max_iterations = 20;
  compute_options.final_l2.max_iterations = 5;

  HMP::Level1MCTS<TASK::State, TASK::State2, TASK> tree(
      task, start_state, compute_options);

  HMP::Node<TASK::State> *current_node = tree.search_tree();

  std::vector<TASK::State> object_trajectory;
  std::vector<TASK::State2> action_trajectory;
  tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

  std::vector<Vector7d> object_traj;
  std::vector<VectorXd> mnp_traj;

  std::cout << "Best value " << current_node->m_value << std::endl;

  // std::cout << object_trajectory.size() << std::endl;
  // std::cout << action_trajectory.size() << std::endl;

  // for (int kk = 0; kk < object_trajectory.size(); ++kk)
  // {
  //   std::cout << "Timestep " << kk << std::endl;
  //   std::cout << "Pose " << object_trajectory[kk].m_pose.transpose()
  //             << std::endl;
  //   std::cout << "# envs " << object_trajectory[kk].envs.size() << std::endl;
  //   std::cout << "action " << action_trajectory[kk].finger_index <<
  //   std::endl; std::cout << "Fingers "; for (int jj:
  //   task->get_finger_locations(action_trajectory[kk].finger_index)){
  //     std::cout << jj << " ";
  //   }
  //   std::cout << std::endl;
  //   // object_traj.push_back(object_trajectory[kk].m_pose);
  //   object_traj.insert(object_traj.end(),
  //   object_trajectory[kk].m_path.begin(),
  //                      object_trajectory[kk].m_path.end());
  //   for(int i = 0; i < object_trajectory[kk].m_path.size(); i++){
  //     mnp_traj.push_back(task->get_robot_config_from_action_idx(action_trajectory[kk].finger_index));
  //   }
  // }
  std::cout << "Total timesteps" << object_trajectory.size() << std::endl;

  for (auto &action : action_trajectory) {
    std::cout << "Timestep " << action.timestep << std::endl;
    std::cout
        << "Pose "
        << task->saved_object_trajectory[action.timestep].m_pose.transpose()
        << std::endl;
    std::cout << "Fingers ";
    std::cout << "Tmax " << action.t_max << std::endl;
    for (int jj : task->get_finger_locations(action.finger_index)) {
      std::cout << jj << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "Total level 1 tree nodes " << tree.count_total_nodes()
            << std::endl;

  std::cout << "Total shared rrt nodes " << tree.m_task->total_rrt_nodes()
            << std::endl;

  VializeStateTraj(task->m_world, task, object_trajectory, action_trajectory);
  // VisualizeTraj(task->m_world, object_traj, mnp_traj);
  task->m_world->startWindow(&argc, argv);
}
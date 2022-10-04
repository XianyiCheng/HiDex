

#include "../search/level1.h"

#include "../tasks/cmg_task.h"

#include "../mechanics/utilities/utilities.h"

#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

int main(int argc, char* argv[]) {

  std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

  // create world, create environment, an object sliding on the table

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object = createFreeBox("box_object", 0.05 * Vector3d(1, 1, 1));
  SkeletonPtr env1 =
      createFixedBox("ground", Vector3d(2, 2, 0.2), Vector3d(0, 0, -0.1));

  world->addObject(object);
  world->addEnvironmentComponent(env1);

  // set the task parameters, start, goal, object inertial, etc....

  Vector7d x_start;
  Vector7d x_goal;
  x_start << 0, 0, 0.025 * 0.9999, 0, 0, 0, 1;
  // x_goal << 0.48, 0, 0.025 * 0.9999, 0, 0, 0, 1;
  x_goal << 0.1, 0, 0.025 * 0.9999, 0, 0.7071, 0, 0.7071;


  double goal_thr = 0.05 * 3.14 * 30 / 180;

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

  rrt_options.eps_trans = 0.5;
  rrt_options.eps_angle = 3.14 * 95 / 180;
  rrt_options.max_samples = 20;

  rrt_options.goal_biased_prob = 0.7;

  // pass the world and task parameters to the task through task->initialize
  task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                   mu_mnp, oi, f_g, world, rrt_options);
  
  // read surface point, add robot contacts

  CMGTASK::State start_state = task->get_start_state();

  HMP::Level1Tree<CMGTASK::State,CMGTASK::State2, CMGTASK> tree(task, start_state);

  HMP::MCTSOptions compute_options;
  compute_options.max_iterations =
      20; // maximum iteration for search from the root node

  HMP::Node<CMGTASK::State> *current_node = tree.search_tree(compute_options);

  std::vector<CMGTASK::State> object_trajectory;
  std::vector<CMGTASK::State2> action_trajectory;
  tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

  std::vector<Vector7d> object_traj;

  for (int kk = 0; kk < object_trajectory.size(); ++kk){
    std::cout << "Timestep " << kk << std::endl;
    std::cout << "Pose " << object_trajectory[kk].m_pose.transpose() << std::endl;
    std::cout << "Action " << action_trajectory[kk].finger_index << std::endl;
    object_traj.push_back(object_trajectory[kk].m_pose);
  }
  // // backtrack the tree to get the path
  // while (current_node != nullptr) {
  //   std::cout << "Node type: " << current_node->m_type << " Pose "
  //             << current_node->m_state.m_pose.transpose()
  //             << " Value: " << current_node->m_value
  //             << " Visits: " << current_node->m_visits << std::endl;
  //   if (current_node->m_type == "mode"){
  //     std::cout << "Mode " << current_node->m_state.modes[current_node->m_action].transpose() << std::endl;
  //   }
  //   object_traj.push_back(current_node->m_state.m_pose);
  //   current_node = current_node->m_parent;
  // }
  world->setObjectTrajectory(object_traj);

  world->startWindow(&argc, argv);
  
}
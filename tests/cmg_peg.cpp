

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

#include "visualization.h"

void peg(std::shared_ptr<CMGTASK> task) {
  // Test with two fingers and one finger quasidynamics

  double box_length = 2.0;
  double box_height = 4.0;

  double wall_width = 1.0;

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  double gap = 0.1;

  SkeletonPtr object =
      createFreeBox("box_object", Vector3d(box_length, box_length, box_height));

  SkeletonPtr wall1 =
      createFixedBox("wall1", Vector3d(wall_width, box_length + wall_width*2, box_height),
                     Vector3d(-(box_length/2 + gap + wall_width/2), 0, box_height / 2));

  SkeletonPtr wall2 =
      createFixedBox("wall2", Vector3d(wall_width, box_length + wall_width*2, box_height),
                     Vector3d(box_length/2 + gap + wall_width/2, 0, box_height / 2));

  SkeletonPtr wall3 = createFixedBox("wall3", Vector3d(box_length + wall_width*2, wall_width, box_height),
                     Vector3d(0, box_length/2 + gap + wall_width/2, box_height / 2));

  SkeletonPtr wall4 = createFixedBox("wall4", Vector3d(box_length + wall_width*2, wall_width, box_height),
                     Vector3d(0, -(box_length/2 + gap + wall_width/2), box_height / 2));

  SkeletonPtr ground =
      createFixedBox("ground", Vector3d(10, 10, 1), Vector3d(0, 0, 1e-4 - 0.5));

  world->addObject(object);
  // world->addEnvironmentComponent(wall1);
  // world->addEnvironmentComponent(wall2);
  // world->addEnvironmentComponent(wall3);
  // world->addEnvironmentComponent(wall4);
  world->addEnvironmentComponent(ground);

  int n_robot_contacts = 3;
  DartPointManipulator *rpt =
      new DartPointManipulator(n_robot_contacts, gap * 1.5);
  rpt->is_patch_contact = true;
  world->addRobot(rpt);

  // set the task parameters, start, goal, object inertial, etc....

  Vector7d x_start;
  Vector7d x_goal;
  x_start << 0, 0, box_height / 2, 0, 0, 0, 1;

  x_goal << 0, 0, box_height*2.1, 0, 0, 0, 1;


  double goal_thr = box_length * 3.14 * 10 / 180;

  double wa = 1.0;
  double wt = 1.0;

  double mu_env = 0.1;
  double mu_mnp = 0.9;

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  CMGTASK::SearchOptions rrt_options;

  rrt_options.x_ub << box_length, box_length, box_height * 2;
  rrt_options.x_lb << -box_length, -box_length, 0.0;

  rrt_options.eps_trans = 2.0;
  // rrt_options.eps_angle = 3.14 * 95 / 180;
  // rrt_options.eps_trans = 0.10;
  rrt_options.eps_angle = 3.14 * 35 / 180;
  rrt_options.max_samples = 50;
  // rrt_options.sampleSO3 = false;
  // rrt_options.sample_rotation_axis << 1, 0, 0;

  rrt_options.goal_biased_prob = 0.7;

  bool if_refine = false;
  bool refine_dist = 0.5;

  // read surface point, add robot contacts
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) +
                  "/data/test_cmg_peg/surface_contacts.csv");
  aria::csv::CsvParser parser(f);

  for (auto &row : parser) {
    int n_cols = row.size();
    assert(n_cols == 6);

    Vector6d v;
    for (int j = 0; j < 6; ++j) {
      v(j) = std::stod(row[j]);
    }
    ContactPoint p(Vector3d(v[0], v[1], v[2]), v.tail(3));
    surface_pts.push_back(p);
  }
  // pass the world and task parameters to the task through task->initialize
  task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                   mu_mnp, oi, f_g, world, n_robot_contacts, CMG_QUASISTATIC,
                   surface_pts, rrt_options, if_refine, refine_dist);
//   VisualizeSG(task->m_world, x_start, x_goal);
}

int main(int argc, char *argv[]) {
  std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

  peg(task);

  // for (int k = 0; k < task->n_finger_combinations; ++k){
  //   std::cout << "action idx : " << k << ", ";

  // for (auto p: task->get_finger_locations(k)){
  //   std::cout << p << " ";

  // }
  // std::cout << std::endl;
  // }

  // for (int idx = 0; idx < task->n_finger_combinations; idx++){
  //   std::vector<int> ff = task->get_finger_locations(idx);
  //   std::cout << "idx: " << idx << " , locations: " ;
  //   for (auto k : ff){
  //     std::cout << k << " ";
  //   }
  //   std::cout << std::endl;
  // }
//   task->m_world->startWindow(&argc, argv);

  CMGTASK::State start_state = task->get_start_state();

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2,
                  CMGTASK>::HierarchicalComputeOptions compute_options;

  compute_options.l1_1st.max_iterations = 20;
  compute_options.l1.max_iterations = 5;
  compute_options.l2_1st.max_iterations = 20000;
  compute_options.l2.max_iterations = 500;
  compute_options.final_l2_1st.max_iterations = 50000;
  compute_options.final_l2.max_iterations = 500;

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2, CMGTASK> tree(
      task, start_state, compute_options);

  HMP::Node<CMGTASK::State> *current_node = tree.search_tree();

  std::vector<CMGTASK::State> object_trajectory;
  std::vector<CMGTASK::State2> action_trajectory;
  tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

  std::vector<Vector7d> object_traj;
  std::vector<VectorXd> mnp_traj;

  std::cout << "Best value " << current_node->m_value << std::endl;

  std::cout << object_trajectory.size() << std::endl;
  std::cout << action_trajectory.size() << std::endl;

  for (int kk = 0; kk < object_trajectory.size(); ++kk) {
    std::cout << "Timestep " << kk << std::endl;
    std::cout << "Pose " << object_trajectory[kk].m_pose.transpose()
              << std::endl;
    std::cout << "# envs " << object_trajectory[kk].envs.size() << std::endl;
    std::cout << "action " << action_trajectory[kk].finger_index << std::endl;
    std::cout << "Fingers ";
    for (int jj :
         task->get_finger_locations(action_trajectory[kk].finger_index)) {
      std::cout << jj << " ";
    }
    std::cout << std::endl;
    // object_traj.push_back(object_trajectory[kk].m_pose);
    object_traj.insert(object_traj.end(), object_trajectory[kk].m_path.begin(),
                       object_trajectory[kk].m_path.end());
    for (int i = 0; i < object_trajectory[kk].m_path.size(); i++) {
      mnp_traj.push_back(task->get_robot_config_from_action_idx(
          action_trajectory[kk].finger_index));
    }
  }

  std::cout << "Total level 1 tree nodes " << tree.count_total_nodes()
            << std::endl;

  std::cout << "Total shared rrt nodes " << tree.m_task->total_rrt_nodes()
            << std::endl;

  VisualizeTraj(task->m_world, object_traj, mnp_traj);

  task->m_world->startWindow(&argc, argv);
}

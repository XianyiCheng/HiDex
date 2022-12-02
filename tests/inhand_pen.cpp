
#include "../search/level1.h"

#include "../tasks/inhand_task.h"

#include "../mechanics/utilities/utilities.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

#include "visualization.h"

void cube(std::shared_ptr<InhandTASK> task) {
  // create world, create environment, an object sliding on the table

  double box_lx = 8;
  double box_ly = 0.5;
  double box_lz = 0.5;

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", Vector3d(box_lx, box_ly, box_lz));

  world->addObject(object);

  SkeletonPtr env1 =
      createFixedBox("ground", Vector3d(5, 5, 0.2), Vector3d(0, 0, -2));

  world->addEnvironmentComponent(env1);

  int n_robot_contacts = 5;
  DartPointManipulator *rpt = new DartPointManipulator(n_robot_contacts, 0.1);

  std::vector<Vector6d> workspace_limits;
  {
    // [x_min, x_max, y_min, y_max, z_min, z_max]
    Vector6d wl1;
    wl1 << -3, 0,
        0, 1.5,
        -1, 1;
    workspace_limits.push_back(wl1);

    Vector6d wl2;
    wl2 << -2, 1,
        0, 1.5,
        -1, 1;
    workspace_limits.push_back(wl2);

    Vector6d wl3;
    wl3 << -1, 2,
        0, 1.5,
        -1, 1;
    workspace_limits.push_back(wl3);

    Vector6d wl4;
    wl4 << 0, 3,
        0, 1.5,
        -1, 1;
    workspace_limits.push_back(wl4);


    Vector6d wl5;
    wl5 << 0, 3.5,
        -1.5, 1,
        -1, 1;
    workspace_limits.push_back(wl5);
  }

  rpt->set_workspace_limit(workspace_limits);
  rpt->is_patch_contact = true;
  world->addRobot(rpt);

  // set the task parameters, start, goal, object inertial, etc....

  Vector7d x_start;
  Vector7d x_goal;
  x_start << 0, 0, 0, 0, 0, 0, 1;
  //   x_goal << 0, 0, 0, 0, 0, 0.7071, 0.7071;
  x_goal << 0, 0, 0, 0, 0, -1, 0;

  int start_finger_idx = -1;
  int goal_finger_idx = -1;

  double goal_thr = 3.14 * 5 / 180;

  double wa = 1;
  double wt = 1;

  double mu_env = 0.2;
  double mu_mnp = 0.8;

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  InhandTASK::SearchOptions rrt_options;

  rrt_options.x_ub << 4, 0, 4;
  rrt_options.x_lb << -3, 0, 0;

  rrt_options.eps_trans = 0.2;
  rrt_options.eps_angle = 3.14 * 15 / 180;
  rrt_options.max_samples = 50;

  rrt_options.goal_biased_prob = 0.8;

  bool is_refine = false;
  double refine_dist = 0.1;

  // read surface point, add robot contacts
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) +
                  "/data/test_inhand_pen/surface_contacts.csv");
  aria::csv::CsvParser parser(f);

  for (auto &row : parser) {
    int n_cols = row.size();
    assert(n_cols == 6);

    Vector6d v;
    for (int j = 0; j < 6; ++j) {
      v(j) = std::stod(row[j]);
    }
    // only use contacts have normals in y direction
    if ((v[4] > -0.8) && (v[4] < 0.8)) {
      continue;
    }
    Vector3d pos;
    pos << v(0) * box_lx / 2, v(1) * box_ly / 2, v(2) * box_lz / 2;
    ContactPoint p(pos, v.tail(3));
    surface_pts.push_back(p);
  }
  std::cout << "surface pts: " << surface_pts.size() << std::endl;
  // pass the world and task parameters to the task through task->initialize

  task->initialize(x_start, x_goal, start_finger_idx, goal_finger_idx, goal_thr,
                   wa, wt, charac_len, mu_env, mu_mnp, f_g, world,
                   n_robot_contacts, surface_pts, rrt_options, is_refine,
                   refine_dist);

    // VisualizeSG(task->m_world, x_start, x_goal);
}

void test_finger_idx(std::shared_ptr<InhandTASK> task) {
  std::cout << task->n_finger_combinations << std::endl;
  for (int iter = 0; iter < 100; iter++) {
    int idx = randi(task->n_finger_combinations);

    std::vector<int> locs = task->get_finger_locations(idx);

    int idx2 = task->finger_locations_to_finger_idx(locs);

    std::cout << "idx: " << idx << " idx2: " << idx2 << ", if equal "
              << (idx == idx2) << std::endl;
  }
}

int main(int argc, char *argv[]) {
  std::shared_ptr<InhandTASK> task = std::make_shared<InhandTASK>();

  cube(task);

  InhandTASK::State start_state = task->get_start_state();

  HMP::Level1Tree<InhandTASK::State, InhandTASK::State2,
                  InhandTASK>::HierarchicalComputeOptions compute_options;

  compute_options.l1_1st.max_iterations = 10;
  compute_options.l1.max_iterations = 1;
  compute_options.l2_1st.max_iterations = 10;
  compute_options.l2.max_iterations = 2;
  compute_options.final_l2_1st.max_iterations = 10;
  compute_options.final_l2.max_iterations = 3;

  compute_options.l1.max_time = 10;

  HMP::Level1Tree<InhandTASK::State, InhandTASK::State2, InhandTASK> tree(
      task, start_state, compute_options);

  HMP::Node<InhandTASK::State> *current_node = tree.search_tree();

  std::vector<InhandTASK::State> object_trajectory;
  std::vector<InhandTASK::State2> action_trajectory;
  tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

  std::vector<Vector7d> object_traj;
  std::vector<VectorXd> mnp_traj;

  std::cout << "Best value " << current_node->m_value << std::endl;

  for (auto &action : action_trajectory) {
    std::cout << "Timestep " << action.timestep << std::endl;
    std::cout
        << "Pose "
        << task->saved_object_trajectory[action.timestep].m_pose.transpose()
        << std::endl;
    std::cout << "Fingers ";
    for (int jj : task->get_finger_locations(action.finger_index)) {
      std::cout << jj << " ";
    }
    std::cout << std::endl;
  }

  VializeStateTraj(task->m_world, task, object_trajectory, action_trajectory);

  std::cout << "Total level 1 tree nodes " << tree.count_total_nodes()
            << std::endl;

  std::cout << "Total shared rrt nodes " << tree.m_task->total_rrt_nodes()
            << std::endl;

  task->m_world->startWindow(&argc, argv);
}

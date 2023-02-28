
#include <yaml-cpp/yaml.h>

#include "../search/level1.h"

#include "../tasks/inhand_task.h"

#include "../mechanics/utilities/sample.h"
#include "../mechanics/utilities/utilities.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#include "../mechanics/utilities/io.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

// #include "visualization.h"

#include "../tasks/visualization.h"

Vector7d randomize_a_pose_on_palm(const std::vector<ContactPoint> pts) {
  Quaterniond q_rand = generate_unit_quaternion();
  Vector7d x;
  x << 0, 0, 0, q_rand.x(), q_rand.y(), q_rand.z(), q_rand.w();

  Eigen::Matrix3d R;
  R = quat2SO3(x(6), x(3), x(4), x(5));

  double min_z = 0;
  for (auto pt : pts) {
    Vector3d pp = R * pt.p;
    if (pp[2] < min_z)
      min_z = pp[2];
  }
  x[2] = -min_z;
  return x;
}

Vector7d randomize_a_pose(double start_x, double start_y, double start_z) {
  Quaterniond q_rand = generate_unit_quaternion();
  Vector7d x;
  x << start_x, start_y, start_z, q_rand.x(), q_rand.y(), q_rand.z(),
      q_rand.w();
  return x;
}

// This script is used to setup planning inhand manipulation experiments with
// different objects and different hand configs.

const InhandTASK::State2::Action InhandTASK::State2::no_action =
    InhandTASK::State2::Action(-1, -1);
const InhandTASK::State::Action InhandTASK::State::no_action = -1;

void setup(const std::string &hand_type, double finger_radius,
           const std::string &object_name, std::shared_ptr<InhandTASK> task,
           double start_x, double start_y, double start_z, double scale,
           double grasp_measure_scale) {
  // create world, create environment, an object sliding on the table

  // all data are store in the data folder under inhand_all

  // read surface point
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) + "/data/inhand_all/data/" +
                  object_name + ".csv");
  aria::csv::CsvParser parser(f);

  for (auto &row : parser) {
    int n_cols = row.size();
    assert(n_cols == 6);

    Vector6d v;
    for (int j = 0; j < 6; ++j) {
      v(j) = std::stod(row[j]);
    }
    Vector3d pos;
    pos << v(0) * scale, v(1) * scale, v(2) * scale;
    ContactPoint p(pos, v.tail(3));
    surface_pts.push_back(p);
  }

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object = createFreeObjectfromMesh(
      object_name,
      std::string(SRC_DIR) + "/data/inhand_all/data/" + object_name + ".stl",
      Vector3d(scale, scale, scale));

  world->addObject(object);

  int n_robot_contacts;

  Vector7d x_start;
  Vector7d x_goal;

  if (hand_type == "2x2") {
    // x y z dimension of the best object fit
    double box_lx = 1;
    double box_ly = 1;
    double box_lz = 1;

    SkeletonPtr env1 =
        createFixedBox("ground", Vector3d(5, 5, 0.2), Vector3d(0, 0, -1000),
                       Vector3d(0.9, 0.9, 0.9), 0.01);

    n_robot_contacts = 4;
    DartPointManipulator *rpt =
        new DartPointManipulator(n_robot_contacts, finger_radius);

    std::vector<Vector6d> workspace_limits;
    {
      // [x_min, x_max, y_min, y_max, z_min, z_max]
      Vector6d wl1;
      wl1 << -box_lx / 2 - box_lx * 0.25, -box_lx / 2 + box_lx * 0.25,
          -1.25 * box_ly / 2, 1.25 * box_ly / 2, -1.25 * box_lz / 2,
          1.25 * box_lz / 2;
      workspace_limits.push_back(wl1);

      Vector6d wl2;
      wl2 << box_lx / 2 - box_lx * 0.25, box_lx / 2 + box_lx * 0.25,
          -1.25 * box_ly / 2, 1.25 * box_ly / 2, -1.25 * box_lz / 2,
          1.25 * box_lz / 2;
      workspace_limits.push_back(wl2);

      Vector6d wl3;
      wl3 << -1.25 * box_lx / 2, 1.25 * box_lx / 2, -box_ly / 2 - box_ly * 0.25,
          -box_ly / 2 + box_ly * 0.25, -1.25 * box_lz / 2, 1.25 * box_lz / 2;
      workspace_limits.push_back(wl3);

      Vector6d wl4;
      wl4 << -1.25 * box_lx / 2, 1.25 * box_lx / 2, box_ly / 2 - box_ly * 0.25,
          box_ly / 2 + box_ly * 0.25, -1.25 * box_lz / 2, 1.25 * box_lz / 2;
      workspace_limits.push_back(wl4);
    }

    rpt->set_workspace_limit(workspace_limits);
    rpt->is_patch_contact = true;

    world->addEnvironmentComponent(env1);
    world->addRobot(rpt);

    x_start = randomize_a_pose(start_x, start_y, start_z);
    x_goal = randomize_a_pose(start_x, start_y, start_z);
  } else if (hand_type == "5") {
    SkeletonPtr env1 =
        createFixedBox("ground", Vector3d(5, 5, 0.2), Vector3d(0, 0, -1000),
                       Vector3d(0.9, 0.9, 0.9), 0.01);
    n_robot_contacts = 5;
    DartPointManipulator *rpt =
        new DartPointManipulator(n_robot_contacts, finger_radius);

    std::vector<Vector6d> workspace_limits;
    {
      // [x_min, x_max, y_min, y_max, z_min, z_max]
      Vector6d wl1;
      wl1 << -3, 0, 0, 1.5, -1, 1;
      workspace_limits.push_back(wl1);

      Vector6d wl2;
      wl2 << -2, 1, 0, 1.5, -1, 1;
      workspace_limits.push_back(wl2);

      Vector6d wl3;
      wl3 << -1, 2, 0, 1.5, -1, 1;
      workspace_limits.push_back(wl3);

      Vector6d wl4;
      wl4 << 0, 3, 0, 1.5, -1, 1;
      workspace_limits.push_back(wl4);

      Vector6d wl5;
      wl5 << 0, 3.5, -1.5, 1, -1, 1;
      workspace_limits.push_back(wl5);
    }

    rpt->set_workspace_limit(workspace_limits);
    rpt->is_patch_contact = true;

    world->addEnvironmentComponent(env1);
    world->addRobot(rpt);

    x_start = randomize_a_pose(start_x, start_y, start_z);
    x_goal = randomize_a_pose(start_x, start_y, start_z);
  } else if (hand_type == "palm_2") {
    SkeletonPtr env1 =
        createFixedBox("palm", Vector3d(3, 3, 0.2), Vector3d(0, 0, -0.1),
                       Vector3d(161.0 / 256.0, 159.0 / 256.0, 204 / 256.0));

    world->addEnvironmentComponent(env1);

    n_robot_contacts = 2;
    DartPointManipulator *rpt =
        new DartPointManipulator(n_robot_contacts, finger_radius);

    double a = 0.8;
    double h = 2;
    std::vector<Vector6d> workspace_limits;
    Vector3d p1(1.2, 0, 0);
    Vector3d p2(-1.2, 0, 0);

    a = 1;

    // [x_min, x_max, y_min, y_max, z_min, z_max]
    Vector6d wl1;
    wl1 << p1[0] - a, p1[0] + a, p1[1] - a, p1[1] + a, p1[2], p1[2] + h;
    workspace_limits.push_back(wl1);

    Vector6d wl2;
    wl2 << p2[0] - a, p2[0] + a, p2[1] - a, p2[1] + a, p2[2], p2[2] + h;
    workspace_limits.push_back(wl2);

    rpt->set_workspace_limit(workspace_limits);
    rpt->is_patch_contact = true;
    world->addRobot(rpt);
    task->if_transition_pruning = true;

    x_start = randomize_a_pose_on_palm(surface_pts);
    x_goal = randomize_a_pose_on_palm(surface_pts);
  } else if (hand_type == "palm_3") {
    SkeletonPtr env1 =
        createFixedBox("palm", Vector3d(3, 3, 0.2), Vector3d(0, 0, -0.1),
                       Vector3d(161.0 / 256.0, 159.0 / 256.0, 204 / 256.0));
    // SkeletonPtr env1 =
    //     createFixedBox("palm", Vector3d(3, 3, 0.2), Vector3d(0, 0, -1000),
    //                    Vector3d(161.0 / 256.0, 159.0 / 256.0, 204 / 256.0));

    world->addEnvironmentComponent(env1);

    n_robot_contacts = 3;
    DartPointManipulator *rpt =
        new DartPointManipulator(n_robot_contacts, finger_radius);

    double a = 0.8;
    double h = 2;
    std::vector<Vector6d> workspace_limits;

    Vector3d p1(0, 1, 0);
    Vector3d p2(1.73 / 2, -0.5, 0);
    Vector3d p3(-1.73 / 2, -0.5, 0);

    // [x_min, x_max, y_min, y_max, z_min, z_max]

    Vector6d wl1;
    wl1 << p1[0] - a, p1[0] + a, p1[1] - a, p1[1] + a, p1[2], p1[2] + h;
    workspace_limits.push_back(wl1);

    Vector6d wl2;
    wl2 << p2[0] - a, p2[0] + a, p2[1] - a, p2[1] + a, p2[2], p2[2] + h;
    workspace_limits.push_back(wl2);

    Vector6d wl3;
    wl3 << p3[0] - a, p3[0] + a, p3[1] - a, p3[1] + a, p3[2], p3[2] + h;
    workspace_limits.push_back(wl3);

    rpt->set_workspace_limit(workspace_limits);
    rpt->is_patch_contact = true;
    world->addRobot(rpt);
    task->if_transition_pruning = true;

    x_start = randomize_a_pose_on_palm(surface_pts);
    x_start[2] += 0.1;
    x_goal = randomize_a_pose_on_palm(surface_pts);
    x_goal[2] += 0.1;
  } else if (hand_type == "3_finger") {

    SkeletonPtr env1 =
        createFixedBox("palm", Vector3d(3, 3, 0.2), Vector3d(0, 0, -1000),
                       Vector3d(0.9, 0.9, 0.9), 0.01);

    world->addEnvironmentComponent(env1);

    n_robot_contacts = 3;
    DartPointManipulator *rpt =
        new DartPointManipulator(n_robot_contacts, finger_radius);

    double a = 0.8;
    double h = 2;
    std::vector<Vector6d> workspace_limits;

    Vector3d p1(0, 1, 0);
    Vector3d p2(1.73 / 2, -0.5, 0);
    Vector3d p3(-1.73 / 2, -0.5, 0);

    // [x_min, x_max, y_min, y_max, z_min, z_max]

    Vector6d wl1;
    wl1 << p1[0] - a, p1[0] + a, p1[1] - a, p1[1] + a, p1[2], p1[2] + h;
    workspace_limits.push_back(wl1);

    Vector6d wl2;
    wl2 << p2[0] - a, p2[0] + a, p2[1] - a, p2[1] + a, p2[2], p2[2] + h;
    workspace_limits.push_back(wl2);

    Vector6d wl3;
    wl3 << p3[0] - a, p3[0] + a, p3[1] - a, p3[1] + a, p3[2], p3[2] + h;
    workspace_limits.push_back(wl3);

    rpt->set_workspace_limit(workspace_limits);
    rpt->is_patch_contact = true;
    world->addRobot(rpt);
    task->if_transition_pruning = true;

    x_start = randomize_a_pose_on_palm(surface_pts);
    x_goal = randomize_a_pose_on_palm(surface_pts);

  } else {
    std::cout << "unknown hand type" << std::endl;
    exit(0);
  }

  // set the task parameters, start, goal, object inertial, etc....

  // x_start << start_x, start_y, start_z, 0, 0, 0, 1;
  // x_goal << start_x, start_y, start_z, 0, 0, -1, 0;

  long int start_finger_idx = -1;
  long int goal_finger_idx = -1;

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
  rrt_options.eps_angle = 3.14 * 20 / 180;
  rrt_options.max_samples = 100;

  rrt_options.goal_biased_prob = 0.8;

  bool is_refine = false;
  double refine_dist = 0.1;

  // pass the world and task parameters to the task through task->initialize

  task->initialize(x_start, x_goal, start_finger_idx, goal_finger_idx, goal_thr,
                   wa, wt, charac_len, mu_env, mu_mnp, f_g, world,
                   n_robot_contacts, surface_pts, rrt_options, is_refine,
                   refine_dist);

  task->grasp_measure_charac_length = grasp_measure_scale;
}

int main(int argc, char *argv[]) {

  std::string para_path = std::string(SRC_DIR) + "/data/inhand_all/setup.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  // parse configurations
  std::string hand_type = config["hand_type"].as<std::string>();
  std::string object_name = config["object"].as<std::string>();
  double object_scale = config["object_scale"].as<double>();

  std::string save_path = std::string(SRC_DIR) + "/data/inhand_all/results/" +
                          hand_type + "_" + object_name + ".csv";

  double grasp_measure_scale = config["grasp_measure_scale"].as<double>();

  std::string visualization_option =
      config["visualization_option"].as<std::string>();

  int number_of_experiments = config["number_of_experiments"].as<int>();

  double start_x = config["start_position"]["x"].as<double>();
  double start_y = config["start_position"]["y"].as<double>();
  double start_z = config["start_position"]["z"].as<double>();

  int random_seed = config["random_seed"].as<int>();

  double finger_radius = config["finger_radius"].as<double>();

  HMP::Level1Tree<InhandTASK::State, InhandTASK::State2,
                  InhandTASK>::HierarchicalComputeOptions compute_options;

  compute_options.l1_1st.max_iterations =
      config["l1_1st_max_iterations"].as<int>();
  compute_options.l1.max_iterations = config["l1_max_iterations"].as<int>();
  compute_options.l2_1st.max_iterations =
      config["l2_1st_max_iterations"].as<int>();
  compute_options.l2.max_iterations = config["l2_max_iterations"].as<int>();
  compute_options.final_l2_1st.max_iterations =
      config["final_l2_1st_max_iterations"].as<int>();
  compute_options.final_l2.max_iterations =
      config["final_l2_max_iterations"].as<int>();

  compute_options.l1.max_time = config["max_time"].as<double>();

  if (number_of_experiments > 1) {
    std::vector<VectorXd> results;
    for (int i = 0; i < number_of_experiments; ++i) {

      std::srand(std::time(nullptr) + i * 100);

      std::shared_ptr<InhandTASK> task = std::make_shared<InhandTASK>();

      setup(hand_type, finger_radius, object_name, task, start_x, start_y,
            start_z, object_scale, grasp_measure_scale);

      InhandTASK::State start_state = task->get_start_state();

      HMP::Level1Tree<InhandTASK::State, InhandTASK::State2, InhandTASK> tree(
          task, start_state, compute_options);

      tree.ita = 0.2;

      HMP::Node<InhandTASK::State> *current_node = tree.search_tree();

      std::vector<InhandTASK::State> object_trajectory;
      std::vector<InhandTASK::State2> action_trajectory;
      tree.get_final_results(current_node, &object_trajectory,
                             &action_trajectory);
      VectorXd result =
          get_inhand_result(&tree, task, object_trajectory, action_trajectory,
                            current_node->m_value);
      results.push_back(result);

      if (config["save_data"].as<bool>()) {
        appendData(save_path, result.transpose());
      }

      std::cout << "Results: " << std::endl;
      for (auto result : results) {
        std::cout << result.transpose() << std::endl;
      }
    }
    return 0;
  }

  if (random_seed >= 0) {
    std::srand(random_seed);
  } else {
    std::srand(std::time(nullptr));
  }
  
  std::shared_ptr<InhandTASK> task = std::make_shared<InhandTASK>();

  setup(hand_type, finger_radius, object_name, task, start_x, start_y, start_z,
        object_scale, grasp_measure_scale);

  std::string output_file_path = std::string(SRC_DIR) + "/data/inhand_all/" +
                                 config["output_file_name"].as<std::string>();

  if (visualization_option == "csv") {
    visualize_output_file(task->m_world, output_file_path);
    task->m_world->startWindow(&argc, argv);
    return 0;
  }

  if (visualization_option == "setup") {
    VisualizeSG(task->m_world, task->start_object_pose, task->goal_object_pose);
    task->m_world->startWindow(&argc, argv);
    return 0;
  }

  InhandTASK::State start_state = task->get_start_state();

  HMP::Level1Tree<InhandTASK::State, InhandTASK::State2, InhandTASK> tree(
      task, start_state, compute_options);

  tree.ita = 0.2;

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

  get_inhand_result(&tree, task, object_trajectory, action_trajectory,
                    current_node->m_value);

  MatrixXd output_mat = get_output(object_trajectory, action_trajectory, task);
  saveData(output_file_path, output_mat);

  if (visualization_option == "result") {
    visualize_output_file(task->m_world, output_file_path);
    task->m_world->startWindow(&argc, argv);
  }
  return 0;
}
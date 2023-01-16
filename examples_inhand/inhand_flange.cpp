
#include <yaml-cpp/yaml.h>

#include "../search/level1.h"

#include "../tasks/inhand_task.h"

#include "../mechanics/utilities/sample.h"
#include "../mechanics/utilities/utilities.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

#include "visualization.h"

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

void setup(std::shared_ptr<InhandTASK> task, const YAML::Node &config) {

  double box_lx = config["box_lx"].as<double>();
  double box_ly = config["box_ly"].as<double>();
  double box_lz = config["box_lz"].as<double>();

  // read surface point
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) +
                  "/data/box_halflength_50.csv");
  aria::csv::CsvParser parser(f);

  for (auto &row : parser) {
    int n_cols = row.size();
    assert(n_cols == 6);

    Vector6d v;
    for (int j = 0; j < 6; ++j) {
      v(j) = std::stod(row[j]);
    }

    if (randd() > 0.3){
        continue;
    }

    ContactPoint p(
        Vector3d(v[0] * box_lx / 100, v[1] * box_ly / 100, v[2] * box_lz / 100),
        v.tail(3));
    surface_pts.push_back(p);
  }

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", Vector3d(box_lx, box_ly, box_lz));

  world->addObject(object);

  int n_robot_contacts = 2;

  Vector7d x_start;
  Vector7d x_goal;

  x_start << config["start"]["x"].as<double>(),
      config["start"]["y"].as<double>(), config["start"]["z"].as<double>(),
      config["start"]["qx"].as<double>(), config["start"]["qy"].as<double>(),
      config["start"]["qz"].as<double>(), config["start"]["qw"].as<double>();
  x_goal << config["goal"]["x"].as<double>(), config["goal"]["y"].as<double>(),
      config["goal"]["z"].as<double>(), config["goal"]["qx"].as<double>(),
      config["goal"]["qy"].as<double>(), config["goal"]["qz"].as<double>(),
      config["goal"]["qw"].as<double>();

  {
    SkeletonPtr env1 =
        createFixedBox("palm", Vector3d(4, 1, 0.2), Vector3d(0, 0, 0.1 + 3.5),
                       Vector3d(161.0 / 256.0, 159.0 / 256.0, 204 / 256.0));
    double finger_length = config["finger_length"].as<double>();
    SkeletonPtr env2 =
        createFixedBox("fixed_finger", Vector3d(0.2, 1, finger_length),
                       Vector3d(0, 0, 3.5 - finger_length / 2),
                       Vector3d(161.0 / 256.0, 159.0 / 256.0, 204 / 256.0));

    world->addEnvironmentComponent(env1);
    world->addEnvironmentComponent(env2);

    DartPointManipulator *rpt = new DartPointManipulator(n_robot_contacts, 0.2);

    double a = 1;
    double h = 3;
    std::vector<Vector6d> workspace_limits;
    Vector3d p1(1.2, 0, 0);
    Vector3d p2(-1.2, 0, 0);

    // a = 1;

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
    task->if_transition_pruning = false;
  }
  // set the task parameters, start, goal, object inertial, etc....

  long int start_finger_idx = -1;
  long int goal_finger_idx = -1;

  double goal_thr = 3.14 * 5 / 180;

  double wa = 1;
  double wt = 0.1;

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

  rrt_options.x_ub = Eigen::Map<Eigen::Vector3d>(
      config["rrt_options"]["x_ub"].as<std::vector<double>>().data());
  rrt_options.x_lb = Eigen::Map<Eigen::Vector3d>(
      config["rrt_options"]["x_lb"].as<std::vector<double>>().data());

  rrt_options.eps_trans = config["rrt_options"]["eps_trans"].as<double>();
  rrt_options.eps_angle =
      3.14 * config["rrt_options"]["eps_angle_deg"].as<double>() / 180;
  rrt_options.max_samples = config["rrt_options"]["max_samples"].as<int>();

  rrt_options.goal_biased_prob =
      config["rrt_options"]["goal_biased_prob"].as<double>();

  bool is_refine = config["is_refine"].as<bool>();
  double refine_dist = config["refine_dist"].as<double>();

  // pass the world and task parameters to the task through task->initialize

  task->initialize(x_start, x_goal, start_finger_idx, goal_finger_idx, goal_thr,
                   wa, wt, charac_len, mu_env, mu_mnp, f_g, world,
                   n_robot_contacts, surface_pts, rrt_options, is_refine,
                   refine_dist);

  task->grasp_measure_charac_length = -1.0;
}

int main(int argc, char *argv[]) {

  std::string para_path =
      std::string(SRC_DIR) + "/data/inhand_flange/setup.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  // parse configurations

  bool visualize_setup = config["visualize_setup"].as<bool>();
  bool visualize_result = config["visualize_result"].as<bool>();

  int number_of_experiments = config["number_of_experiments"].as<int>();

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

      setup(task, config);

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

      std::cout << "Results: " << std::endl;
      for (auto result : results) {
        std::cout << result.transpose() << std::endl;
      }
    }
    return 0;
  }

  std::srand(std::time(nullptr));

  std::shared_ptr<InhandTASK> task = std::make_shared<InhandTASK>();

  setup(task, config);

  if (visualize_setup) {
    VisualizeSG(task->m_world, task->start_object_pose, task->goal_object_pose);
    task->m_world->startWindow(&argc, argv);
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

  if (visualize_result) {
    VializeStateTraj(task->m_world, task, object_trajectory, action_trajectory);
    task->m_world->startWindow(&argc, argv);
  }
}
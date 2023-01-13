
#include "../search/level1.h"
#include <yaml-cpp/yaml.h>

#include "../tasks/cmg_task.h"

#include "../mechanics/utilities/utilities.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

#include "../tasks/visualization.h"

const CMGTASK::State2::Action CMGTASK::State2::no_action =
    CMGTASK::State2::Action(-1, -1);
const CMGTASK::State::Action CMGTASK::State::no_action = -1;

void setup(std::shared_ptr<CMGTASK> task, bool use_object_surface_sampling)
{
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
  x_goal << 2.5, 0, box_length / 2 * 1.5, 0, -0.7071, 0, 0.7071;
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

  CMGTASK::SearchOptions rrt_options;

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

  if (!use_object_surface_sampling)
  {

    std::ifstream f(std::string(SRC_DIR) +
                    "/data/env_pick_card/surface_contacts.csv");
    aria::csv::CsvParser parser(f);

    for (auto &row : parser)
    {
      int n_cols = row.size();
      assert(n_cols == 6);

      Vector6d v;
      for (int j = 0; j < 6; ++j)
      {
        v(j) = std::stod(row[j]);
      }
      ContactPoint p(v.head(3), v.tail(3));
      surface_pts.push_back(p);
    }
  }
  else
  {
    std::ifstream f(std::string(SRC_DIR) +
                    "/data/box_halflength_1.csv");
    aria::csv::CsvParser parser(f);

    for (auto &row : parser)
    {
      int n_cols = row.size();
      assert(n_cols == 6);

      Vector6d v;
      for (int j = 0; j < 6; ++j)
      {
        v(j) = std::stod(row[j]);
      }
      if (abs(v[5]) < 0.8)
      {
        continue;
      }
      ContactPoint p(Vector3d(v[0] * box_length / 2, v[1] * box_length / 2, v[2] * box_height / 2), -v.tail(3));
      surface_pts.push_back(p);
    }
  }

  // pass the world and task parameters to the task through task->initialize
  task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                   mu_mnp, oi, f_g, world, n_robot_contacts, CMG_QUASISTATIC,
                   surface_pts, rrt_options, if_refine, refine_dist);
}

int main(int argc, char *argv[])
{
  std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

  std::string para_path =
      std::string(SRC_DIR) + "/data/env_pick_card/setup.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  std::string visualization_option =
      config["visualization_option"].as<std::string>();
  double grasp_measure_scale = config["grasp_measure_scale"].as<double>();
  int random_seed = config["random_seed"].as<int>();

  bool use_object_surface_sampling = config["use_object_surface_sampling"].as<bool>();

  bool run_batch = config["run_batch"].as<bool>();

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2,
                  CMGTASK>::HierarchicalComputeOptions compute_options;

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

  if (run_batch)
  {
    std::vector<VectorXd> results;
    for (int iter = 0; iter < 10; iter++)
    {
      std::srand(iter * 10000 + 254122);
      setup(task, use_object_surface_sampling);
      task->grasp_measure_charac_length = grasp_measure_scale;
      CMGTASK::State start_state = task->get_start_state();

      HMP::Level1Tree<CMGTASK::State, CMGTASK::State2, CMGTASK> tree(
          task, start_state, compute_options);

      HMP::Node<CMGTASK::State> *current_node = tree.search_tree();

      std::vector<CMGTASK::State> object_trajectory;
      std::vector<CMGTASK::State2> action_trajectory;
      tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

      std::vector<Vector7d> object_traj;
      std::vector<VectorXd> mnp_traj;
      VectorXd result = get_results(&tree, task, object_trajectory, action_trajectory,
                                    current_node->m_value);

      results.push_back(result);
      std::cout << "batch results:" << std::endl;

      for (int i = 0; i < results.size(); i++)
      {
        std::cout << results[i].transpose() << std::endl;
      }
    }
    return 0;
  }
  std::srand(random_seed);
  setup(task, use_object_surface_sampling);

  task->grasp_measure_charac_length = grasp_measure_scale;

  if (visualization_option == "setup")
  {
    VisualizeSG(task->m_world, task->start_object_pose, task->goal_object_pose);
    task->m_world->startWindow(&argc, argv);
  }

  CMGTASK::State start_state = task->get_start_state();

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2, CMGTASK> tree(
      task, start_state, compute_options);

  HMP::Node<CMGTASK::State> *current_node = tree.search_tree();

  std::vector<CMGTASK::State> object_trajectory;
  std::vector<CMGTASK::State2> action_trajectory;
  tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

  std::vector<Vector7d> object_traj;
  std::vector<VectorXd> mnp_traj;

  get_results(&tree, task, object_trajectory, action_trajectory,
              current_node->m_value);

  if (visualization_option == "result")
  {
    VisualizeStateTraj(task->m_world, task, object_trajectory,
                       action_trajectory);
    task->m_world->startWindow(&argc, argv);
  }
}

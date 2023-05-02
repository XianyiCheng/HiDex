#include "../src/tasks/setup.h"

#include "../src/tasks/visualization.h"
#include <ctime>

#include <sys/stat.h>
#include <sys/types.h>

#ifndef SAMPLE_H
#define SAMPLE_H
#include "../src/mechanics/utilities/sample.h"
#endif

// const TASK::State2::Action TASK::State2::no_action =
//     TASK::State2::Action(-1, -1);
// const TASK::State::Action TASK::State::no_action = -1;

std::string get_current_time()
{
  std::time_t t = std::time(nullptr);
  char buffer[15];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", std::localtime(&t));
  std::string current_time_string = buffer;
  return current_time_string;
}

void sample_and_load_start_and_goal(std::shared_ptr<TASK> task,
                                    const YAML::Node &config, Vector3d ub,
                                    Vector3d lb)
{
  Vector7d x_start;
  Vector7d x_goal;
  Vector3d p_start = sample_position(ub, lb);
  Vector3d p_goal = sample_position(ub, lb);
  x_start << p_start, 0, 0, 0, 1;
  x_goal << p_goal, 0, 0, 0, 1;
  if (config["sample_start_and_goal"]["random_orientation"])
  {
    if (config["sample_start_and_goal"]["random_orientation"].as<bool>())
    {
      Quaterniond q_start = generate_unit_quaternion();
      Quaterniond q_goal = generate_unit_quaternion();
      x_start << p_start, q_start.x(), q_start.y(), q_start.z(),
          q_start.w();
      x_goal << p_goal, q_goal.x(), q_goal.y(), q_goal.z(),
          q_goal.w();
    }
  }

  task->set_start_and_goal(x_start, x_goal);
}

int main(int argc, char *argv[])
{

  std::string batch_config_file;

  if (argc > 1)
  {
    batch_config_file = argv[1];
  }
  else
  {
    batch_config_file =
        "/home/xianyi/Research/MCTS/general_planner/batch_config_template.yaml";
  }

  // should use arg to specify the path to the setup.yaml file

  YAML::Node batch_config = YAML::LoadFile(batch_config_file);

  std::string task_folder = batch_config["folder"].as<std::string>();
  YAML::Node config = YAML::LoadFile(task_folder + "/setup.yaml");

  mkdir((task_folder + "/runs").c_str(), 0755);

  int number_of_experiments = batch_config["number_of_experiments"].as<int>();

  std::string visualize_option = config["visualize_option"].as<std::string>();

  for (int n_exp = 0; n_exp < number_of_experiments; n_exp++)
  {

    std::string run_name = get_current_time();
    std::cout << run_name << std::endl;

    // set random seed
    std::srand(std::stoll(run_name));

    std::shared_ptr<TASK> task = std::make_shared<TASK>();

    std::string run_folder = task_folder + "/runs/" + run_name;

    load_task(task, config);

    // TODO: replace this with radomization
    if (batch_config["sample_start_and_goal"]["random"].as<bool>())
    {

      sample_and_load_start_and_goal(
          task, batch_config,
          Eigen::Map<Vector3d>(
              batch_config["sample_start_and_goal"]["position_upper_bound"]
                  .as<std::vector<double>>()
                  .data()),
          Eigen::Map<Vector3d>(
              batch_config["sample_start_and_goal"]["position_lower_bound"]
                  .as<std::vector<double>>()
                  .data()));
    }
    else
    {
      load_start_and_goal_poses(task, config);
    }

    load_reward_functions(task, config);

    if (visualize_option == "setup")
    {
      VisualizeSG(task->m_world, task->start_object_pose,
                  task->goal_object_pose);
      task->m_world->startWindow(&argc, argv);
      return 0;
    }

    task->initialize();

    HMP::HierarchicalComputeOptions compute_options;
    load_mcts_options(compute_options, config);

    HMP::Level1MCTS<TASK::State, TASK::State2, TASK> tree(
        task, task->get_start_state(), compute_options);

    HMP::Node<TASK::State> *current_node = tree.search_tree();

    std::vector<TASK::State> object_trajectory;
    std::vector<TASK::State2> action_trajectory;

    tree.get_final_results(current_node, &object_trajectory,
                           &action_trajectory);

    if (visualize_option == "show")
    {
      VisualizeStateTraj(task->m_world, task, object_trajectory,
                         action_trajectory);
      task->m_world->startWindow(&argc, argv);
    }

    mkdir(run_folder.c_str(), 0755);

    if (batch_config["collect_trajectory"].as<bool>())
    {
      MatrixXd output_mat =
          get_output_object_centric(object_trajectory, action_trajectory, task);
      saveData(run_folder + "/trajectory.csv", output_mat);
    }

    if (batch_config["collect_results"].as<bool>())
    {
      VectorXd result = get_results(&tree, task, object_trajectory,
                                    action_trajectory, current_node->m_value);
      saveData(run_folder + "/results.csv", result);
    }
  }
}
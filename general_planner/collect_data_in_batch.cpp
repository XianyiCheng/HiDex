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

std::string get_current_time() {
  std::time_t t = std::time(nullptr);
  char buffer[15];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", std::localtime(&t));
  std::string current_time_string = buffer;
  return current_time_string;
}

Vector7d sample_a_pose(const Vector7d &original_pose, const YAML::Node &config,
                       const std::string &name) {
  Vector7d x = original_pose;
  if (config[name.c_str()]) {
    if (config[name.c_str()]["allowed"].as<bool>()) {
      Vector3d ub =
          Eigen::Map<Vector3d>(config[name.c_str()]["position_upper_bound"]
                                   .as<std::vector<double>>()
                                   .data());
      Vector3d lb =
          Eigen::Map<Vector3d>(config[name.c_str()]["position_lower_bound"]
                                   .as<std::vector<double>>()
                                   .data());
      Vector3d p = sample_position(ub, lb);
      x.head(3) = p;
      if (config[name.c_str()]["random_orientation"].as<bool>()) {
        Quaterniond q = generate_unit_quaternion();
        x[3] = q.x();
        x[4] = q.y();
        x[5] = q.z();
        x[6] = q.w();
      }
    }
  }
  return x;
}

int main(int argc, char *argv[]) {

  

  std::string task_folder;

  if (argc > 1) {
    task_folder = path_join(std::string(SRC_DIR),argv[1]);
    std::cout << "Task_folder: " << task_folder << std::endl;
  } else {
    task_folder =
        path_join(std::string(SRC_DIR), "/data/pushing_all_dir");
  }

  std::string batch_config_file = path_join(task_folder, "batch.yaml");

  YAML::Node batch_config = YAML::LoadFile(batch_config_file);

  YAML::Node config = YAML::LoadFile(path_join(task_folder, "/setup.yaml"));

  std::string run_folder = path_join(task_folder, "runs");
  mkdir(run_folder.c_str(), 0755);

  int number_of_experiments = batch_config["number_of_experiments"].as<int>();

  std::string visualize_option =
      batch_config["visualize_option"].as<std::string>();

  for (int n_exp = 0; n_exp < number_of_experiments; n_exp++) {

    std::string run_name = get_current_time();
    std::cout << "Run name " << run_name << std::endl;

    // set random seed
    std::srand(std::stoll(run_name));

    std::shared_ptr<TASK> task = std::make_shared<TASK>();

    std::string instance_folder = path_join(run_folder, run_name);

    load_task(task, config, task_folder);

    std::cout << "Loading start and goal poses" << std::endl;
    load_start_and_goal_poses(task, config);

    std::cout << "Sampling start and goal poses" << std::endl;
    task->start_object_pose =
        sample_a_pose(task->start_object_pose, batch_config, "random_start");
    task->goal_object_pose =
        sample_a_pose(task->goal_object_pose, batch_config, "random_goal");

    load_reward_functions(task, config);

    if (visualize_option == "setup") {
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

    mkdir(instance_folder.c_str(), 0755);

    // collect full trajectory
    std::string traj_file = path_join(instance_folder, "/trajectory.csv");
    save_full_output_object_centric(object_trajectory, action_trajectory, task,
                                    traj_file);

    if (batch_config["collect_results"].as<bool>()) {
      VectorXd result = get_results(&tree, task, object_trajectory,
                                    action_trajectory, current_node->m_value);
      saveData(path_join(instance_folder,"/results.csv"), result);
    }

    if (visualize_option == "show") {
      visualize_full_output_file_object_centric(task->m_world, traj_file);
      task->m_world->startWindow(&argc, argv);
    }
  }
}
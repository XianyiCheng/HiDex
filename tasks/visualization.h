void VisualizeSG(std::shared_ptr<WorldTemplate> world, Vector7d start_pose,
                 Vector7d goal_pose) {
  std::vector<Vector7d> traj;
  for (int i = 0; i < 100; i++) {
    traj.push_back(start_pose);
  }
  for (int i = 0; i < 100; i++) {
    traj.push_back(goal_pose);
  }
  world->setObjectTrajectory(traj);
}
void VisualizeTraj(std::shared_ptr<WorldTemplate> world,
                   const std::vector<Vector7d> &object_traj,
                   const std::vector<VectorXd> &mnp_traj) {
  world->setPlaybackTrajectory(object_traj, mnp_traj);
}

template <class State, class State2, class Task>
void VisualizeStateTraj(std::shared_ptr<WorldTemplate> world,
                      std::shared_ptr<Task> task,
                      const std::vector<State> &object_traj,
                      const std::vector<State2> &mnp_traj) {
  std::vector<Vector7d> object_traj_vec;
  std::vector<VectorXd> mnp_traj_vec;

  for (int kk = 0; kk < 5; kk++) {
    object_traj_vec.push_back(object_traj[0].m_pose);
    mnp_traj_vec.push_back(
        task->get_robot_config_from_action_idx(mnp_traj[1].finger_index));
  }

  for (int i = 0; i < object_traj.size(); i++) {
    object_traj_vec.push_back(object_traj[i].m_pose);
    // std::cout << object_traj[i].m_pose.transpose() << std::endl;
  }
  for (int k = 1; k < mnp_traj.size(); k++) {
    int start_j = mnp_traj[k].timestep;
    int end_j = ((k + 1) == mnp_traj.size()) ? object_traj.size()
                                             : mnp_traj[k + 1].timestep;
    for (int j = start_j; j < end_j; j++) {
      mnp_traj_vec.push_back(
          task->get_robot_config_from_action_idx(mnp_traj[k].finger_index));
    }
  }
  world->setPlaybackTrajectory(object_traj_vec, mnp_traj_vec);
}

template <class Tree, class State, class State2, class Task>
VectorXd get_results(Tree *tree, std::shared_ptr<Task> task,
                           const std::vector<State> &object_trajectory,
                           const std::vector<State2> &action_trajectory,
                           double final_value) {

  std::cout << "Solution found time"
            << ", "
            << "Total time"
            << ", "
            << "Success? "
            << ", "
            << "Nodes in MCTS "
            << ", "
            << "Nodes in RRT"
            << ", "
            << "Solution length"
            << ", "
            << "Total travel distance"
            << ", "
            << "Finger relocation"
            << ", "
            << "Finger Change Ratio"
            << ", "
            << "Env contact changes"
            << ", "
            << "Grasp measure"
            << ", "
            << "Final reward" << std::endl;

  double solution_found_time = tree->solution_found_time;
  double total_time = tree->total_time;
  bool success = final_value > 0.0;
  int nodes_in_mcts = tree->count_total_nodes();
  int nodes_in_rrt = task->total_rrt_nodes();
  int solution_length = object_trajectory.size();
  double total_travel_distance =
      task->travel_distance(object_trajectory) /
      task->shared_rrt->dist(task->start_object_pose, task->goal_object_pose);
  int finger_relocation = action_trajectory.size() - 1;
  double finger_change_ratio =
      task->total_finger_change_ratio(action_trajectory);
  int num_env_change = task->number_environment_contact_changes(object_trajectory);

  double grasp_measure;
  if (task->grasp_measure_charac_length > 0) {
    double avg_grasp_d = 0.0;
    for (auto s2 : action_trajectory) {
      if (s2.finger_index == -1 || s2.timestep == -1) {
        continue;
      }
      double grasp_d = task->grasp_measure(s2.finger_index, s2.timestep);
      avg_grasp_d += grasp_d;
    }
    grasp_measure = avg_grasp_d / double(action_trajectory.size() - 1);
  } else {
    grasp_measure = -1;
  }

  std::cout << solution_found_time << ", " << total_time << ", " << success
            << ", " << nodes_in_mcts << ", " << nodes_in_rrt << ", "
            << solution_length << ", " << total_travel_distance << ", "
            << finger_relocation << ", " << finger_change_ratio << ", " << num_env_change << ", "
            << grasp_measure << ", " << final_value << std::endl;

  VectorXd result(12);
  result << solution_found_time, total_time, success, nodes_in_mcts,
      nodes_in_rrt, solution_length, total_travel_distance, finger_relocation,
      finger_change_ratio, num_env_change, grasp_measure, final_value;
  return result;
}
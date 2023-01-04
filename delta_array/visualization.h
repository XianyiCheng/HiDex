void VisualizeSG(std::shared_ptr<WorldTemplate> world, Vector7d start_pose,
                 Vector7d goal_pose) {
  std::vector<Vector7d> traj;
  traj.push_back(start_pose);
  traj.push_back(goal_pose);
  world->setObjectTrajectory(traj);
}

void VisualizeTraj(std::shared_ptr<WorldTemplate> world,
                   const std::vector<Vector7d> &object_traj,
                   const std::vector<VectorXd> &mnp_traj) {
  world->setPlaybackTrajectory(object_traj, mnp_traj);
}

template <class State, class State2, class Task>
void VializeStateTraj(std::shared_ptr<WorldTemplate> world,
                      std::shared_ptr<Task> task,
                      const std::vector<State> &object_traj,
                      const std::vector<State2> &mnp_traj) {
  std::vector<Vector7d> object_traj_vec;
  std::vector<VectorXd> mnp_traj_vec;

  for (int i = 0; i < object_traj.size(); i++) {
    object_traj_vec.push_back(object_traj[i].m_pose);
    std::cout << object_traj[i].m_pose.transpose() << std::endl;
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
void output_results(Tree *tree, std::shared_ptr<Task> task,
                    const std::vector<State> &object_trajectory,
                    const std::vector<State2> &action_trajectory,
                    double final_value) {

  std::cout << "Solution found time"
            << "\t"
            << "Total time"
            << "\t"
            << "Success? "
            << "\t"
            << "Nodes in MCTS "
            << "\t"
            << "Nodes in RRT"
            << "\t"
            << "Solution length"
            << "\t"
            << "Total travel distance"
            << "\t"
            << "Finger relocation"
            << "\t"
            << "Finger Change Ratio"
            << "\t"
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

  std::cout << solution_found_time << "\t" << total_time << "\t" << success
            << "\t" << nodes_in_mcts << "\t" << nodes_in_rrt << "\t"
            << solution_length << "\t" << total_travel_distance << "\t"
            << finger_relocation << "\t" << finger_change_ratio << "\t"
            << final_value << std::endl;
}

template <class Tree, class State, class State2, class Task>
VectorXd get_inhand_result(Tree *tree, std::shared_ptr<Task> task,
                           const std::vector<State> &object_trajectory,
                           const std::vector<State2> &action_trajectory,
                           double final_value) {

  std::cout << "Solution found time"
            << "\t"
            << "Total time"
            << "\t"
            << "Success? "
            << "\t"
            << "Nodes in MCTS "
            << "\t"
            << "Nodes in RRT"
            << "\t"
            << "Solution length"
            << "\t"
            << "Total travel distance"
            << "\t"
            << "Finger relocation"
            << "\t"
            << "Finger Change Ratio"
            << "\t"
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

  std::cout << solution_found_time << "\t" << total_time << "\t" << success
            << "\t" << nodes_in_mcts << "\t" << nodes_in_rrt << "\t"
            << solution_length << "\t" << total_travel_distance << "\t"
            << finger_relocation << "\t" << finger_change_ratio << "\t"
            << final_value << std::endl;

  VectorXd result(10);
  result << solution_found_time, total_time, success, nodes_in_mcts,
      nodes_in_rrt, solution_length, total_travel_distance, finger_relocation,
      finger_change_ratio, final_value;
  return result;
}

template <class State, class State2, class Task>
MatrixXd get_output(const std::vector<State> &object_trajectory,
                    const std::vector<State2> &action_trajectory,
                    std::shared_ptr<Task> task) {
  std::vector<VectorXd> output;
  int t_span = 5;

  for (int i = 1; i < action_trajectory.size(); ++i) {
    int t = action_trajectory[i].timestep;
    int t_next;
    if (i == action_trajectory.size() - 1) {
      t_next = object_trajectory.size() - 1;
    } else {
      t_next = action_trajectory[i + 1].timestep;
    }

    VectorXd mnp_config = task->get_robot_config_from_action_idx(
        action_trajectory[i].finger_index);

    // get the array of object pose
    std::vector<Vector7d> object_poses;
    if (t_next <= t) {
      for (int kk = 0; kk < t_span; ++kk) {
        object_poses.push_back(object_trajectory[t].m_pose);
      }
    } else {
      std::vector<Vector7d> object_poses_all;
      for (int kk = t; kk < t_next; ++kk) {
        object_poses_all.insert(object_poses_all.end(),
                                object_trajectory[kk].m_path.begin(),
                                object_trajectory[kk].m_path.end());
      }
      int n_span = (object_poses_all.size() - 2) / 3;
      object_poses.push_back(object_poses_all[0]);
      object_poses.push_back(object_poses_all[n_span]);
      object_poses.push_back(object_poses_all[2 * n_span]);
      object_poses.push_back(object_poses_all[3 * n_span]);
      object_poses.push_back(object_poses_all[object_poses_all.size() - 1]);
    }

    // for each object pose, get the array of mnp config

    for (auto x : object_poses) {
      VectorXd mnp_config_world(6 * task->number_of_robot_contacts);
      for (int n_pt = 0; n_pt < task->number_of_robot_contacts; ++n_pt) {
        if (std::isnan(mnp_config[6 * n_pt])) {
          mnp_config_world.segment(6 * n_pt, 6) = mnp_config.segment(6 * n_pt, 6);
          continue;
        }
        Vector3d p = mnp_config.segment(6 * n_pt, 3);
        Vector3d n = mnp_config.segment(6 * n_pt + 3, 3);
        Eigen::Matrix3d R;
        R = quat2SO3(x(6), x(3), x(4), x(5));
        Vector3d p_world = R * p + x.segment(0, 3);
        Vector3d n_world = R * n;
        mnp_config_world.segment(6 * n_pt, 3) = p_world;
        mnp_config_world.segment(6 * n_pt + 3, 3) = n_world;
      }
      VectorXd output_row(6 * task->number_of_robot_contacts + 7);
      output_row.segment(0, 6 * task->number_of_robot_contacts) =
          mnp_config_world;
      output_row.segment(6 * task->number_of_robot_contacts, 7) = x;
      output.push_back(output_row);
    }
  }

  MatrixXd output_mat(output.size(), 6 * task->number_of_robot_contacts + 7);
  for (int i = 0; i < output.size(); ++i) {
    output_mat.row(i) = output[i];
  }
  return output_mat;
}

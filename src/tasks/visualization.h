#include "../mechanics/utilities/io.h"

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
      double grasp_d = task->grasp_measure_charac_length*task->grasp_measure(s2.finger_index, s2.timestep);
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

template <class State, class State2, class Task>
MatrixXd get_output(const std::vector<State> &object_trajectory,
                    const std::vector<State2> &action_trajectory,
                    std::shared_ptr<Task> task) {

  // each row: robot contacts (p,n) in the world frame, object pose
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
      // object_poses_all.push_back(object_trajectory[t].m_pose);
      for (int kk = t+1; kk <= t_next; ++kk) {
        object_poses_all.insert(object_poses_all.end(),
                                object_trajectory[kk].m_path.begin(),
                                object_trajectory[kk].m_path.end());
      }
      int n_span = std::max(int(object_poses_all.size())- 2, 0) / 3;
      object_poses = object_poses_all;
    }

    // for each object pose, get the array of mnp config

    for (auto x : object_poses) {
      VectorXd mnp_config_world(6 * task->number_of_robot_contacts);
      for (int n_pt = 0; n_pt < task->number_of_robot_contacts; ++n_pt) {
        if (std::isnan(mnp_config[6 * n_pt])) {
          mnp_config_world.segment(6 * n_pt, 6) = mnp_config.segment(6 * n_pt, 6);
          mnp_config_world(6 * n_pt) = -7777;
          mnp_config_world(6 * n_pt + 1) = -7777;
          mnp_config_world(6 * n_pt + 2) = -7777;
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

template <class State, class State2, class Task>
MatrixXd get_output_object_centric(const std::vector<State> &object_trajectory,
                    const std::vector<State2> &action_trajectory,
                    std::shared_ptr<Task> task) {

  // each row: robot contacts (p,n) in the object frame, object pose
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
      // object_poses_all.push_back(object_trajectory[t].m_pose);
      for (int kk = t+1; kk <= t_next; ++kk) {
        object_poses_all.insert(object_poses_all.end(),
                                object_trajectory[kk].m_path.begin(),
                                object_trajectory[kk].m_path.end());
      }
      int n_span = std::max(int(object_poses_all.size())- 2, 0) / 3;
      object_poses = object_poses_all;
    }

    // for each object pose, get the array of mnp config

    for (auto x : object_poses) {
      VectorXd output_row(6 * task->number_of_robot_contacts + 7);
      output_row.segment(0, 6 * task->number_of_robot_contacts) =
          mnp_config;
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


void visualize_output_file_object_centric(std::shared_ptr<WorldTemplate> world, std::string file_name){
  MatrixXd data = openData(file_name);
  int n_data = data.rows();
  int n_pts = (data.cols() - 7) / 6;
  std::vector<Vector7d> object_traj;
  std::vector<VectorXd> mnp_traj;
  for (int i = 0; i < n_data; ++i){
    VectorXd mnp_config = data.row(i).segment(0, 6 * n_pts);
    Vector7d object_pose = data.row(i).segment(6 * n_pts, 7);
    object_traj.push_back(object_pose);
    mnp_traj.push_back(mnp_config);
  }
  world->setPlaybackTrajectory(object_traj, mnp_traj);
}

template <class State, class State2, class Task>
void save_full_output_object_centric(const std::vector<State> &object_trajectory,
                    const std::vector<State2> &action_trajectory,
                    std::shared_ptr<Task> task, const std::string &file_name) {

  // first row: number of robot contacts
  MatrixXd output_mat(1, 1);
  output_mat(0,0) = task->number_of_robot_contacts;
  appendData(file_name, output_mat);

  // each row: object pose (7), robot contacts (p,n) in the object frame, env contacts (p,n) in the object frame, env_cs_mode

  int t_span = 5;

  for (int i = 1; i < action_trajectory.size(); ++i) {

    std::vector<VectorXd> output;
    
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
    std::vector<std::vector<ContactPoint>> envs_all;
    std::vector<VectorXi> modes_all;

    if (t_next <= t) {
      for (int kk = 0; kk < t_span; ++kk) {
        object_poses.push_back(object_trajectory[t].m_pose);
        envs_all.push_back(object_trajectory[t].envs);
        modes_all.push_back(object_trajectory[t].modes[object_trajectory[t].m_mode_idx]);
      }
    } else {
      std::vector<Vector7d> object_poses_all;
      // object_poses_all.push_back(object_trajectory[t].m_pose);
      for (int kk = t+1; kk <= t_next; ++kk) {
        object_poses_all.insert(object_poses_all.end(),
                                object_trajectory[kk].m_path.begin(),
                                object_trajectory[kk].m_path.end());
        for (int nk = 0; nk < object_trajectory[kk].m_path.size(); nk++){
          envs_all.push_back(object_trajectory[kk].envs);
          if (object_trajectory[kk].m_mode_idx == -1){
            VectorXi mode = VectorXi::Zero(object_trajectory[kk].envs.size());
            modes_all.push_back(mode);
          } else {
            modes_all.push_back(object_trajectory[kk].modes[object_trajectory[kk].m_mode_idx]);
          }
        }
      }
      int n_span = std::max(int(object_poses_all.size())- 2, 0) / 3;
      object_poses = object_poses_all;
    }

    // for each object pose, get the array of mnp config

    for (int nk = 0; nk < object_poses.size(); nk++) {
      int n_cols = 7 + 6 * task->number_of_robot_contacts + 6 * envs_all[nk].size() + envs_all[nk].size();

      VectorXd output_row(n_cols);
      output_row.segment(0, 7) = object_poses[nk];
      output_row.segment(7, 6 * task->number_of_robot_contacts) =
          mnp_config;
      for (int k = 0; k < envs_all[nk].size(); k++){
        output_row.segment(7 + 6 * task->number_of_robot_contacts + 6 * k, 3) = envs_all[nk][k].p;
        output_row.segment(7 + 6 * task->number_of_robot_contacts + 6 * k + 3, 3) = envs_all[nk][k].n;
        output_row(7 + 6 * task->number_of_robot_contacts + 6 * envs_all[nk].size()+k) = double(modes_all[nk][k]);
      }
      // output.push_back(output_row);
      MatrixXd output_data(1, n_cols);
      output_data.row(0) = output_row;
      appendData(file_name, output_data);
    }
    
  }
}


void visualize_full_output_file_object_centric(std::shared_ptr<WorldTemplate> world, std::string file_name){
  std::vector<std::vector<double>> data = openVectorData(file_name);
  int n_data = data.size();
  int n_robot_pts = int(data[1][0]);
  std::vector<Vector7d> object_traj;
  std::vector<VectorXd> mnp_traj;
  for (int i = 2; i < n_data; ++i){
    std::vector<double> data_row(data[i].begin(), data[i].begin()+7+6*n_robot_pts);
    VectorXd v_row = Eigen::Map<VectorXd, Eigen::Unaligned>(data_row.data(), data_row.size());
    VectorXd mnp_config = v_row.segment(7, 6 * n_robot_pts);
    Vector7d object_pose = v_row.segment(0, 7);
    object_traj.push_back(object_pose);
    mnp_traj.push_back(mnp_config);
  }
  world->setPlaybackTrajectory(object_traj, mnp_traj);
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
            << "Grasp measure"
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

  // compute grasp measurement reward
  // double max_grasp_d = 0.0;
  // for (auto s2 : action_trajectory) {
  //   double grasp_d = action_trajectory->grasp_measure(s2.finger_index,
  //   s2.timestep); if (grasp_d > max_grasp_d) {
  //     max_grasp_d = grasp_d;
  //   }
  // }

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

  std::cout << solution_found_time << "\t" << total_time << "\t" << success
            << "\t" << nodes_in_mcts << "\t" << nodes_in_rrt << "\t"
            << solution_length << "\t" << total_travel_distance << "\t"
            << finger_relocation << "\t" << finger_change_ratio << "\t"
            << grasp_measure << "\t" << final_value << std::endl;

  VectorXd result(11);
  result << solution_found_time, total_time, success, nodes_in_mcts,
      nodes_in_rrt, solution_length, total_travel_distance, finger_relocation,
      finger_change_ratio, grasp_measure, final_value;
  return result;
}

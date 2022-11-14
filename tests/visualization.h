void VisualizeSG(std::shared_ptr<WorldTemplate> world, Vector7d start_pose, Vector7d goal_pose){
  std::vector<Vector7d> traj;
  traj.push_back(start_pose);
  traj.push_back(goal_pose);
  world->setObjectTrajectory(traj);
}

void VisualizeTraj(std::shared_ptr<WorldTemplate> world, const std::vector<Vector7d> & object_traj, const std::vector<VectorXd> & mnp_traj ){
  world->setPlaybackTrajectory(object_traj, mnp_traj);
}

void VializeStateTraj(std::shared_ptr<WorldTemplate> world, std::shared_ptr<CMGTASK> task, const std::vector<CMGTASK::State> & object_traj, const std::vector<CMGTASK::State2> & mnp_traj ){
  std::vector<Vector7d> object_traj_vec;
  std::vector<VectorXd> mnp_traj_vec;

  for(int i = 0; i < object_traj.size(); i++){
    object_traj_vec.push_back(object_traj[i].m_pose);
  }
  for (int k = 1; k < mnp_traj.size(); k++){
    int start_j = mnp_traj[k].timestep;
    int end_j = ((k+1) == mnp_traj.size())? object_traj.size():mnp_traj[k+1].timestep;
    for (int j = start_j; j < end_j; j++){
      mnp_traj_vec.push_back(task->get_robot_config_from_action_idx(mnp_traj[k].finger_index));
    }
  }
  world->setPlaybackTrajectory(object_traj_vec, mnp_traj_vec);
}
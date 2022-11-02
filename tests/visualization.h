void VisualizeSG(std::shared_ptr<WorldTemplate> world, Vector7d start_pose, Vector7d goal_pose){
  std::vector<Vector7d> traj;
  traj.push_back(start_pose);
  traj.push_back(goal_pose);
  world->setObjectTrajectory(traj);
}

void VisualizeTraj(std::shared_ptr<WorldTemplate> world, const std::vector<Vector7d> & object_traj, const std::vector<VectorXd> & mnp_traj ){
  world->setPlaybackTrajectory(object_traj, mnp_traj);
}
#include "DartPointManipulator.h"

#include "../utilities/sample.h"

#include "../contacts/contact_kinematics.h"

DartPointManipulator::DartPointManipulator(int n, double radius) {

  this->n_pts = n;
  this->fingertip_radius = radius;

  for (int i = 0; i < n; i++) {
    SkeletonPtr ball = createFreeBall("ball_" + std::to_string(i), radius,
                                      Vector3d(0.3, 0.3, 0.8));
    this->addBody(ball);
  }
}

void DartPointManipulator::getFingertipsOnObject(
    const VectorXd &config, const Vector7d &object_pose,
    std::vector<ContactPoint> *fingertips) {

  int n = int(config.size() / 6);

  if (fingertips->size() != 0) {
    fingertips->clear();
  }

  for (int i = 0; i < n; i++) {
    fingertips->push_back(
        ContactPoint(config.segment<3>(6 * i), config.segment<3>(6 * i + 3)));
  }
}

void DartPointManipulator::setConfig(const VectorXd &config,
                                     const Vector7d &object_pose) {

  Eigen::Matrix4d T;
  T = pose2SE3(object_pose);
  Eigen::Matrix3d R;
  R = T.block(0, 0, 3, 3);
  Eigen::Vector3d p;
  p = T.block(0, 3, 3, 1);

  int n = int(config.size() / 6);
  for (int i = 0; i < this->n_pts; i++) {
    Eigen::Vector6d pos(Eigen::Vector6d::Zero());
    Vector3d pp;
    if (std::isnan(config[6 * i])) {
      pp << 100, 100, 100;
    } else {
      pp << config[6 * i], config[6 * i + 1], config[6 * i + 2];
    }
    if (i < n) {
      pos.tail(3) = R * pp + p;
    }
    this->bodies[i]->setPositions(pos);
  }
}

bool DartPointManipulator::resampleFingers(
    int n_on, const VectorXd &config, const Vector7d &object_pose,
    const std::vector<ContactPoint> &object_surface, VectorXd &new_config,
    std::vector<ContactPoint> *remain_fingertips) {

  // n_on: number of relocating fingers
  if (n_on == 0) {
    return false;
  }

  std::vector<ContactPoint> fingertips;
  Vector7d x_o;
  getFingertipsOnObject(config, x_o, &fingertips);

  std::vector<ContactPoint> new_samples;
  // TODO: this can be again formulated as a search problem
  int n_pts = this->n_pts;
  int n_located = fingertips.size();
  int n_unlocated = n_pts - n_located;
  int N = object_surface.size();

  std::vector<ContactPoint> remain_mnps;
  std::vector<int> relocate_idxes;

  for (int i = 0; i < std::min(n_on, n_unlocated); i++) {
    if (randd() > 0.5) {
      // randomly choose to not locate this unlocated finger
      continue;
    }
    int idx = randi(N);
    new_samples.push_back(object_surface[idx]);
    remain_mnps.push_back(object_surface[idx]);
  }

  for (int i = 0; i < n_on - n_unlocated; i++) {
    // randomly choose fingers to relocate

    int idx = randi(n_located);

    while (std::find(relocate_idxes.begin(), relocate_idxes.end(), idx) !=
           relocate_idxes.end()) {
      idx = randi(n_located);
    }

    // randomly choose to release the finger
    if (randd() > 0.5) {
      relocate_idxes.push_back(idx);
    } else {
      new_samples.push_back(object_surface[randi(N)]);
      relocate_idxes.push_back(idx);
    }
  }
  for (int k = 0; k < n_located; k++) {
    // find remaining fingers
    if (std::find(relocate_idxes.begin(), relocate_idxes.end(), k) ==
        relocate_idxes.end()) {
      remain_mnps.push_back(fingertips[k]);
      new_samples.push_back(fingertips[k]);
    }
  }

  // copy_points(new_samples, new_mnps);
  copy_points(remain_mnps, remain_fingertips);

  // new configuration
  new_config.resize(new_samples.size() * 6);
  for (int i = 0; i < new_samples.size(); i++) {
    new_config.segment(6 * i, 3) = new_samples[i].p;
    new_config.segment(6 * i + 3, 3) = new_samples[i].n;
  }

  new_samples.clear();
  return true;
}

void DartPointManipulator::Fingertips2PointContacts(
    const std::vector<ContactPoint> &fingertips,
    std::vector<ContactPoint> *point_contacts) {

  if (is_patch_contact) {
    Matrix3d xs;
    xs << 1, 0, 0, -0.5, 0.865, 0, -0.5, -0.865, 0;

    double r = this->fingertip_radius;
    for (auto &pt : fingertips) {
      Matrix6d Adgco = contact_jacobian(pt.p, pt.n);
      Matrix3d Roc = Adgco.topLeftCorner(3, 3).transpose();
      for (int i = 0; i < 3; i++) {
        ContactPoint gpt;
        gpt.p = pt.p + r * Roc * (xs.row(i)).transpose();
        gpt.n = pt.n;
        gpt.d = 0;
        point_contacts->push_back(gpt);
      }
    }
  } else {
    copy_points(fingertips, point_contacts);
  }
}

void DartPointManipulator::setupCollisionGroup(WorldPtr world) {
  auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();
  this->mCollisionGroup = collisionEngine->createCollisionGroup();
  for (int i = 0; i < this->n_pts; i++) {
    this->mCollisionGroup->addShapeFramesOf(this->bodies[i].get());
  }
}

void DartPointManipulator::set_workspace_limit(
    std::vector<Vector6d> workspace_limits) {
  // workspace limits with the format of [x_min, x_max, y_min, y_max, z_min,
  // z_max]

  std::vector<Vector3d> colors;
  colors.push_back(Vector3d(252.0/256.0, 32.0/256.0, 3.0/256.0)); // red
  colors.push_back(Vector3d(252.0/256.0, 169.0/256.0, 3.0/256.0)); // orange
  // colors.push_back(Vector3d(252.0/256.0, 223.0/256.0, 3.0/256.0)); // yellow
  colors.push_back(Vector3d(132.0/256.0, 252.0/256.0, 3.0/256.0)); // green
  colors.push_back(Vector3d(3.0/256.0, 252.0/256.0, 207.0/256.0)); // gree-blue
  colors.push_back(Vector3d(61.0/256.0, 3.0/256.0, 252.0/256.0)); // blue
  colors.push_back(Vector3d(248.0/256.0, 3.0/256.0, 252.0/256.0)); // purple

  this->is_workspace_limit = true;
  this->workspace_limits = workspace_limits;

  // create fixed box for every workspace limit
  for (int i = 0; i < workspace_limits.size(); i++) {
    Vector6d wl = workspace_limits[i];
    double lx = wl[1] - wl[0];
    double ly = wl[3] - wl[2];
    double lz = wl[5] - wl[4];

    SkeletonPtr wl_box =
        createFixedBox("ws_" + std::to_string(i), Vector3d(lx, ly, lz),
                       Vector3d(wl[0] + lx / 2, wl[2] + ly / 2, wl[4] + lz / 2),
                       colors[i], 0.1);
    this->addBody(wl_box);
  }
}

bool DartPointManipulator::ifIKsolution(const VectorXd &mnp_config,
                                        const Vector7d &object_pose) {

  if (!this->is_workspace_limit) {
    return true;
  }
  Eigen::Matrix4d T;
  T = pose2SE3(object_pose);
  Eigen::Matrix3d R;
  R = T.block(0, 0, 3, 3);
  Eigen::Vector3d p;
  p = T.block(0, 3, 3, 1);

  int n = int(mnp_config.size() / 6);
  for (int i = 0; i < this->n_pts; i++) {
    if (std::isnan(mnp_config[6 * i])) {
      continue;
    }
    Vector3d pos;
    if (i < n) {
      pos = R * Vector3d(mnp_config[6 * i], mnp_config[6 * i + 1],
                                 mnp_config[6 * i + 2]) +
                    p;
    }
    if (pos[0] > workspace_limits[i][1] || pos[0] < workspace_limits[i][0] ||
        pos[1] > workspace_limits[i][3] || pos[1] < workspace_limits[i][2] ||
        pos[2] > workspace_limits[i][5] || pos[2] < workspace_limits[i][4]) {
      return false;
    }
  }
  return true;
}

void DartPointManipulator::points_in_workspace(int finger_idx, std::vector<ContactPoint> object_surface_world, std::vector<int>* points_in_ws){
  
  
  for (int i = 0; i < object_surface_world.size(); i++) {
    Vector3d pos = object_surface_world[i].p;
    if (pos[0] > workspace_limits[finger_idx][1] || pos[0] < workspace_limits[finger_idx][0] ||
        pos[1] > workspace_limits[finger_idx][3] || pos[1] < workspace_limits[finger_idx][2] ||
        pos[2] > workspace_limits[finger_idx][5] || pos[2] < workspace_limits[finger_idx][4]) {
      continue;
    }
    else{
      points_in_ws->push_back(i);
    }
  }
  return;

}
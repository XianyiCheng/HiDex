#include "DartDDHandScalable.h"

#include "../utilities/sample.h"

#ifndef CONTACTKINEMATICS_H
#define CONTACTKINEMATICS_H
#include "../contacts/contact_kinematics.h"
#endif

#ifndef UTILS_H
#define UTILS_H
#include "../utilities/utilities.h"
#endif

void add_circle_contacts(ContactPoint pt, double r,
                         std::vector<ContactPoint> *point_contacts) {

  Matrix3d xs;
  xs << 1, 0, 0, -0.5, 0.865, 0, -0.5, -0.865, 0;

  Matrix6d Adgco = contact_jacobian(pt.p, pt.n);
  Matrix3d Roc = Adgco.topLeftCorner(3, 3).transpose();
  for (int i = 0; i < 3; i++) {
    ContactPoint gpt;
    gpt.p = pt.p + r * Roc * (xs.row(i)).transpose();
    gpt.n = pt.n;
    gpt.d = 0;
    point_contacts->push_back(gpt);
  }
  return;
}

Eigen::Vector3d quat2aaxis_(double x, double y, double z, double w) {
  Eigen::Quaterniond q(w, x, y, z);
  Eigen::AngleAxisd aa(q);
  Eigen::Vector3d aaxis = aa.axis() * aa.angle();
  return aaxis;
}

Matrix3d quat2rotm_(double x, double y, double z, double w) {
  Eigen::Quaterniond q(w, x, y, z);
  // Eigen::AngleAxisd aa(q);
  // Eigen::Vector3d aaxis = aa.axis()*aa.angle();
  return q.toRotationMatrix();
}

Vector3d znormal2aaxis_(Vector3d normal) {
  Vector3d z(0, 0, 1);
  Vector3d rotvec = z.cross(normal);
  Matrix3d R;

  Vector3d aa;

  if ((rotvec.norm() < 1e-6) && (signbit(z[2]) == signbit(normal[2]))) {
    aa = Vector3d(0, 0, 0);

  } else if ((rotvec.norm() < 1e-6) && (signbit(z[2]) != signbit(normal[2]))) {
    aa = Vector3d(M_PI, 0, 0);
  } else {
    rotvec = rotvec * (1 / rotvec.norm());
    double rotangle = acos(z.dot(normal) / normal.norm());
    AngleAxisd aaxis(rotangle, rotvec);
    aa = aaxis.axis() * aaxis.angle();
  }
  return aa;
}

void getActiveFingerIdx_(const VectorXd &config, std::vector<int> *idx) {
  if (config.size() < 12) {
    return;
  }
  double n0 = config.segment<3>(3).norm();
  double n1 = config.segment<3>(9).norm();
  if (n0 > 0.5)
    idx->push_back(0);
  if (n1 > 0.5)
    idx->push_back(1);
}

DartDDHandScalable::DartDDHandScalable(int fingertype, double scale) {

  hand_ik = new HandIK(74E-3, 50E-3, 30E-3);

  this->ifCheckObjectCollision = false;
  this->n_pts = 2;
  this->fingertype = fingertype;
  this->scale = scale;

  Vector3d color = Vector3d(0.3, 0.3, 0.8);

  if (fingertype == I_FINGER) {

    for (int i = 0; i < n_pts; i++) {
      SkeletonPtr ball = createFreeBall("fingertip_" + std::to_string(i),
                                        scale * radius, color);
      this->addBody(ball);
    }
    // add fingers
    for (int i = 0; i < n_pts; i++) {
      SkeletonPtr cylinder =
          createFreeCylinder("finger_" + std::to_string(i), scale * radius,
                             scale * this->finger_length, color);
      // SkeletonPtr cylinder = createFreeBox("finger_"+std::to_string(i),
      // Vector3d(radius*1, radius*1, this->finger_length), color);
      this->addBody(cylinder);
    }

  } else if (fingertype == L_FINGER) {
    // add fingers
    for (int i = 0; i < n_pts; i++) {
      SkeletonPtr cylinder =
          createFreeCylinder("finger_" + std::to_string(i), scale * radius,
                             scale * this->finger_length, color);
      // SkeletonPtr cylinder = createFreeBox("finger_"+std::to_string(i),
      // Vector3d(2*radius, 2*radius, this->finger_length), color);
      this->addBody(cylinder);
    }
  }
  // add hand
  SkeletonPtr handbox = createFreeBox(
      "hand", Vector3d(scale * 0.2, scale * 60E-3, scale * 60E-3), color);
  this->addBody(handbox);
}

void DartDDHandScalable::setHandFrameTransform(const Vector7d &pose) {
  hand_pose = pose;
  this->bodies.back()->setPositions(pose7d_to_pose6d(hand_pose));
  // VectorXd empty(0);
  // Vector7d x;
  // this->setConfig(empty, x);
}

void DartDDHandScalable::setupCollisionGroup(WorldPtr world) {

  auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();

  if (fingertype == I_FINGER) {
    this->mCollisionGroup = collisionEngine->createCollisionGroup(
        this->bodies[0].get(), this->bodies[1].get(), this->bodies[2].get(),
        this->bodies[3].get());
  } else if (fingertype == L_FINGER) {
    this->mCollisionGroup = collisionEngine->createCollisionGroup(
        this->bodies[0].get(), this->bodies[1].get());
  }
}

void DartDDHandScalable::getFingertipsOnObject(
    const VectorXd &config, const Vector7d &object_pose,
    std::vector<ContactPoint> *fingertips) {

  this->obj_pose = object_pose;

  std::vector<int> idx;
  getActiveFingerIdx_(config, &idx);

  for (auto i : idx) {
    fingertips->push_back(
        ContactPoint(config.segment<3>(6 * i), config.segment<3>(6 * i + 3)));
  }
}

void DartDDHandScalable::setConfig(const VectorXd &config,
                                   const Vector7d &object_pose) {

  // first set all fingers to its reset position
  Eigen::Vector6d pos(Eigen::Vector6d::Zero());
  pos.tail(3) = hand_pose.head(3);
  for (int i = 0; i < this->bodies.size(); i++) {
    this->bodies[i]->setPositions(pos);
  }

  this->obj_pose = object_pose;

  Matrix3d R_WH =
      quat2rotm_(hand_pose[3], hand_pose[4], hand_pose[5], hand_pose[6]);

  Matrix4d T = pose2SE3(object_pose);
  Matrix3d R_WO = T.block(0, 0, 3, 3);
  Vector3d p_WO = T.block(0, 3, 3, 1);

  Vector3d finger_axis;

  if (fingertype == I_FINGER) {
    finger_axis = R_WH * Vector3d(0, 0, 1);
  } else if (fingertype == L_FINGER) {
    finger_axis = R_WH * Vector3d(0, 1, 0);
  }

  Vector3d orn = znormal2aaxis_(finger_axis);

  {
    Eigen::Vector6d pos(Eigen::Vector6d::Zero());
    pos.head(3) = orn;

    std::vector<int> idx;
    getActiveFingerIdx_(config, &idx);

    for (auto i : idx) {
      if (fingertype == I_FINGER) {

        Vector3d pp = R_WO * (Vector3d(config[6 * i + 0], config[6 * i + 1],
                                       config[6 * i + 2]) -
                              scale * radius *
                                  Vector3d(config[6 * i + 3], config[6 * i + 4],
                                           config[6 * i + 5])) +
                      p_WO;

        pos.tail(3) = pp + scale * radius * 1.1 * finger_axis;
        this->bodies[i]->setPositions(pos);
        pos.tail(3) = pp + 0.5 *
                               (scale * finger_length + 1.2 * scale * radius) *
                               finger_axis;
        this->bodies[i + 2]->setPositions(pos);

      } else if (fingertype == L_FINGER) {
        Vector3d pp = R_WO * (Vector3d(config[6 * i + 0], config[6 * i + 1],
                                       config[6 * i + 2])) +
                      p_WO;

        pos.tail(3) = pp;
        this->bodies[i]->setPositions(pos);
      }
    }
  }
}

bool DartDDHandScalable::resampleFingers(
    int n_on, const VectorXd &config, const Vector7d &object_pose,
    const std::vector<ContactPoint> &object_surface, VectorXd &new_config,
    std::vector<ContactPoint> *remain_fingertips) {

  // n_on: number of relocating fingers
  if (n_on == 0) {
    return false;
  }

  int N = object_surface.size();

  std::vector<int> located_idx;
  getActiveFingerIdx_(config, &located_idx);

  for (int sample = 0; sample < 20; sample++) {

    std::vector<ContactPoint> remain_mnps;

    VectorXd new_sample(12);
    if (config.size() < 12) {
      new_sample << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    } else {
      new_sample = config;
    }

    if (n_on == 1) {
      // randomly choose a finger to relocate
      int finger_idx = randi(2);
      int sample_idx = randi(N);

      new_sample.segment<3>(6 * finger_idx) = object_surface[sample_idx].p;
      new_sample.segment<3>(6 * finger_idx + 3) = object_surface[sample_idx].n;

      // if the finger is not located before
      if (std::find(located_idx.begin(), located_idx.end(), finger_idx) ==
          located_idx.end()) {
        remain_mnps.push_back(object_surface[sample_idx]);
      }

      for (auto i : located_idx) {
        // add the finger that's not relocating
        if (i != finger_idx) {
          remain_mnps.push_back(ContactPoint(config.segment<3>(6 * i),
                                             config.segment<3>(6 * i + 3)));
        }
      }
    }

    if (n_on == 2) {

      int sample_idx = randi(N);

      new_sample.segment<3>(0) = object_surface[sample_idx].p;
      new_sample.segment<3>(3) = object_surface[sample_idx].n;

      int sample_idx1 = randi(N);

      while (sample_idx1 == sample_idx && N > 1) {
        sample_idx1 = randi(N);
      }

      new_sample.segment<3>(6) = object_surface[sample_idx1].p;
      new_sample.segment<3>(9) = object_surface[sample_idx1].n;

      // if the finger is not located before

      if (std::find(located_idx.begin(), located_idx.end(), 0) ==
          located_idx.end()) {
        remain_mnps.push_back(object_surface[sample_idx]);
      }

      if (std::find(located_idx.begin(), located_idx.end(), 1) ==
          located_idx.end()) {
        remain_mnps.push_back(object_surface[sample_idx1]);
      }
    }

    if (this->isIK(new_sample, object_pose)) {
      new_config = new_sample;
      copy_points(remain_mnps, remain_fingertips);
      return true;
    }
  }

  printf("Failed to sample a manipulator contact! Please check your IK. \n");
  return false;
}

void DartDDHandScalable::Fingertips2PointContacts(
    const std::vector<ContactPoint> &fingertips,
    std::vector<ContactPoint> *point_contacts) {
  //   double r = scale * this->radius / 3;
  double r = scale * this->radius;
  if (fingertype == I_FINGER) {

    for (auto &pt : fingertips) {
      add_circle_contacts(pt, r, point_contacts);
    }

  } else if (fingertype == L_FINGER) {
    Matrix3d R_WH =
        quat2rotm_(hand_pose[3], hand_pose[4], hand_pose[5], hand_pose[6]);
    Matrix3d R_WO =
        quat2rotm_(obj_pose[3], obj_pose[4], obj_pose[5], obj_pose[6]);
    Matrix3d R_HO = R_WH.transpose() * R_WO;
    Vector3d finger_axis = R_HO * Vector3d::UnitY();

    for (auto fpt : fingertips) {
      ContactPoint p1;
      ContactPoint p2;
      p1.p = fpt.p + 0.5 * this->finger_length * finger_axis * scale;
      p2.p = fpt.p - 0.5 * this->finger_length * finger_axis * scale;
      p1.n = fpt.n;
      p2.n = fpt.n;
      //   point_contacts->push_back(p1);
      //   point_contacts->push_back(p2);
      add_circle_contacts(p1, r, point_contacts);
      add_circle_contacts(p2, r, point_contacts);
    }
  }
}

bool DartDDHandScalable::isIK(const VectorXd &config,
                              const Vector7d &object_pose) {

  std::vector<int> idx;
  getActiveFingerIdx_(config, &idx);

  int n = idx.size();

  if (n == 0) {
    return true;
  }
  // // hard code for peg out, delete later!!
  // if (std::find(idx.begin(), idx.end(), 1) != idx.end()) {
  //   if (config.segment(9, 3).norm() < 1.2) {
  //     if (config[11] > 0.8) {
  //       return false;
  //     }
  //   }
  // }

  // compute p1 p2 in the hand frame
  Matrix4d T_WH = pose2SE3(hand_pose);
  Matrix4d T_WO = pose2SE3(object_pose);
  Matrix4d T_HO = SE3Inv(T_WH) * T_WO;

  Matrix3d R_HO = T_HO.block(0, 0, 3, 3);
  Vector3d p_HO = T_HO.block(0, 3, 3, 1);

  std::vector<Vector2d> pts;
  pts.push_back(Vector2d(0, 0));
  pts.push_back(Vector2d(0, 0));

  if (idx.size() == 2) {
    if ((config.segment(0, 3) - config.segment(6, 3)).norm() <
        2.5 * scale * this->radius) {
      return false;
    }
    // finger 1 must be on the left to finger 2
    if (config[0] > config[6]) {
      return false;
    }
  }

  for (auto i : idx) {
    Vector3d pp = R_HO * config.segment<3>(6 * i) + p_HO;

    if (i == 0) {

      if (isBoxConstraints_0) {
        if ((ub_0 - pp).minCoeff() < 0 || (pp - lb_0).minCoeff() < 0) {
          return false;
        }
      }

      pp[0] = pp[0] / scale - dx;
      pp[2] = pp[2] / scale + dz;

      pts[i] = Vector2d(pp[0], pp[2]);

    } else {

      if (isBoxConstraints_1) {
        if ((ub_1 - pp).minCoeff() < 0 || (pp - lb_1).minCoeff() < 0) {
          return false;
        }
      }

      pp[0] = pp[0] / scale + dx;
      pp[2] = pp[2] / scale + dz;

      pts[i] = Vector2d(pp[0], pp[2]);
    }

    // if ((ws_ub - pp).minCoeff() < 0){
    //     return false;
    // }
    // if ((pp - ws_lb).maxCoeff() < 0){
    //     return false;
    // }
  }

  if (idx.size() == 2) {
    if (pts[1][0] - pts[0][0] < 2 * dx) {
      return false;
    }
  }

  bool if_hand_ik = hand_ik->is_inverse_kinematics(pts[0], pts[1]);
  return if_hand_ik;

  // use external ik
}

bool DartDDHandScalable::setBoxConstraints(int side, const Vector3d &ub,
                                           const Vector3d &lb) {

  if (side == FINGER_0) {
    isBoxConstraints_0 = true;
    ub_0 = ub;
    lb_0 = lb;
  }

  if (side == FINGER_1) {
    isBoxConstraints_1 = true;
    ub_1 = ub;
    lb_1 = lb;
  }

  return true;
}

VectorXd DartDDHandScalable::getHandFrameConfig(const VectorXd &config,
                                                const Vector7d &object_pose) {
  int n = int(config.size() / 6);

  std::vector<Vector2d> pts;

  pts.push_back(Vector2d(-0.04, -0.05));
  pts.push_back(Vector2d(0.04, -0.05));

  if (n > 0) {

    // compute p1 p2 in the hand frame
    Matrix4d T_WH = pose2SE3(hand_pose);
    Matrix4d T_WO = pose2SE3(object_pose);
    Matrix4d T_HO = SE3Inv(T_WH) * T_WO;

    Matrix3d R_HO = T_HO.block(0, 0, 3, 3);
    Vector3d p_HO = T_HO.block(0, 3, 3, 1);

    std::vector<int> idx;
    getActiveFingerIdx_(config, &idx);

    for (auto i : idx) {
      // Vector3d pp = R_HO*(config.segment<3>(6*i) +
      // 0.001*config.segment<3>(6*i+3)) + p_HO;
      Vector3d pp = R_HO * config.segment<3>(6 * i) + p_HO;

      if (i == 0) {

        pp[0] = pp[0] / scale - dx;
        pp[2] = pp[2] / scale + dz;

        pts[i] = Vector2d(pp[0], pp[2]);

      } else {

        pp[0] = pp[0] / scale + dx;
        pp[2] = pp[2] / scale + dz;

        pts[i] = Vector2d(pp[0], pp[2]);
      }
    }
  }

  VectorXd rc(4);
  rc << pts[0][0], pts[0][1], pts[1][0], pts[1][1];

  return rc;
}
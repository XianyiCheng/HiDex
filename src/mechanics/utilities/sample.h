#ifndef UTILS_H
#define UTILS_H
#include "utilities.h"
#endif

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

Vector3d sample_position(const Vector3d &ub, const Vector3d &lb);
Quaterniond generate_unit_quaternion();
Quaterniond sample_rotation(const Vector3d &axis);

Vector3d steer_position(const Vector3d &p0, const Eigen::Vector3d &p1,
                        const double &length);
Quaterniond steer_quaternion(const Quaterniond &q0, const Quaterniond &q1,
                             const double &angle);
Vector7d steer_config(Vector7d x_near, Vector7d x_rand,
                      double epsilon_translation, double epsilon_angle);

void steer_velocity(Vector6d &x, double h, double cl);   

void combinations(const std::vector<int> &elems, int req_len,
                  std::vector<std::vector<int>> *elem_combs);

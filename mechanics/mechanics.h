
#include <vector>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utilities/utilities.h"

#ifndef CONTACTKINEMATICS_H
#define CONTACTKINEMATICS_H
#include "contacts/contact_kinematics.h"
#endif

#ifndef CONTACTCONSTRAINTS_H
#define CONTACTCONSTRAINTS_H
#include "contacts/contact_constraints.h"
#endif

bool force_balance(const std::vector<Vector6d> &mnp_contacts,
                   const std::vector<Vector6d> &env_contacts,
                   const VectorXi &env_mode, const Vector6d &f_ext_w,
                   const Vector7d object_pose, double mu_env, double mu_mnp);

bool quasistatic_check(const std::vector<ContactPoint> &mnp_contacts,
                       const std::vector<ContactPoint> &env_contacts,
                       const Vector6d v, const Vector7d object_pose,
                       const Vector6d &f_ext_w, double mu_env, double mu_mnp);

bool quasistatic_check_2(const std::vector<Vector6d> &mnps,
                       const std::vector<Vector6d> &envs,
                       const Vector6d v, const Vector7d object_pose,
                       const Vector6d &f_ext_w, double mu_env, double mu_mnp);
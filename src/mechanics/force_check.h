
#include <vector>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifndef UTILS_H
#define UTILS_H
#include "utilities/utilities.h"
#endif

#ifndef CONTACTKINEMATICS_H
#define CONTACTKINEMATICS_H
#include "contacts/contact_kinematics.h"
#endif

#ifndef CONTACTCONSTRAINTS_H
#define CONTACTCONSTRAINTS_H
#include "contacts/contact_constraints.h"
#endif

bool isQuasistatic(const std::vector<ContactPoint> &mnps,
                   const std::vector<ContactPoint> &envs,
                   const VectorXi &env_mode, const Vector6d &f_ext_w,
                   const Vector7d object_pose, double mu_env, double mu_mnp,
                   ContactConstraints *cons);

bool isQuasidynamic(const Vector6d &v_b, const std::vector<ContactPoint> &mnps,
                    const std::vector<ContactPoint> &envs,
                    const VectorXi &env_mode, const Vector6d &f_ext_w,
                    const Matrix6d &object_inertia, const Vector7d object_pose,
                    double mu_env, double mu_mnp, double wa, double wt,
                    double h_time, ContactConstraints *cons, double thr);

bool isQuasidynamic_LP(const Vector6d &v_b, const std::vector<ContactPoint> &mnps,
                    const std::vector<ContactPoint> &envs,
                    const VectorXi &env_mode, const Vector6d &f_ext_w,
                    const Matrix6d &object_inertia, const Vector7d object_pose,
                    double mu_env, double mu_mnp, double wa, double wt,
                    double h_time, ContactConstraints *cons, double thr);

bool is_force_closure(Vector7d x, const std::vector<ContactPoint> &mnps,
                   double friction_coeff);

Vector6d EnvironmentConstrainedVelocity(const Vector6d &v_goal,
                                        const std::vector<ContactPoint> &envs,
                                        const VectorXi &env_mode,
                                        ContactConstraints &cons);

Vector6d EnvironmentConstrainedVelocity_CSModeOnly(
    const Vector6d &v_goal, const std::vector<ContactPoint> &envs,
    const VectorXi &mode, ContactConstraints &cons);
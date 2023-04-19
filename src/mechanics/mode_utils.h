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

VectorXi mode_from_velocity(const Vector6d &v,
                            const std::vector<ContactPoint> &envs,
                            ContactConstraints *cons);

bool contactTrack(ContactPoint pt0, ContactPoint pt1,
                  double normal_product = 0.85, double thr = 0.1);

VectorXi track_contacts_remain(const std::vector<ContactPoint> &pts,
                               const std::vector<ContactPoint> &pts_new,
                               double normal_product = 0.85, double thr = 0.1);

VectorXi cs_mode_from_contacts(const std::vector<ContactPoint> &pts,
                               const std::vector<ContactPoint> &pts_new);

VectorXi conservative_cs_mode(const VectorXi &m1, const VectorXi &m2);

VectorXi conservative_mode_from_velocity(const std::vector<ContactPoint> &envs,
                   const VectorXi &ref_cs_mode, const Vector6d &v, ContactConstraints *cons);
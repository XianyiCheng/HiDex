#ifndef UTILS_H
#define UTILS_H
#include "../../utilities/utilities.h"
#endif
#ifndef CONTACTCONSTRAINTS_H
#define CONTACTCONSTRAINTS_H
#include "../../contacts/contact_constraints.h"
#endif

#include "../../contacts/contact_kinematics.h"


double BoxSDF(const Vector3d &box_shape, const Vector7d &box_pose, const Vector3d &pt);

double PointsSDF(const std::vector<ContactPoint>& points, const Vector3d &pt);

#include <vector>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utilities/utilities.h"

bool force_balance(const std::vector<Vector6d> &mnp_contacts,
                   const std::vector<Vector6d> &env_contacts,
                   const VectorXi &env_mode,
                   const Vector6d &f_ext_w, const Vector7d object_pose, double mu_env, double mu_mnp);
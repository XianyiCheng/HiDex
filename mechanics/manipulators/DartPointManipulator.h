#ifndef MANIPULATORS_DARTMANIPULATORTEMPLATE
#define MANIPULATORS_DARTMANIPULATORTEMPLATE
#include "DartManipulatorTemplate.h"
#endif

#ifndef MANIPULATORS_MANIPULATORTEMPLATE
#define MANIPULATORS_MANIPULATORTEMPLATE
#include "ManipulatorTemplate.h"
#endif

#ifndef DART_UTILS
#define DART_UTILS
#include "../dart_utils/dart_utils.h"
#endif

class DartPointManipulator : public virtual DartManipulatorTemplate {

public:
  // config: [p1, n1, p2, n2 ...] in the object frame

  double fingertip_radius = 0;

  bool is_patch_contact = false;

  bool is_workspace_limit = false;

  std::vector<Vector6d> workspace_limits;

  DartPointManipulator(int n, double radius);
  DartPointManipulator(const DartPointManipulator &dpm)
      : DartManipulatorTemplate(dpm), fingertip_radius(dpm.fingertip_radius) {}

  void setupCollisionGroup(WorldPtr world);

  void getFingertipsOnObject(const VectorXd &config,
                             const Vector7d &object_pose,
                             std::vector<ContactPoint> *fingertips);

  void setConfig(const VectorXd &config, const Vector7d &object_pose);

  bool resampleFingers(int n_on, const VectorXd &config,
                       const Vector7d &object_pose,
                       const std::vector<ContactPoint> &object_surface,
                       VectorXd &new_config,
                       std::vector<ContactPoint> *remain_fingertips);

  void Fingertips2PointContacts(const std::vector<ContactPoint> &fingertips,
                                std::vector<ContactPoint> *point_contacts);

  void set_workspace_limit(std::vector<Vector6d> workspace_limits);

  bool ifIKsolution(const VectorXd &mnp_config, const Vector7d &object_pose);

  void
  points_in_workspace(int finger_idx,
                      std::vector<ContactPoint> object_surface_world,
                      std::vector<int> *points_in_ws);
};
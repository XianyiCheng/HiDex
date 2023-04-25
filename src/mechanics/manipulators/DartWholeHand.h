#ifndef MANIPULATORS_DARTURDFMANIPULATORTEMP
#define MANIPULATORS_DARTURDFMANIPULATORTEMP
#include "DartURDFManipulatorTemplate.h"
#endif

class DartWholeHandManipulator : public virtual DartURDFManipulatorTemplate
{

public:
    // config: joint angles
    // n_pts: number of hand segments

    DartWholeHandManipulator(const std::string &manipulator_folder, double patch_contact_radius);

    bool ifCollision(const VectorXd &mnp_config, const Vector7d &object_pose, std::shared_ptr<WorldTemplate> world);

    void setConfig(const VectorXd &config, const Vector7d &object_pose) override;

    bool resampleFingers(int n_on, const VectorXd &config, const Vector7d &object_pose, const std::vector<ContactPoint> &object_surface,
                         VectorXd &new_config, std::vector<ContactPoint> *remain_fingertips) override;

    void getFingertipsOnObject(const VectorXd &config, const Vector7d &object_pose, std::vector<ContactPoint> *fingertips) override;

    // contact model
    void Fingertips2PointContacts(const std::vector<ContactPoint> &fingertips, std::vector<ContactPoint> *point_contacts) override;

    bool ifIKsolution(const VectorXd &mnp_config, const Vector7d &object_pose) override;

    void points_in_workspace(int finger_idx, std::vector<ContactPoint> object_surface_world, std::vector<int> *points_in_ws) override;

    void setupCollisionGroup(WorldPtr world) override;

    void setConfig(const VectorXd &config, const Vector7d &object_pose) override;

    bool inverseKinematics(const VectorXd &mnp_config, const Vector7d &object_pose, VectorXd &result_joint_config) override;

    void computeWorkspace(std::string save_to) override;

    std::vector<std::string> getPartNames() const;

private:
    double patch_contact_radius = 0;

    std::vector<std::string> part_names;
};
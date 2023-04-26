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

    void setConfig(const VectorXd &config, const Vector7d &object_pose) override;

    bool resampleFingers(int n_on, const VectorXd &config, const Vector7d &object_pose, const std::vector<ContactPoint> &object_surface,
                         VectorXd &new_config, std::vector<ContactPoint> *remain_fingertips) override
    {
        // We will not use this function in WholeHandTask
        return false;
    }

    // contact model
    void Fingertips2PointContacts(const std::vector<ContactPoint> &fingertips, std::vector<ContactPoint> *point_contacts) override;

    bool ifIKsolution(const VectorXd &mnp_config, const Vector7d &object_pose) override{
        // This function is not used in WholeHandTask
        return true;
    }

    void points_in_workspace(int finger_idx, std::vector<ContactPoint> object_surface_world, std::vector<int> *points_in_ws) override{
        // This function is only used in TASK::sample_likely_feasible_finger_idx, for increasing the sampling success mostly for inhand manipulation
        // We will not use this function in WholeHandTask
        return;
    }

    void setupCollisionGroup(WorldPtr world) override;

    bool inverseKinematics(const VectorXd &mnp_config, const Vector7d &object_pose, VectorXd &result_joint_config) override{
        // This function is not used in WholeHandTask
        return true;
    }

    void computeWorkspace(std::string save_to) override{
        // This function is not used in WholeHandTask
        return;
    }

    std::vector<std::string> getPartNames() const;

    bool roughIKsolutions(const std::vector<std::string> &part_names, const std::vector<ContactPoint> &object_contacts, const Vector7d &object_pose, std::vector<VectorXd> *rough_ik_solutions = nullptr);

private:
    double patch_contact_radius = 0;

    std::vector<std::string> part_names;

    VectorXd mJointLowerLimits;
    VectorXd mJointUpperLimits;
};
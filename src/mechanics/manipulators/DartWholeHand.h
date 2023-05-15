#ifndef MANIPULATORS_DARTURDFMANIPULATORTEMP
#define MANIPULATORS_DARTURDFMANIPULATORTEMP
#include "DartURDFManipulatorTemplate.h"
#endif

#ifndef _DART_WORLD
#define _DART_WORLD
#include "../worlds/DartWorld.h"
#endif

class DartWholeHandManipulator : public virtual DartURDFManipulatorTemplate
{

public:
    // config: joint angles
    // n_pts: number of hand segments

    DartWholeHandManipulator(const std::string &manipulator_folder, double patch_contact_radius);

    void setConfig(const VectorXd &config, const Vector7d &object_pose) override;

    void setSpheres(const std::vector<ContactPoint> &fingertips, const Vector7d &object_pose);

    void setSpheres(VectorXd positions, const Vector7d &object_pose);

    bool resampleFingers(int n_on, const VectorXd &config, const Vector7d &object_pose, const std::vector<ContactPoint> &object_surface,
                         VectorXd &new_config, std::vector<ContactPoint> *remain_fingertips) override
    {
        // We will not use this function in WholeHandTask
        return false;
    }

    // contact model
    void Fingertips2PointContacts(const std::vector<ContactPoint> &fingertips, std::vector<ContactPoint> *point_contacts) override;

    bool ifIKsolution(const VectorXd &mnp_config, const Vector7d &object_pose) override
    {
        // This function is not used in WholeHandTask
        return true;
    }

    void points_in_workspace(int finger_idx, std::vector<ContactPoint> object_surface_world, std::vector<int> *points_in_ws) override
    {
        // This function is only used in TASK::sample_likely_feasible_finger_idx, for increasing the sampling success mostly for inhand manipulation
        // We will not use this function in WholeHandTask
        return;
    }

    void setupCollisionGroup(WorldPtr world) override;

    bool inverseKinematics(const VectorXd &mnp_config, const Vector7d &object_pose, VectorXd &result_joint_config) override
    {
        // This function is not used in WholeHandTask
        return true;
    }

    void computeWorkspace(std::string save_to) override
    {
        // This function is not used in WholeHandTask
        return;
    }

    std::vector<std::string> getPartNames() const;

    double getPatchContactRadius() const
    {
        return this->patch_contact_radius;
    }

    VectorXd random_sample_config(double unbounded_max=100, double unbounded_min=-100) const;

    std::vector<ContactPoint> get_points_in_world(const std::vector<std::string> &part_names, const std::vector<int> & part_point_idxes, const VectorXd &config);

    bool roughIKsolutions(const std::vector<std::string> &part_names, const std::vector<int> &part_point_idxes, const std::vector<ContactPoint> &object_contacts, const Vector7d &object_pose, std::vector<VectorXd> *rough_ik_solutions = nullptr);

    bool roughSDFIKsolutions(const std::vector<std::string> &part_names, const std::vector<int> &part_point_idxes, const std::vector<ContactPoint> &object_contacts, const Vector7d &object_pose, const std::vector<ContactPoint> &repell_object_contacts, Vector3d box_shape, int n_sample_points, std::vector<VectorXd> *rough_ik_solutions = nullptr);
    
    bool roughCollisionCheck(const std::vector<ContactPoint> &object_contacts, const Vector7d &object_pose, std::shared_ptr<DartWorld> world);

    void preprocess(const std::vector<std::string> &allowed_part_names, const std::vector<int> &allowed_part_point_idxes, int maximum_simultaneous_contact=-1);

    bool ifConsiderPartPairs(int i, int j, double contact_distance);

    double maxPenetrationDistance(const VectorXd &config, const Vector7d & object_pose, const Vector3d &object_box_shape, int n_sample_points=100000); 
    double averagePenetrateDistance(const VectorXd &config, const Vector7d &object_pose, const Vector3d &object_box_shape, int n_sample_points=100000);
    double maxPenetrationDistance(const VectorXd &config, const Vector7d & object_pose, const std::vector<ContactPoint> &object_surface_points, int n_sample_points=100000);

    double signedDistance(const std::string &part_name, const Vector3d &p);

    std::vector<std::string> allowed_part_names;
    std::vector<int> allowed_part_point_idxes;
    std::vector<ContactPoint> allowed_part_points;

    int maximum_simultaneous_contact = 0;

private:
    double patch_contact_radius = 0;

    std::vector<std::string> part_names;

    VectorXd mJointLowerLimits;
    VectorXd mJointUpperLimits;

    bool if_spheres_added = false;

    std::shared_ptr<CollisionGroup> sphereCollisionGroup = 0;

    // std::vector<std::vector<int>> sampled_idxes_for_sdf;
};
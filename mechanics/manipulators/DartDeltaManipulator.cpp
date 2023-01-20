#include "DartDeltaManipulator.h"

#include "../utilities/sample.h"

#include "../contacts/contact_kinematics.h"

bool is_in_cylindar(Vector3d p, Vector3d loc, double r, double h){
    if (p[2] < loc[2] || p[2] > loc[2] + h){
        return false;
    }
    if (((p[0] - loc[0])*(p[0] - loc[0]) + (p[1] - loc[1])*(p[1] - loc[1])) > r*r){
        return false;
    }
    return true;
}

DartDeltaManipulator::DartDeltaManipulator(int n, double radius, double workspace_radius, double workspace_height, std::vector<Vector3d> robot_locations)
{

    this->n_pts = n;
    this->fingertip_radius = radius;

    for (int i = 0; i < n; i++)
    {
        SkeletonPtr ball = createFreeBall("ball_" + std::to_string(i), radius,
                                          Vector3d(0.3, 0.3, 0.8));
        this->addBody(ball);
    }
    this->workspace_radius = workspace_radius;
    this->workspace_height = workspace_height;
    this->robot_locations = robot_locations;

    // create fixed box for every workspace limit
    for (int i = 0; i < this->n_pts; i++)
    {
        Vector3d loc = robot_locations[i];
        loc[2] += workspace_height / 2;
        SkeletonPtr wl_cylindar =
            createFixedCylindar("ws_" + std::to_string(i), workspace_radius, workspace_height,
                                loc,
                                Vector3d(0.3, 0.4, 0.6), 0.2);
        this->addBody(wl_cylindar);
    }
}

void DartDeltaManipulator::getFingertipsOnObject(
    const VectorXd &config, const Vector7d &object_pose,
    std::vector<ContactPoint> *fingertips)
{

    int n = int(config.size() / 6);

    if (fingertips->size() != 0)
    {
        fingertips->clear();
    }

    for (int i = 0; i < n; i++)
    {
        fingertips->push_back(
            ContactPoint(config.segment<3>(6 * i), config.segment<3>(6 * i + 3)));
    }
}

void DartDeltaManipulator::setConfig(const VectorXd &config,
                                     const Vector7d &object_pose)
{

    Eigen::Matrix4d T;
    T = pose2SE3(object_pose);
    Eigen::Matrix3d R;
    R = T.block(0, 0, 3, 3);
    Eigen::Vector3d p;
    p = T.block(0, 3, 3, 1);

    int n = int(config.size() / 6);
    for (int i = 0; i < this->n_pts; i++)
    {
        Eigen::Vector6d pos(Eigen::Vector6d::Zero());
        Vector3d pp;
        if (std::isnan(config[6 * i]))
        {
            pp << 100, 100, 100;
        }
        else
        {
            pp << config[6 * i], config[6 * i + 1], config[6 * i + 2];
        }
        if (i < n)
        {
            pos.tail(3) = R * pp + p;
        }
        this->bodies[i]->setPositions(pos);
    }
}

bool DartDeltaManipulator::resampleFingers(
    int n_on, const VectorXd &config, const Vector7d &object_pose,
    const std::vector<ContactPoint> &object_surface, VectorXd &new_config,
    std::vector<ContactPoint> *remain_fingertips)
{

    // n_on: number of relocating fingers
    if (n_on == 0)
    {
        return false;
    }

    std::vector<ContactPoint> fingertips;
    Vector7d x_o;
    getFingertipsOnObject(config, x_o, &fingertips);

    std::vector<ContactPoint> new_samples;
    // TODO: this can be again formulated as a search problem
    int n_pts = this->n_pts;
    int n_located = fingertips.size();
    int n_unlocated = n_pts - n_located;
    int N = object_surface.size();

    std::vector<ContactPoint> remain_mnps;
    std::vector<int> relocate_idxes;

    for (int i = 0; i < std::min(n_on, n_unlocated); i++)
    {
        if (randd() > 0.5)
        {
            // randomly choose to not locate this unlocated finger
            continue;
        }
        int idx = randi(N);
        new_samples.push_back(object_surface[idx]);
        remain_mnps.push_back(object_surface[idx]);
    }

    for (int i = 0; i < n_on - n_unlocated; i++)
    {
        // randomly choose fingers to relocate

        int idx = randi(n_located);

        while (std::find(relocate_idxes.begin(), relocate_idxes.end(), idx) !=
               relocate_idxes.end())
        {
            idx = randi(n_located);
        }

        // randomly choose to release the finger
        if (randd() > 0.5)
        {
            relocate_idxes.push_back(idx);
        }
        else
        {
            new_samples.push_back(object_surface[randi(N)]);
            relocate_idxes.push_back(idx);
        }
    }
    for (int k = 0; k < n_located; k++)
    {
        // find remaining fingers
        if (std::find(relocate_idxes.begin(), relocate_idxes.end(), k) ==
            relocate_idxes.end())
        {
            remain_mnps.push_back(fingertips[k]);
            new_samples.push_back(fingertips[k]);
        }
    }

    // copy_points(new_samples, new_mnps);
    copy_points(remain_mnps, remain_fingertips);

    // new configuration
    new_config.resize(new_samples.size() * 6);
    for (int i = 0; i < new_samples.size(); i++)
    {
        new_config.segment(6 * i, 3) = new_samples[i].p;
        new_config.segment(6 * i + 3, 3) = new_samples[i].n;
    }

    new_samples.clear();
    return true;
}

void DartDeltaManipulator::Fingertips2PointContacts(
    const std::vector<ContactPoint> &fingertips,
    std::vector<ContactPoint> *point_contacts)
{

    if (is_patch_contact)
    {
        Matrix3d xs;
        xs << 1, 0, 0, -0.5, 0.865, 0, -0.5, -0.865, 0;

        double r = this->fingertip_radius;
        for (auto &pt : fingertips)
        {
            Matrix6d Adgco = contact_jacobian(pt.p, pt.n);
            Matrix3d Roc = Adgco.topLeftCorner(3, 3).transpose();
            for (int i = 0; i < 3; i++)
            {
                ContactPoint gpt;
                gpt.p = pt.p + r * Roc * (xs.row(i)).transpose();
                gpt.n = pt.n;
                gpt.d = 0;
                point_contacts->push_back(gpt);
            }
        }
    }
    else
    {
        copy_points(fingertips, point_contacts);
    }
}

void DartDeltaManipulator::setupCollisionGroup(WorldPtr world)
{
    auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();
    this->mCollisionGroup = collisionEngine->createCollisionGroup();
    for (int i = 0; i < this->n_pts; i++)
    {
        this->mCollisionGroup->addShapeFramesOf(this->bodies[i].get());
    }
}

bool DartDeltaManipulator::ifIKsolution(const VectorXd &mnp_config,
                                        const Vector7d &object_pose)
{

    Eigen::Matrix4d T;
    T = pose2SE3(object_pose);
    Eigen::Matrix3d R;
    R = T.block(0, 0, 3, 3);
    Eigen::Vector3d p;
    p = T.block(0, 3, 3, 1);

    // int n = int(mnp_config.size() / 6);

    // check if the fingertips are in the workspace
    for (int i = 0; i < this->n_pts; i++)
    {
        if (std::isnan(mnp_config[6 * i]))
        {
            continue;
        }
        Vector3d pos = R * Vector3d(mnp_config[6 * i], mnp_config[6 * i + 1], mnp_config[6 * i + 2]) + p;

        if (!is_in_cylindar(pos, this->robot_locations[i], this->workspace_radius, this->workspace_height))
        {
            return false;
        }
    }

    // check the distance between two fingers
    for (int i = 0; i < this->n_pts; i++){
        if (std::isnan(mnp_config[6 * i])){
            continue;
        } 
        Vector3d pos1 = Vector3d(mnp_config[6 * i], mnp_config[6 * i + 1], mnp_config[6 * i + 2]);
        for (int j = 0; j < this->n_pts; j++){
            if (i == j){
                continue;
            }
            if (std::isnan(mnp_config[6 * j])){
                continue;
            }
            Vector3d pos2 = Vector3d(mnp_config[6 * j], mnp_config[6 * j + 1], mnp_config[6 * j + 2]);
            // in cm
            if ((pos1.head(2) - pos2.head(2)).norm() < this->fingertip_radius*3){
                return false;
            }
        }
    }
    return true;
}

void DartDeltaManipulator::points_in_workspace(int finger_idx, std::vector<ContactPoint> object_surface_world, std::vector<int> *points_in_ws)
{

    for (int i = 0; i < object_surface_world.size(); i++)
    {
        Vector3d pos = object_surface_world[i].p;
        if (!is_in_cylindar(pos, this->robot_locations[finger_idx], this->workspace_radius, this->workspace_height))
        {
            continue;
        }
        else
        {
            points_in_ws->push_back(i);
        }
    }
    return;
}
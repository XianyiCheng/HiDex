#include "DartWholeHand.h"
#include "../contacts/contact_kinematics.h"
#include "wholehand/optimizer.h"

DartWholeHandManipulator::DartWholeHandManipulator(const std::string &manipulator_folder, double patch_contact_radius)
{
    this->patch_contact_radius = patch_contact_radius;

    // TODO: urdf name
    std::string urdf_file = manipulator_folder + "/hand.urdf";
    this->loadManipulator("hand", urdf_file);

    // load parts names
    std::cout << "Robot body nodes: " << std::endl;
    for (int i = 0; i < this->bodies[0]->getNumBodyNodes(); i++)
    {
        BodyNode *bn = this->bodies[0]->getBodyNode(i);
        std::string part_name = bn->getName();
        std::cout << part_name << ", ";
        this->part_names.push_back(part_name);
    }
    std::cout << std::endl;

    mJointLowerLimits.resize(this->NumDofs);
    mJointUpperLimits.resize(this->NumDofs);

    for (int i = 0; i < this->NumDofs; i++)
    {
        DegreeOfFreedom *dof = this->bodies[0]->getDof(i);
        mJointLowerLimits[i] = dof->getPositionLowerLimit();
        mJointUpperLimits[i] = dof->getPositionUpperLimit();
    }

    std::cout << "Joint lower limits: " << mJointLowerLimits.transpose() << std::endl;
    std::cout << "Joint upper limits: " << mJointUpperLimits.transpose() << std::endl;

    // TODO: load sampled vertices on each part
}

std::vector<std::string> DartWholeHandManipulator::getPartNames() const
{
    return this->part_names;
}

void DartWholeHandManipulator::Fingertips2PointContacts(const std::vector<ContactPoint> &fingertips, std::vector<ContactPoint> *point_contacts)
{
    // patch contact approximated by three points on a circle
    Matrix3d xs;
    xs << 1, 0, 0, -0.5, 0.865, 0, -0.5, -0.865, 0;

    double r = this->patch_contact_radius;
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

void DartWholeHandManipulator::setConfig(const VectorXd &config, const Vector7d &object_pose)
{
    this->bodies[0]->setPositions(config);
}

VectorXd DartWholeHandManipulator::random_sample_config(double unbounded_max, double unbounded_min) const
{
    VectorXd config(this->getNumDofs());
    for (int i = 0; i < this->getNumDofs(); i++)
    {
        double q_max;
        double q_min;
        if (std::isinf(this->mJointLowerLimits[i]) || std::isnan(this->mJointLowerLimits[i]))
        {
            q_min = unbounded_min;
        }
        else
        {
            q_min = this->mJointLowerLimits[i];
        }
        if (std::isinf(this->mJointUpperLimits[i]) || std::isnan(this->mJointUpperLimits[i]))
        {
            q_max = unbounded_max;
        }
        else
        {
            q_max = this->mJointUpperLimits[i];
        }
        config[i] = randd() * (q_max - q_min) + q_min;
    }
    return config;
}

void DartWholeHandManipulator::setupCollisionGroup(WorldPtr world)
{
    auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();
    this->mCollisionGroup = collisionEngine->createCollisionGroup();

    for (int i = 1; i < this->bodies[0]->getNumBodyNodes(); i++)
    {
        this->mCollisionGroup->addShapeFramesOf(
            this->bodies[0]->getBodyNode(i));
    }
}

std::vector<ContactPoint> DartWholeHandManipulator::get_points_in_world(const std::vector<std::string> &part_names, const std::vector<int> & part_point_idxes, const VectorXd &config){
    std::vector<ContactPoint> pts;

    this->bodies[0]->setPositions(config);

    for (int i = 0; i < part_names.size(); i++)
    {

        BodyNodePtr mbn = this->bodies[0]->getBodyNode(part_names[i]);
        Isometry3d mtf = mbn->getWorldTransform();
        ShapePtr ms = mbn->getShapeNodes().front()->getShape();
        MeshShape *meshShape =
            dynamic_cast<MeshShape *>(ms.get());

        Vector3d p;
        Vector3d n;

        {
            int index = part_point_idxes[i];
            aiVector3D point = meshShape->getMesh()->mMeshes[0]->mVertices[index];
            aiVector3D normal = meshShape->getMesh()->mMeshes[0]->mNormals[index];
            p << point[0], point[1], point[2];
            n << normal[0], normal[1], normal[2];
        }

        Vector3d p_w = mtf * p;
        Vector3d n_w = mtf.rotation() * n;
        pts.push_back(ContactPoint(p_w, n_w));
    }

    return pts;
}

bool DartWholeHandManipulator::roughIKsolutions(const std::vector<std::string> &part_names, const std::vector<int> &part_point_idxes, const std::vector<ContactPoint> &object_contacts, const Vector7d &object_pose, std::vector<VectorXd> *rough_ik_solutions)
{

    // TODO: need to add SDF in the optimization

    VectorXd initial_guess = (this->mJointLowerLimits + this->mJointUpperLimits) / 2.0;
    for (int i = 0; i < initial_guess.size(); i++)
    {
        if (std::isinf(initial_guess[i]) || std::isnan(initial_guess[i]))
        {
            initial_guess[i] = 0;
        }
    }

    // Initialize OptSetup
    std::shared_ptr<OptSetup> setup = std::make_shared<OptSetup>(this->bodies[0], part_names, part_point_idxes, object_contacts, object_pose, initial_guess);
    // Initialize Optimizer
    std::shared_ptr<Optimizer> optimizer = std::make_shared<Optimizer>(setup);

    // TODO: Sample different hand poses (initial_guess)
    // TODO: Sample part_point_idxes

    // For each hand pose, compute the solution
    
    // TODO: need to filter out optimal values that are too large.
    optimizer->solve();
    std::pair<double, VectorXd> solution = optimizer->getSolution();
    std::cout << "Value: " << solution.first << std::endl;
    std::cout << "Solution: " << solution.second.transpose() << std::endl;
    if (rough_ik_solutions != nullptr)
    {
        if (solution.second.norm() > 1e-6)
        {
            // TODO: change this temporary threshold!!!
            // if (solution.first < 0.1)
            // {
            rough_ik_solutions->push_back(solution.second);
            // }
        }
    }
}

void DartWholeHandManipulator::preprocess(const std::vector<std::string> &allowed_part_names, const std::vector<int> &allowed_part_point_idxes, int maximum_simultaneous_contact)
{
    this->allowed_part_names = allowed_part_names;
    this->allowed_part_point_idxes = allowed_part_point_idxes;

    if (maximum_simultaneous_contact > 0)
    {
        this->maximum_simultaneous_contact = maximum_simultaneous_contact;
    }
    else
    {
        this->maximum_simultaneous_contact = allowed_part_names.size();
    }

    if (this->allowed_part_points.size() > 0)
    {
        this->allowed_part_points.clear();
    }

    VectorXd initial_config = this->mJointLowerLimits;
    for (int i = 0; i < initial_config.size(); i++)
    {
        if (std::isinf(initial_config[i]) || std::isnan(initial_config[i]))
        {
            initial_config[i] = 0;
        }
    }

    this->allowed_part_points = this->get_points_in_world(allowed_part_names, allowed_part_point_idxes, initial_config);

    // for (int i = 0; i < this->allowed_part_points.size(); i++)
    // {
    //     for (int j = i + 1; j < this->allowed_part_points.size(); j++)
    //     {
    //         double d = (this->allowed_part_points[i].p - this->allowed_part_points[j].p).norm();
    //         std::cout << "Distance between " << allowed_part_names[i] << " and " << allowed_part_names[j] << " is " << d << std::endl;
    //     }
    // }
}

bool DartWholeHandManipulator::ifConsiderPartPairs(int i, int j, double contact_distance)
{

    if (i == j)
    {
        return false;
    }

    if (allowed_part_names[i].substr(allowed_part_names[i].size() - 3, 3) == "tip" && allowed_part_names[j].substr(allowed_part_names[j].size() - 3, 3) == "tip")
    {
        return true;
    }

    double d = (this->allowed_part_points[i].p - this->allowed_part_points[j].p).norm();
    double k = 1.5;

    if (d < contact_distance * k)
    {
        return false;
    }
    return true;
}
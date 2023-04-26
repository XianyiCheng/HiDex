#include "DartWholeHand.h"
#include "../contacts/contact_kinematics.h"

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

bool DartWholeHandManipulator::roughIKsolutions(const std::vector<std::string> &part_names, const std::vector<ContactPoint> &object_contacts, const Vector7d &object_pose, std::vector<VectorXd> *rough_ik_solutions)
{
    // TODO: implement this function
    if (rough_ik_solutions != nullptr)
    {
        rough_ik_solutions->clear();
    }

    // Solve the optimization problem
}

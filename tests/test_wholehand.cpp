#include <yaml-cpp/yaml.h>

#include "../src/mechanics/utilities/sample.h"
#include "../src/mechanics/utilities/utilities.h"
#include "../src/mechanics/contacts/contact_kinematics.h"

#include "../src/mechanics/manipulators/DartWholeHand.h"

#include "../src/mechanics/utilities/parser.hpp"
#include "../src/mechanics/worlds/DartWorld.h"

#include "../src/mechanics/utilities/io.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../src/mechanics/dart_utils/dart_utils.h"
#endif

// Tests
// Loading the allegro hand & visualize
// Visualize the rough IK solutions
// Visualize and print the sampled configs

int main(int argc, char *argv[])
{
    std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();
    SkeletonPtr object =
        createFreeBox("object", Vector3d(0.01, 0.01, 0.01),
                      Vector3d(0.7, 0.3, 0.3), 0.45);
    world->addObject(object);

    SkeletonPtr env_block =
        createFixedBox("env", Vector3d(0.1, 0.1, 0.1),
                       Vector3d(0, 0, -1000));
    world->addEnvironmentComponent(env_block);

    std::string folder_path = std::string(SRC_DIR) +"/data/wholehand/AllegroHand";

    std::shared_ptr<DartWholeHandManipulator> robot = std::make_shared<DartWholeHandManipulator>(folder_path, 0.5);

    world->addRobot(robot.get());

    std::cout << "Robot has " << robot->getNumDofs() << " dofs" << std::endl;

    std::vector<VectorXd> ik_solutions;
    VectorXd s1(robot->getNumDofs());
    s1.setZero();
    Vector7d x;
    x << 0, 0, 0, 0, 0, 0, 1;
    // robot->setConfig(s1, x);
    ik_solutions.push_back(s1);
    VectorXd s2 = s1;
    s2(0) = 0.5;
    s2(1) = 0.5;
    ik_solutions.push_back(s2);
    world->setRobotTrajectory(ik_solutions);
    world->startWindow(&argc, argv);

    // // --- Test rought IK ---
    // std::vector<std::string> part_names = {};
    // std::vector<ContactPoint> contact_points = {};
    // std::vector<VectorXd> ik_solutions = {};
    // Vector7d object_pose;
    // object_pose << 0, 0, 0, 0, 0, 0, 1;
    // robot->roughIKsolutions(part_names, contact_points, ik_solutions);
    // world->setRobotTrajectory(ik_solutions);
    // world->startWindow(&argc, argv);
}
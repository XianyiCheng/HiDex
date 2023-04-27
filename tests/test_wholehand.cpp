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
    double l = 0.1;
    SkeletonPtr object =
        createFreeBox("object", Vector3d(l, l, l),
                      Vector3d(0.7, 0.3, 0.3), 0.45);
    world->addObject(object);

    SkeletonPtr env_block =
        createFixedBox("env", Vector3d(0.1, 0.1, 0.1),
                       Vector3d(0, 0, -1000));
    world->addEnvironmentComponent(env_block);

    std::string folder_path = std::string(SRC_DIR) + "/data/wholehand/AllegroHand";

    std::shared_ptr<DartWholeHandManipulator> robot = std::make_shared<DartWholeHandManipulator>(folder_path, 0.5);

    world->addRobot(robot.get());

    std::cout << "Robot has " << robot->getNumDofs() << " dofs" << std::endl;

    // --- Test visualization ---
    // std::vector<VectorXd> ik_solutions;
    // VectorXd initial_guess(robot->getNumDofs());
    // initial_guess.setZero();
    // initial_guess(0) = 2;
    // ik_solutions.push_back(initial_guess);

    // Vector7d object_pose;
    // object_pose << 0, 0, 0, 0, 0, 0, 1;
    // std::vector<Vector7d> object_poses;
    // object_poses.push_back(object_pose);
    // world->setPlaybackTrajectory(object_poses, ik_solutions);

    // --- Test rough IK ---

    Vector7d object_pose;
    object_pose << 0, 0, 0, 0, 0, 0, 1;

    std::vector<ContactPoint> contact_points;
    contact_points.push_back(ContactPoint(Vector3d(0.0, 0.0, l / 2), Vector3d(0, 0, 1)));
    contact_points.push_back(ContactPoint(Vector3d(l / 2, 0.0, 0.0), Vector3d(1, 0, 0)));

    std::vector<std::string> all_parts = robot->getPartNames();

    std::vector<VectorXd> ik_solutions;

    for (int i = 0; i < all_parts.size(); i++)
    {
        for (int j = 0; j < all_parts.size(); j++)
        {
            if (i == j)
                continue;

            std::vector<std::string> part_names;
            part_names.push_back(all_parts[i]);
            part_names.push_back(all_parts[j]);
            robot->roughIKsolutions(part_names, contact_points, object_pose, &ik_solutions);
        }
        
    }

    std::vector<Vector7d> object_poses;
    for (int k = 0; k < ik_solutions.size(); k++)
    {
        object_poses.push_back(object_pose);
    }

    world->setPlaybackTrajectory(object_poses, ik_solutions);
    world->startWindow(&argc, argv);
}
#include <yaml-cpp/yaml.h>

#include "../src/mechanics/utilities/sample.h"
#include "../src/mechanics/utilities/utilities.h"
#include "../src/mechanics/contacts/contact_kinematics.h"

#include "../src/mechanics/manipulators/DartWholeHand.h"

#include "../src/mechanics/utilities/parser.hpp"
#ifndef _DART_WORLD
#define _DART_WORLD
#include "../src/mechanics/worlds/DartWorld.h"
#endif
#include "../src/mechanics/utilities/io.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../src/mechanics/dart_utils/dart_utils.h"
#endif

#include "../src/mechanics/manipulators/wholehand/optimizer.h"

// Tests
// Loading the allegro hand & visualize
// Visualize the rough IK solutions
// Visualize and print the sampled configs

int main(int argc, char *argv[])
{
    std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();
    double l = 0.14;
    std::string object_mesh_file = std::string(SRC_DIR) + "/data/cube_0.1.obj";

    Vector3d box_shape(l, l, l);
    SkeletonPtr object = createFreeObjectfromMesh("object", object_mesh_file);
    world->addObject(object);

    SkeletonPtr env_block =
        createFixedBox("env", Vector3d(0.1, 0.1, 0.1),
                       Vector3d(0, 0, -1000));
    world->addEnvironmentComponent(env_block);

    std::string folder_path = std::string(SRC_DIR) + "/data/wholehand/AllegroHand";

    std::shared_ptr<DartWholeHandManipulator> robot = std::make_shared<DartWholeHandManipulator>(folder_path, 0.01);

    world->addRobot(robot.get());

    std::cout << "Robot has " << robot->getNumDofs() << " dofs" << std::endl;

    // --- Test preprocess ---

    std::vector<std::string> allowed_parts = {"base_link", "link_1", "link_2", "link_3_tip", "link_5", "link_6", "link_7_tip", "link_9", "link_10", "link_11_tip", "link_14", "link_15", "link_15_tip"};
    std::vector<int> allowed_part_idxes = {8150, 2375, 4329, 2649, 2375, 4329, 2649, 2375, 4329, 2649, 2456, 3602, 2114};

    robot->preprocess(allowed_parts, allowed_part_idxes, 5);

    // --- Test signed distance field ---
    double d = robot->signedDistance("base_link", Vector3d(0.0, 0.0, 0.5));
    std::cout << "Signed distance to base_link " << d << std::endl;

    // // --- Test rough IK ---

    Vector7d object_pose;
    object_pose << 0, 0, 0, 0, 0, 0, 1;

    std::vector<ContactPoint> contact_points;
    contact_points.push_back(ContactPoint(Vector3d(0.8*-l / 2, 0, l / 2), Vector3d(0, 0, -1)));
    contact_points.push_back(ContactPoint(Vector3d(l / 2, 0, 0.8*-l / 2), Vector3d(-1, 0, 0)));

    double d_contact = (contact_points[0].p - contact_points[1].p).norm();

    std::cout << "Distance between two contact points " << d_contact << std::endl;

    std::vector<VectorXd> ik_solutions;
    std::vector<std::string> texts;

    std::vector<std::string> part_names = {"base_link", "link_7_tip"};
    std::vector<int> part_p_idxes = {8150, 2649};

    int n_sdf_sample = 50;

    {
        robot->roughIKsolutions(part_names, part_p_idxes, contact_points, object_pose, VectorXd::Zero(0), &ik_solutions);
        double avg_d = robot->averagePenetrateDistance(ik_solutions.back(), object_pose, box_shape, n_sdf_sample);
        double max_d = robot->maxPenetrationDistance(ik_solutions.back(), object_pose, box_shape, n_sdf_sample);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4) << "Penetration distance, average " << avg_d << ", max " << max_d;
        texts.push_back(oss.str());
    }

    {
        // VectorXd initial_robot_guess = ik_solutions.back(); // should not use previous solutions as initial guess, will stuck in the same local minima
        VectorXd initial_robot_guess = robot->getMiddleJointAngles();
        initial_robot_guess.head(6) = ik_solutions.back().head(6);

        std::shared_ptr<ObjectSurfaceOptSetup> setup = std::make_shared<ObjectSurfaceOptSetup>(robot->bodies[0], part_names, part_p_idxes, contact_points, object_pose, initial_robot_guess, object_mesh_file);

        std::shared_ptr<ObjectSurfaceOptimizer> optimizer = std::make_shared<ObjectSurfaceOptimizer>(setup);

        optimizer->solve();
        std::pair<double, VectorXd> solution = optimizer->getSolution();
        std::cout << "Value: " << solution.first << std::endl;
        std::cout << "Solution: " << solution.second.transpose() << std::endl;

        ik_solutions.push_back(solution.second.head(robot->getNumDofs()));

        double avg_d = robot->averagePenetrateDistance(ik_solutions.back(), object_pose, box_shape, n_sdf_sample);
        double max_d = robot->maxPenetrationDistance(ik_solutions.back(), object_pose, box_shape, n_sdf_sample);
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4) << "+ObjectSurface Opt: Penetration distance, average " << avg_d << ", max " << max_d;
        texts.push_back(oss.str());

        std::vector<ContactPoint> updatad_contacts_world = setup->getUpdatedObjectContactPointsWorld(solution.second.tail(contact_points.size() * 2));
        std::cout << "Updated contact point in world frame " << std::endl;
        for (auto cp : updatad_contacts_world)
        {
            std::cout << cp.p.transpose() << " " << cp.n.transpose() << std::endl;
        }
    }

    std::vector<Vector7d> object_poses;
    for (int k = 0; k < ik_solutions.size(); k++)
    {
        object_poses.push_back(object_pose);
    }

    world->setPlaybackTrajectoryWithText(object_poses, ik_solutions, texts);
    world->startWindow(&argc, argv);
}
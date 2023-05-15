
#include "../src/tasks/wholehand_setup.h"

// #include "../src/tasks/visualization.h"

int main(int argc, char *argv[])
{

    std::shared_ptr<WholeHandTASK> task = std::make_shared<WholeHandTASK>();

    std::string config_file;

    if (argc > 1)
    {
        config_file = argv[1];
    }
    else
    {
        // config_file = "/home/xianyi/Research/MCTS/general_planner/setup_template.yaml";
        config_file = "/home/xianyi/Research/MCTS/data/wholehand/setup.yaml";
    }

    // should use arg to specify the path to the setup.yaml file

    YAML::Node config = YAML::LoadFile(config_file);

    std::string visualize_option = config["visualize_option"].as<std::string>();
    std::string output_file = config["save_file_path"].as<std::string>();

    load_task(task, config);
    load_start_and_goal_poses(task, config);
    load_reward_functions(task, config);

    task->initialize();

    std::vector<double> box_l =
        config["box_object"]["shape"].as<std::vector<double>>();
    Vector3d box_shape(box_l[0], box_l[1], box_l[2]);

    // // ---- Test rough IK SDF ----
    int n_sample = 2;
    {
        Vector7d object_pose = task->start_object_pose;
        std::vector<ContactPoint> repell_contact_points;
        repell_contact_points = task->object_surface_pts;
        for (int k = 0; k < repell_contact_points.size() - 10; k++)
        {
            repell_contact_points.erase(repell_contact_points.begin() + randi(repell_contact_points.size()));
        }

        std::vector<ContactPoint> contact_points;
        double l = 0.1;
        contact_points.push_back(ContactPoint(Vector3d(0.0, 0.0, l / 2), Vector3d(0, 0, -1)));
        contact_points.push_back(ContactPoint(Vector3d(l / 2, 0.0, 0.0), Vector3d(-1, 0, 0)));

        double d_contact = (contact_points[0].p - contact_points[1].p).norm();

        std::cout << "Distance between two contact points " << d_contact << std::endl;

        std::vector<VectorXd> sdf_ik_solutions;

        std::vector<std::string> allowed_parts = task->getRobot()->allowed_part_names;
        std::vector<int> allowed_part_idxes = task->getRobot()->allowed_part_point_idxes;

        {
            std::vector<std::string> part_names = {"base_link", "link_7_tip"};
            std::vector<int> part_p_idxes = {8150, 2649};
            task->getRobot()->roughSDFIKsolutions(part_names, part_p_idxes, contact_points, object_pose, repell_contact_points, box_shape, n_sample, &sdf_ik_solutions);
            // task->getRobot()->roughIKsolutions(part_names, part_p_idxes, contact_points, object_pose, &sdf_ik_solutions);
            double penetrate_d = task->getRobot()->maxPenetrationDistance(sdf_ik_solutions.back(), object_pose, box_shape);
            double average_signed_d = task->getRobot()->averagePenetrateDistance(sdf_ik_solutions.back(), object_pose, box_shape);
            std::cout << "max penetration distance: " << penetrate_d << ", average signed distance: " << average_signed_d << std::endl;
        }
        {
            std::vector<std::string> part_names = {"base_link", "link_3_tip"};
            std::vector<int> part_p_idxes = {8150, 2649};
            task->getRobot()->roughSDFIKsolutions(part_names, part_p_idxes, contact_points, object_pose, repell_contact_points, box_shape, n_sample, &sdf_ik_solutions);
            // task->getRobot()->roughIKsolutions(part_names, part_p_idxes, contact_points, object_pose, &sdf_ik_solutions);
            double penetrate_d = task->getRobot()->maxPenetrationDistance(sdf_ik_solutions.back(), object_pose, box_shape);
            double average_signed_d = task->getRobot()->averagePenetrateDistance(sdf_ik_solutions.back(), object_pose, box_shape);
            std::cout << "max penetration distance: " << penetrate_d << ", average signed distance: " << average_signed_d << std::endl;
        }
        {
            std::vector<std::string> part_names = {"link_15_tip", "link_3_tip"};
            std::vector<int> part_p_idxes = {2114, 2649};
            task->getRobot()->roughSDFIKsolutions(part_names, part_p_idxes, contact_points, object_pose, repell_contact_points,box_shape, n_sample, &sdf_ik_solutions);
            // task->getRobot()->roughIKsolutions(part_names, part_p_idxes, contact_points, object_pose, &sdf_ik_solutions);
            double penetrate_d = task->getRobot()->maxPenetrationDistance(sdf_ik_solutions.back(), object_pose, box_shape);
            double average_signed_d = task->getRobot()->averagePenetrateDistance(sdf_ik_solutions.back(), object_pose, box_shape);
            std::cout << "max penetration distance: " << penetrate_d << ", average signed distance: " << average_signed_d << std::endl;
        }

        // for (int i = 0; i < allowed_parts.size(); i++)
        // {
        //     for (int j = 0; j < allowed_parts.size(); j++)
        //     {
        //         if (task->getRobot()->ifConsiderPartPairs(i, j, d_contact) == false)
        //         {
        //             continue;
        //         }

        //         std::vector<std::string> part_names;
        //         part_names.push_back(allowed_parts[i]);
        //         part_names.push_back(allowed_parts[j]);
        //         std::vector<int> part_p_idxes;
        //         part_p_idxes.push_back(allowed_part_idxes[i]);
        //         part_p_idxes.push_back(allowed_part_idxes[j]);
        //         std::cout << "Solving SDF for part_names: " << part_names[0] << " " << part_names[1] << std::endl;
        //         task->getRobot()->roughSDFIKsolutions(part_names, part_p_idxes, contact_points, object_pose, repell_contact_points, &sdf_ik_solutions);
        //     }
        // }

        std::vector<Vector7d> object_poses;
        for (int k = 0; k < sdf_ik_solutions.size(); k++)
        {
            object_poses.push_back(object_pose);
        }

        // for (int i = 0; i < sdf_ik_solutions.size(); i++)
        // {
        //     double penetrate_d = task->getRobot()->maxPenetrationDistance(sdf_ik_solutions[i], object_poses[i], box_shape);
        //     std::cout << "max penetration distance: " << penetrate_d << std::endl;
        // }

        task->m_world->setPlaybackTrajectory(object_poses, sdf_ik_solutions);
        task->m_world->startWindow(&argc, argv);
    }
}
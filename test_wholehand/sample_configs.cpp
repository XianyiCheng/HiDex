
#include "../src/tasks/wholehand_setup.h"

// #include "../src/tasks/visualization.h"

int main(int argc, char *argv[])
{

    std::shared_ptr<WholeHandTASK> task = std::make_shared<WholeHandTASK>();

    std::string task_folder;

    if (argc > 1)
    {
        task_folder = argv[1];
    }
    else
    {
        task_folder = path_join(std::string(SRC_DIR), "/data/wholehand");
    }

    std::string config_file = path_join(task_folder, "setup.yaml");

    // should use arg to specify the path to the setup.yaml file

    YAML::Node config = YAML::LoadFile(config_file);

    std::string visualize_option = config["visualize_option"].as<std::string>();
    std::string output_file = path_join(task_folder, "output.csv");

    load_task(task, config, task_folder);
    load_start_and_goal_poses(task, config);
    load_reward_functions(task, config);

    task->initialize();

    // std::vector<double> box_l =
    //     config["box_object"]["shape"].as<std::vector<double>>();
    // Vector3d box_shape(box_l[0], box_l[1], box_l[2]);

    // std::srand(12658645);
    // set random seed with time
    std::srand(std::time(nullptr));

    // --- Test rough collision check ---
    std::vector<ContactPoint> collision_fingertips;
    collision_fingertips.push_back(ContactPoint(Vector3d(0, 0, 0), Vector3d(0, 0, 1)));

    std::cout << "Rough collision check " << task->rough_collision_check(collision_fingertips, task->start_object_pose) << std::endl;

    // --- Test sample likely contact configs ---
    {
        std::vector<ContactPoint> envs;
        task->m_world->getObjectContacts(&envs, task->start_object_pose);
        VectorXi mode(envs.size());
        mode.setZero();
        Vector6d v;
        v << 1, 0, 0, 0, 0, 0;
        std::vector<WholeHandTASK::ContactConfig> sampled_actions;
        std::vector<double> probs;

        int n_max_sample = 200;
        task->sample_likely_contact_configs(task->start_object_pose, mode, v, envs, n_max_sample, &sampled_actions, &probs);

        std::cout << "sampled_actions.size(): " << sampled_actions.size() << std::endl;

        bool if_sdf = true;
        int n_sdf_sample = 50;

        std::vector<VectorXd> ik_solutions;
        std::vector<VectorXd> ik_solutions_raw;
        std::vector<std::string> texts;
        std::vector<WholeHandTASK::ContactConfig> contact_configs;
        for (auto contact_config : sampled_actions)
        {
            std::cout << "contact_config: " << contact_config << std::endl;
            bool if_ik = task->rough_ik_check(contact_config, task->start_object_pose, &ik_solutions_raw);

            if (if_ik)
            {
                // double max_d = task->getRobot()->maxPenetrationDistance(ik_solutions_raw.back(), task->start_object_pose, box_shape, n_sdf_sample);
                // double avg_d = task->getRobot()->averagePenetrateDistance(ik_solutions_raw.back(), task->start_object_pose, box_shape, n_sdf_sample);

                // if ((max_d < 0.03) && (avg_d < 0.01)){
                contact_configs.push_back(contact_config);
                ik_solutions.push_back(ik_solutions_raw.back());

                // std::ostringstream oss;
                // oss << std::fixed << std::setprecision(4) << "Penetration distance, average " << avg_d << ", max " << max_d;
                // texts.push_back(oss.str());
                // break;
                // }
            }
            else
            {
                std::cout << "ik failed" << std::endl;
            }

            // add box surface opt

            // bool if_ik_box_opt = task->rough_ik_box_opt(contact_config, task->start_object_pose, box_shape, &ik_solutions_raw);
            // if (if_ik_box_opt)
            // {
            //     double max_d = task->getRobot()->maxPenetrationDistance(ik_solutions_raw.back(), task->start_object_pose, box_shape, n_sdf_sample);
            //     double avg_d = task->getRobot()->averagePenetrateDistance(ik_solutions_raw.back(), task->start_object_pose, box_shape, n_sdf_sample);

            //     contact_configs.push_back(contact_config);
            //     ik_solutions.push_back(ik_solutions_raw.back());

            //     std::ostringstream oss;
            //     oss << std::fixed << std::setprecision(4) << "+BoxSurface Opt. Penetration distance, average " << avg_d << ", max " << max_d;
            //     texts.push_back(oss.str());
            // }
        }

        std::vector<Vector7d> object_poses;
        for (int k = 0; k < ik_solutions.size(); k++)
        {
            object_poses.push_back(task->start_object_pose);
        }

        std::vector<VectorXd> ik_and_sphere_solutions;
        for (int k = 0; k < ik_solutions.size(); k++)
        {

            VectorXd ik_and_sphere_solution(ik_solutions[k].size() + 3 * contact_configs[k].contact_idxes.size());
            ik_and_sphere_solution.head(ik_solutions[k].size()) = ik_solutions[k];
            for (int kk = 0; kk < contact_configs[k].contact_idxes.size(); kk++)
            {
                ik_and_sphere_solution.segment(ik_solutions[k].size() + 3 * kk, 3) = task->object_surface_pts[contact_configs[k].contact_idxes[kk]].p;
            }
            ik_and_sphere_solutions.push_back(ik_and_sphere_solution);
        }

        // // Only show the hand
        // task->m_world->setPlaybackTrajectory(object_poses, ik_solutions);

        // // Show the hand and the collision fingertip spheres
        task->m_world->setPlaybackTrajectory(object_poses, ik_and_sphere_solutions);

        // Show the hand and the collision fingertip spheres and texts
        // task->m_world->setPlaybackTrajectoryWithText(object_poses, ik_and_sphere_solutions, texts);

        task->m_world->startWindow(&argc, argv);
    }
}
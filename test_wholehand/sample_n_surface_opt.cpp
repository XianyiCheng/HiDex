
#include "../src/tasks/wholehand_setup.h"
#include "../src/mechanics/manipulators/wholehand/optimizer.h"

// #include "../src/tasks/visualization.h"

std::vector<int> get_part_idxes(const std::vector<std::string> &allowed_parts, const std::vector<int> &allowed_part_idxes, const std::vector<std::string> &part_names)
{
    std::vector<int> part_p_idxes;
    for (auto seg : part_names)
    {
        for (int k = 0; k < allowed_parts.size(); k++)
        {
            if (seg == allowed_parts[k])
            {
                part_p_idxes.push_back(allowed_part_idxes[k]);
                break;
            }
        }
    }
    return part_p_idxes;
}

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
        // config_file = "/home/xianyi/Research/MCTS/general_planner/setup_template.yaml";
        task_folder = path_join(std::string(SRC_DIR), "/data/wholehand");
    }

    // should use arg to specify the path to the setup.yaml file
    std::string config_file = path_join(task_folder, "setup.yaml");

    YAML::Node config = YAML::LoadFile(config_file);

    std::string object_mesh_file;
    if (config["mesh_object"]["mesh_file"])
    {
        object_mesh_file = path_join(std::string(SRC_DIR), config["mesh_object"]["mesh_file"].as<std::string>());
    }
    else
    {
        std::cout << "No mesh file specified for the object" << std::endl;
        return 0;
    }

    std::string visualize_option = config["visualize_option"].as<std::string>();
    std::string output_file = path_join(task_folder, "output.csv");
    
    load_task(task, config);
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

        int n_max_sample = 100;
        task->sample_likely_contact_configs(task->start_object_pose, mode, v, envs, n_max_sample, &sampled_actions, &probs);

        std::cout << "sampled_actions.size(): " << sampled_actions.size() << std::endl;

        bool if_sdf = true;
        int n_sdf_sample = 50;

        std::vector<VectorXd> ik_solutions;
        std::vector<VectorXd> ik_and_sphere_solutions;
        std::vector<VectorXd> ik_solutions_raw;
        std::vector<std::string> texts;
        std::vector<WholeHandTASK::ContactConfig> contact_configs;
        int n_solution = task->getRobot()->getNumDofs();
        for (int k_contact_config=0; k_contact_config<sampled_actions.size(); k_contact_config++)
        {
            WholeHandTASK::ContactConfig contact_config = sampled_actions[k_contact_config]; 
            std::cout << "contact_config: " << contact_config << std::endl;
            bool if_ik = task->rough_ik_check(contact_config, task->start_object_pose, &ik_solutions_raw);

            if (if_ik)
            {
                // double max_d = task->getRobot()->maxPenetrationDistance(ik_solutions_raw.back(), task->start_object_pose, box_shape, n_sdf_sample);
                // double avg_d = task->getRobot()->averagePenetrateDistance(ik_solutions_raw.back(), task->start_object_pose, box_shape, n_sdf_sample);

                // if ((max_d < 0.03) && (avg_d < 0.01)){
                contact_configs.push_back(contact_config);
                ik_solutions.push_back(ik_solutions_raw.back());

                texts.push_back(std::to_string(k_contact_config) + " IK");

                // show contact points
                {
                    VectorXd ik_and_sphere_solution(n_solution + 3 * contact_config.contact_idxes.size());
                    ik_and_sphere_solution.head(n_solution) = ik_solutions.back();
                    for (int kk = 0; kk < contact_config.contact_idxes.size(); kk++)
                    {
                        ik_and_sphere_solution.segment(n_solution + 3 * kk, 3) = task->object_surface_pts[contact_config.contact_idxes[kk]].p;
                    }
                    ik_and_sphere_solutions.push_back(ik_and_sphere_solution);
                }

                {
                    // VectorXd initial_robot_guess = ik_solutions.back(); // should not use previous solutions as initial guess, will stuck in the same local minima
                    VectorXd initial_robot_guess = task->getRobot()->getMiddleJointAngles();
                    initial_robot_guess.head(6) = ik_solutions.back().head(6);
                    std::vector<ContactPoint> object_contact_points = retrieve_elements<ContactPoint>(task->object_surface_pts, contact_config.contact_idxes);

                    std::shared_ptr<ObjectSurfaceOptSetup> setup = std::make_shared<ObjectSurfaceOptSetup>(task->getRobot()->bodies[0], contact_config.hand_segments,
                                                                                                           get_part_idxes(task->getRobot()->allowed_part_names, task->getRobot()->allowed_part_point_idxes, contact_config.hand_segments),
                                                                                                           object_contact_points, task->start_object_pose, initial_robot_guess, object_mesh_file);

                    std::shared_ptr<ObjectSurfaceOptimizer> optimizer = std::make_shared<ObjectSurfaceOptimizer>(setup);

                    optimizer->solve();
                    std::pair<double, VectorXd> solution = optimizer->getSolution();
                    std::cout << "Value: " << solution.first << std::endl;
                    std::cout << "Solution: " << solution.second.transpose() << std::endl;

                    ik_solutions.push_back(solution.second.head(task->getRobot()->getNumDofs()));

                    // double avg_d = task->getRobot()->averagePenetrateDistance(ik_solutions.back(), object_pose, box_shape, n_sdf_sample);
                    // double max_d = task->getRobot()->maxPenetrationDistance(ik_solutions.back(), object_pose, box_shape, n_sdf_sample);
                    // std::ostringstream oss;
                    // oss << std::fixed << std::setprecision(4) << "+ObjectSurface Opt: Penetration distance, average " << avg_d << ", max " << max_d;
                    // texts.push_back(oss.str());
                    texts.push_back(std::to_string(k_contact_config) + " IK +ObjectSurface Opt");

                    std::vector<ContactPoint> updated_contacts = setup->getUpdatedObjectContactPointsLocal(solution.second.tail(object_contact_points.size() * 2));

                    // show contact points
                    VectorXd ik_and_sphere_solution(n_solution + 3 * updated_contacts.size());
                    ik_and_sphere_solution.head(n_solution) = ik_solutions.back();
                    for (int kk = 0; kk < updated_contacts.size(); kk++)
                    {
                        ik_and_sphere_solution.segment(n_solution + 3 * kk, 3) = updated_contacts[kk].p;
                    }
                    ik_and_sphere_solutions.push_back(ik_and_sphere_solution);
                    // std::cout << "Updated contact point in world frame " << std::endl;
                    // for (auto cp : updatad_contacts_world)
                    // {
                    //     std::cout << cp.p.transpose() << " " << cp.n.transpose() << std::endl;
                    // }
                }
            }
            else
            {
                std::cout << "ik failed" << std::endl;
            }
        }

        std::vector<Vector7d> object_poses;
        for (int k = 0; k < ik_solutions.size(); k++)
        {
            object_poses.push_back(task->start_object_pose);
        }


        // // Only show the hand
        // task->m_world->setPlaybackTrajectory(object_poses, ik_solutions);

        // // Show the hand and the collision fingertip spheres
        // task->m_world->setPlaybackTrajectory(object_poses, ik_and_sphere_solutions);

        // Show the hand and the collision fingertip spheres and texts
        task->m_world->setPlaybackTrajectoryWithText(object_poses, ik_and_sphere_solutions, texts);
        task->m_world->setWindowFrameRate(800);
        task->m_world->startWindow(&argc, argv);
        
    }
}
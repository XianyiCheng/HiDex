
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

    // if (visualize_option == "csv")
    // {
    //     visualize_output_file_object_centric(task->m_world, output_file);
    //     task->m_world->startWindow(&argc, argv);
    //     return 0;
    // }

    // if (visualize_option == "setup")
    // {
    //     VisualizeSG(task->m_world, task->start_object_pose, task->goal_object_pose);
    //     task->m_world->startWindow(&argc, argv);
    //     return 0;
    // }

    task->initialize();

    // --- Test rough collision check ---
    std::vector<ContactPoint> collision_fingertips;
    collision_fingertips.push_back(ContactPoint(Vector3d(0,0,0), Vector3d(0,0,1)));

    std::cout << "Rough collision check " << task->rough_collision_check(collision_fingertips, task->start_object_pose) << std::endl;

    // --- Test sample likely contact configs ---
    std::vector<ContactPoint> envs;
    task->m_world->getObjectContacts(&envs,task->start_object_pose);
    VectorXi mode(envs.size());
    mode.setZero();
    Vector6d v;
    v << 1,0,0,0,0,0;
    std::vector<WholeHandTASK::ContactConfig> sampled_actions;
    std::vector<double> probs;
    task->sample_likely_contact_configs(task->start_object_pose, mode, v, envs, 200, &sampled_actions, &probs);
    
    std::cout << "sampled_actions.size(): " << sampled_actions.size() << std::endl;

    std::vector<VectorXd> ik_solutions;
    std::vector<WholeHandTASK::ContactConfig> contact_configs;
    for (auto contact_config: sampled_actions){
        bool if_ik = task->rough_ik_check(contact_config, task->start_object_pose, &ik_solutions);
        contact_configs.push_back(contact_config);
    }

    std::vector<Vector7d> object_poses;
    for (int k = 0; k < ik_solutions.size(); k++)
    {
        object_poses.push_back(task->start_object_pose);
    }

    for (auto contact_config: contact_configs){
        std::cout << "contact_config: " << contact_config << std::endl;
    }

    std::vector<VectorXd> ik_and_sphere_solutions;
    for (int k = 0; k < ik_solutions.size(); k++)
    {

        VectorXd ik_and_sphere_solution(ik_solutions[k].size() + 3*contact_configs[k].contact_idxes.size());
        ik_and_sphere_solution.head(ik_solutions[k].size()) = ik_solutions[k];
        for (int kk = 0; kk < contact_configs[k].contact_idxes.size(); kk++)
        {
            ik_and_sphere_solution.segment(ik_solutions[k].size() + 3*kk, 3) = task->object_surface_pts[contact_configs[k].contact_idxes[kk]].p;
        }
        ik_and_sphere_solutions.push_back(ik_and_sphere_solution);
    }

    // only show the hand
    // task->m_world->setPlaybackTrajectory(object_poses, ik_solutions);
    // show the hand and spheres for object contact points
    task->m_world->setPlaybackTrajectory(object_poses, ik_and_sphere_solutions);
    task->m_world->startWindow(&argc, argv);

}
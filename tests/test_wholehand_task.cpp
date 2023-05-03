
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

    // --- Test sample likely contact configs ---
    std::vector<ContactPoint> envs;
    task->m_world->getObjectContacts(&envs,task->start_object_pose);
    VectorXi mode(envs.size());
    mode.setZero();
    Vector6d v;
    v << 1,0,0,0,0,0;
    std::vector<WholeHandTASK::ContactConfig> sampled_actions;
    std::vector<double> probs;
    task->sample_likely_contact_configs(task->start_object_pose, mode, v, envs, 100, &sampled_actions, &probs);
    
    std::cout << "sampled_actions.size(): " << sampled_actions.size() << std::endl;

    std::vector<VectorXd> ik_solutions;
    for (auto contact_config: sampled_actions){
        task->rough_ik_check(contact_config, task->start_object_pose, &ik_solutions);
    }

    std::vector<Vector7d> object_poses;
    for (int k = 0; k < ik_solutions.size(); k++)
    {
        object_poses.push_back(task->start_object_pose);
    }

    for (auto contact_config: sampled_actions){
        std::cout << "contact_config: " << contact_config << std::endl;
    }

    task->m_world->setPlaybackTrajectory(object_poses, ik_solutions);
    task->m_world->startWindow(&argc, argv);


//     int random_seed = config["random_seed"].as<int>();
//     if (random_seed >= 0)
//     {
//         std::srand(random_seed);
//     }
//     else
//     {
//         std::srand(std::time(nullptr));
//     }

//     HMP::HierarchicalComputeOptions compute_options;
//     load_mcts_options(compute_options, config);

//     HMP::Level1MCTS<TASK::State, TASK::State2, TASK> tree(
//         task, task->get_start_state(), compute_options);

//     HMP::Node<TASK::State> *current_node = tree.search_tree();

//     std::vector<TASK::State> object_trajectory;
//     std::vector<TASK::State2> action_trajectory;

//     tree.get_final_results(current_node, &object_trajectory, &action_trajectory);
    
//     VectorXd result = get_results(&tree, task, object_trajectory, action_trajectory,
//                 current_node->m_value);
    
//     if (visualize_option == "show")
//     {
//         VisualizeStateTraj(task->m_world, task, object_trajectory, action_trajectory);
//         task->m_world->startWindow(&argc, argv);
//     }

//     if ((visualize_option == "save") || (visualize_option == "save_n_show"))
//     {
//         std::remove(output_file.c_str());
//         MatrixXd output_mat = get_output_object_centric(object_trajectory, action_trajectory, task);
//         saveData(output_file, output_mat);
//     }

//     if (visualize_option == "save_n_show")
//     {
//         visualize_output_file_object_centric(task->m_world, output_file);
//         task->m_world->startWindow(&argc, argv);
//     }
}
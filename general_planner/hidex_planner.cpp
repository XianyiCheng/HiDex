#include "../src/tasks/setup.h"

#include "../src/tasks/visualization.h"

// const TASK::State2::Action TASK::State2::no_action = TASK::State2::Action(-1, -1);
// const TASK::State::Action TASK::State::no_action = -1;

int main(int argc, char *argv[])
{

    std::shared_ptr<TASK> task = std::make_shared<TASK>();

    std::string config_file;

    if (argc > 1)
    {
        config_file = std::string(SRC_DIR) + argv[1];
    }
    else
    {
        // config_file = "/home/xianyi/Research/MCTS/general_planner/setup_template.yaml";
        config_file = std::string(SRC_DIR) + "/data/pushing/setup.yaml";
    }

    // should use arg to specify the path to the setup.yaml file

    YAML::Node config = YAML::LoadFile(config_file);

    std::string visualize_option = config["visualize_option"].as<std::string>();
    std::string output_file = std::string(SRC_DIR) + config["save_file_path"].as<std::string>();

    load_task(task, config);
    load_start_and_goal_poses(task, config);
    load_reward_functions(task, config);

    if (visualize_option == "csv")
    {
        visualize_output_file_object_centric(task->m_world, output_file);
        task->m_world->startWindow(&argc, argv);
        return 0;
    }

    if (visualize_option == "setup")
    {
        VisualizeSG(task->m_world, task->start_object_pose, task->goal_object_pose);
        task->m_world->startWindow(&argc, argv);
        return 0;
    }

    task->initialize();

    int random_seed = config["random_seed"].as<int>();
    if (random_seed >= 0)
    {
        std::srand(random_seed);
    }
    else
    {
        std::srand(std::time(nullptr));
    }

    HMP::HierarchicalComputeOptions compute_options;
    load_mcts_options(compute_options, config);

    HMP::Level1MCTS<TASK::State, TASK::State2, TASK> tree(
        task, task->get_start_state(), compute_options);

    HMP::Node<TASK::State> *current_node = tree.search_tree();

    std::vector<TASK::State> object_trajectory;
    std::vector<TASK::State2> action_trajectory;

    tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

    VectorXd result = get_results(&tree, task, object_trajectory, action_trajectory,
                                  current_node->m_value);

    if (visualize_option == "show")
    {
        VisualizeStateTraj(task->m_world, task, object_trajectory, action_trajectory);
        task->m_world->startWindow(&argc, argv);
    }

    if ((visualize_option == "save") || (visualize_option == "save_n_show"))
    {
        std::remove(output_file.c_str());
        MatrixXd output_mat = get_output_object_centric(object_trajectory, action_trajectory, task);
        saveData(output_file, output_mat);
        // save_full_output_object_centric(object_trajectory, action_trajectory, task, output_file);
    }

    if (visualize_option == "save_n_show")
    {
        visualize_output_file_object_centric(task->m_world, output_file);
        // visualize_full_output_file_object_centric(task->m_world, output_file);
        task->m_world->startWindow(&argc, argv);
    }
}
#include "../src/tasks/setup.h"
#include "../src/tasks/visualization.h"

int main(int argc, char *argv[])
{

    std::shared_ptr<TASK> task = std::make_shared<TASK>();

    std::string task_folder;
    std::string traj_file;

    if (argc > 2)
    {
        task_folder = argv[1];
        traj_file = argv[2];
    }
    else
    {
        std::cout << "Please specify the absolute path to the setup.yaml file and the path to the trajectory file." << std::endl;
        return 0;
    }

    std::string config_file = path_join(task_folder, "setup.yaml");

    YAML::Node config = YAML::LoadFile(config_file);

    load_task(task, config, task_folder);
    load_start_and_goal_poses(task, config);
    load_reward_functions(task, config);

    visualize_full_output_file_object_centric(task->m_world, traj_file);
    task->m_world->startWindow(&argc, argv);
    return 0;
}
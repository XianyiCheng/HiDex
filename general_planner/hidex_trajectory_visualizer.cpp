#include "../src/tasks/setup.h"
#include "../src/tasks/visualization.h"

int main(int argc, char *argv[])
{

    std::shared_ptr<TASK> task = std::make_shared<TASK>();

    std::string config_file;
    std::string traj_file;

    if (argc > 2)
    {
        config_file = argv[1];
        traj_file = argv[2];
    }
    else
    {
        std::cout << "Please specify the path to the setup.yaml file and the path to the trajectory file." << std::endl;
        return 0;
    }

    YAML::Node config = YAML::LoadFile(config_file);

    load_task(task, config);
    load_start_and_goal_poses(task, config);
    load_reward_functions(task, config);

    visualize_full_output_file_object_centric(task->m_world, traj_file);
    task->m_world->startWindow(&argc, argv);
    return 0;
}
#include "../src/tasks/setup.h"

#include "../src/tasks/visualization.h"

int main(int argc, char *argv[])
{

    std::shared_ptr<TASK> task = std::make_shared<TASK>();

    std::string task_folder = path_join(std::string(SRC_DIR), "/data/reorient_quasidynamic");

    std::string config_file = path_join(task_folder, "setup.yaml");

    YAML::Node config = YAML::LoadFile(config_file);

    load_task(task, config, task_folder);
    load_start_and_goal_poses(task, config);
    load_reward_functions(task, config);

    task->initialize();

    Vector7d x_start;
    // x_start << 0.7, 0, 0.5, 0, 0, 0, 1;
    x_start << 0, 0, 0.5, 0, 0, 0, 1;
    // x_start <<  0.184196 ,  6.5697e-20   ,  0.603777, 5.43097e-19  ,   0.119737, -1.26109e-19  ,  0.992806;
    Vector7d x_goal;
    x_goal << 1, 0, 0.5, 0, 0.7071, 0, 0.7071;

    std::vector<ContactPoint> envs;
    envs.push_back(
        ContactPoint(Vector3d(0.5, 0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
    envs.push_back(
        ContactPoint(Vector3d(0.5, -0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
    envs.push_back(
        ContactPoint(Vector3d(-0.5, -0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));
    envs.push_back(
        ContactPoint(Vector3d(-0.5, 0.5, -0.5), Vector3d(0, 0, 1), 9.99868e-05));

    VectorXd manipulator_config(6);
    VectorXi env_mode(4);

    // // pushing
    // manipulator_config << -0.5, 0, 0, 1, 0, 0;
    // env_mode << 0, 0, 0, 0;

    // pivoting from side contact
    // manipulator_config << -0.5, 0.0489681, 0.45935144, 1, -0, -0;
    // env_mode << 0, 0, 1, 1;

    // pivoting from top contact
    manipulator_config << 0.5, 0, 0.5, 0, 0, -1;
    env_mode << 0, 0, 1, 1;

    std::vector<Vector7d> path;

    task->forward_integration(x_start, x_goal, envs, env_mode, &path, manipulator_config);

    std::cout << path.back().transpose() << std::endl;
}
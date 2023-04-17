#include <yaml-cpp/yaml.h>

#include "../tasks/task.h"

#include "../mechanics/utilities/sample.h"
#include "../mechanics/utilities/utilities.h"
#include "../mechanics/contacts/contact_kinematics.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#include "../mechanics/utilities/io.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

#include "../tasks/visualization.h"

void setup_task(std::string para_path, const Matrix4d &T_start, const Vector6d &v_goal,
                std::shared_ptr<TASK> task)
{

    // limited to 1 finger, cuboid object, maximum 10 environment blocks

    YAML::Node config = YAML::LoadFile(para_path);

    std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

    std::cout << "Loading environment blocks" << std::endl;
    for (int i = 1; i <= 10; ++i)
    {
        std::string block_name = "block_" + std::to_string(i);
        if (config["environment"][block_name.c_str()])
        {
            std::vector<double> loc =
                config["environment"][block_name.c_str()]["location"].as<std::vector<double>>();
            std::vector<double> dim =
                config["environment"][block_name.c_str()]["dimension"].as<std::vector<double>>();
            SkeletonPtr env_block =
                createFixedBox(block_name, Vector3d(dim[0], dim[1], dim[2]),
                               Vector3d(loc[0], loc[1], loc[2]));
            world->addEnvironmentComponent(env_block);
        }
    }

    std::cout << "Loading object" << std::endl;

    std::vector<double> box_l = config["object_shape"].as<std::vector<double>>();

    SkeletonPtr object =
        createFreeBox("object", Vector3d(box_l[0], box_l[1], box_l[2]),
                      Vector3d(0.7, 0.3, 0.3), 0.45);

    world->addObject(object);

    std::cout << "Loading contacts" << std::endl;

    int n_robot_contacts = config["number_of_robot_contacts"].as<int>();
    double robot_contact_radius = config["robot_contact_radius"].as<double>();
    DartPointManipulator *rpt =
        new DartPointManipulator(n_robot_contacts, robot_contact_radius);

    rpt->is_patch_contact = true;
    world->addRobot(rpt);

    // set the task parameters, start, goal, object inertial, etc....

    Vector7d x_start = SE32pose(T_start);
    Vector7d x_goal = SE32pose(T_start * se32SE3(v_goal));

    double goal_thr = box_l[0] * 3.14 * 10 / 180;

    std::cout << "Loading task parameters" << std::endl;

    double wa = config["wa"].as<double>();
    double wt = config["wt"].as<double>();

    double mu_env = config["environment_friction_coeff"].as<double>();
    double mu_mnp = config["robot_friction_coeff"].as<double>();

    double charac_len = config["characteristic_length"].as<double>();

    Vector6d f_g;
    f_g << 0, 0, -0.1, 0, 0, 0;

    Matrix6d oi;
    oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
        0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

    // search options, none
    TASK::SearchOptions rrt_options;
    // empty surface points
    std::vector<ContactPoint> surface_pts;
    surface_pts.push_back(ContactPoint(Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    surface_pts.push_back(ContactPoint(Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    surface_pts.push_back(ContactPoint(Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    surface_pts.push_back(ContactPoint(Vector3d(0, 0, 0), Vector3d(0, 0, 0)));

    std::cout << "Initializing task" << std::endl;

    // pass the world and task parameters to the task through task->initialize
    task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                     mu_mnp, oi, f_g, world, n_robot_contacts, "quasistatic",
                     surface_pts, rrt_options);
    std::cout << "Task initialized" << std::endl;
}

std::vector<double> parseDoubleVector(std::string str)
{
    std::vector<double> vec;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        vec.push_back(std::stod(token));
    }
    return vec;
}

std::vector<int> parseIntVector(std::string str)
{
    std::vector<int> vec;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        vec.push_back(std::stoi(token));
    }
    return vec;
}

void parser(int argc, char *argv[], std::string &param_path, std::string &save_path,  Matrix4d &T_start, Vector6d &v_goal, std::vector<ContactPoint> &mnps, std::vector<ContactPoint> &envs, VectorXi &env_cs_mode)
{
    std::cout << "argc: " << argc << std::endl;
    if (argc < 8)
    {
        // std::cout << "Usage: ./action_integrator [param_path] [x_start] [v_goal] [mnps (p,n,p,n) (must have all the fingers)][envs (p,n,p,n,...)] [env cs mode]" << std::endl;
        // exit(0);
        argv = new char *[8];
        argc = 7;
        argv[1] = (char *)"/home/xianyi/Research/MCTS/action_integrator/setup.yaml";
        argv[2] = (char *)"1,0,0,0.05,0,1,0,0,0,0,1,0.5,0,0,0,1";
        argv[3] = (char *)"1,0,1,0,0,0";
        argv[4] = (char *)"-0.5,0,0,1,0,0";
        argv[5] = (char *)"0.5,0.5,-0.5,0,0,1,0.5,-0.5,-0.5,0,0,1,-0.5,0.5,-0.5,0,0,1,-0.5,-0.5,-0.5,0,0,1";
        argv[6] = (char *)"0,0,0,0";
        argv[7] = (char *)"/home/xianyi/Research/MCTS/action_integrator/output_file.csv";
    }

    param_path = argv[1];
    save_path = argv[7];

    std::vector<double> x_start = parseDoubleVector(argv[2]);
    T_start << x_start[0], x_start[1], x_start[2], x_start[3],
        x_start[4], x_start[5], x_start[6], x_start[7],
        x_start[8], x_start[9], x_start[10], x_start[11],
        x_start[12], x_start[13], x_start[14], x_start[15];

    std::vector<double> v_goal_data = parseDoubleVector(argv[3]);
    v_goal << v_goal_data[0], v_goal_data[1], v_goal_data[2], v_goal_data[3], v_goal_data[4], v_goal_data[5];

    std::vector<double> mnps_data = parseDoubleVector(argv[4]);
    for (int i = 0; i < mnps_data.size(); i += 6)
    {
        ContactPoint cp;
        cp.p << mnps_data[i], mnps_data[i + 1], mnps_data[i + 2];
        cp.n << mnps_data[i + 3], mnps_data[i + 4], mnps_data[i + 5];
        mnps.push_back(cp);
    }

    std::vector<double> envs_data = parseDoubleVector(argv[5]);
    for (int i = 0; i < envs_data.size(); i += 6)
    {
        ContactPoint cp;
        cp.p << envs_data[i], envs_data[i + 1], envs_data[i + 2];
        cp.n << envs_data[i + 3], envs_data[i + 4], envs_data[i + 5];
        envs.push_back(cp);
    }

    std::vector<int> mode = parseIntVector(argv[6]);
    env_cs_mode.resize(mode.size());
    for (int i = 0; i < env_cs_mode.size(); i++)
    {
        env_cs_mode[i] = mode[i];
    }
}

int main(int argc, char *argv[])
{

    std::string param_path;
    std::string save_file_path;
    Matrix4d T_start;
    Vector6d v_goal;
    std::vector<ContactPoint> envs;
    std::vector<ContactPoint> mnps;
    VectorXi env_cs_mode;
    std::cout << "Parsing arguments" << std::endl;
    parser(argc, argv, param_path, save_file_path, T_start, v_goal, mnps, envs, env_cs_mode);
    std::cout << "Arguments parsed" << std::endl;

    std::shared_ptr<TASK> task = std::make_shared<TASK>();
    setup_task(param_path, T_start, v_goal, task);
    // card(task);

    std::vector<Vector7d> path;

    std::cout << "Forward integration" << std::endl;

    task->forward_integration_velocity(task->start_object_pose, v_goal, mnps, envs, env_cs_mode, &path);
    
    std::cout << "Final pose: " << path.back().transpose() << std::endl;

    // TODO: save to the csv file
    // first four row are the object transformation matrix
    // the rest are the object environment contacts
    std::ofstream file;
    file.open(save_file_path);
    Matrix4d T_final = pose2SE3(path.back());
    file << T_final(0, 0) << "," << T_final(0, 1) << "," << T_final(0, 2) << "," << T_final(0, 3) << std::endl;
    file << T_final(1, 0) << "," << T_final(1, 1) << "," << T_final(1, 2) << "," << T_final(1, 3) << std::endl;
    file << T_final(2, 0) << "," << T_final(2, 1) << "," << T_final(2, 2) << "," << T_final(2, 3) << std::endl;
    file << T_final(3, 0) << "," << T_final(3, 1) << "," << T_final(3, 2) << "," << T_final(3, 3) << std::endl;
    std::vector<ContactPoint> envs_final;
    task->m_world->getObjectContacts(&envs_final, path.back());
    for (auto env: envs_final)
    {
        file << env.p(0) << "," << env.p(1) << "," << env.p(2) << "," << env.n(0) << "," << env.n(1) << "," << env.n(2) << std::endl;
    }
    file.close();
    
    task->m_world->setObjectTrajectory(path);

    task->m_world->startWindow(&argc, argv);
}


#include <yaml-cpp/yaml.h>

#include "../search/level1.h"

#include "../tasks/inhand_task.h"

#include "../mechanics/utilities/utilities.h"
#include "../mechanics/utilities/sample.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

#include "visualization.h"

void VisualizeSG(std::shared_ptr<WorldTemplate> world, Vector7d start_pose,
                 Vector7d goal_pose, std::vector<Vector3d> finger_locations)
{
    std::vector<Vector7d> object_traj;
    // object_traj.push_back(start_pose);
    object_traj.push_back(goal_pose);
    std::vector<VectorXd> mnp_traj;
    VectorXd mnp_config(6 * finger_locations.size());
    for (int k = 0; k < finger_locations.size(); ++k)
    {

        mnp_config.block(6 * k, 0, 3, 1) =
            finger_locations[k];
        mnp_config.block(6 * k + 3, 0, 3, 1) =
            Vector3d::Zero();
    }
    // mnp_traj.push_back(mnp_config);
    mnp_traj.push_back(mnp_config);
    world->setPlaybackTrajectory(object_traj, mnp_traj);
}

Vector7d randomize_a_pose(double start_x, double start_y, double start_z)
{
    Quaterniond q_rand = generate_unit_quaternion();
    Vector7d x;
    x << start_x, start_y, start_z, q_rand.x(), q_rand.y(), q_rand.z(), q_rand.w();
    return x;
}


std::vector<Vector3d> get_finger_locations(YAML::Node & config, double object_scale)
{
    
    std::vector<Vector3d> goal_finger_locations;
    {
        
        double x = object_scale * config["goal_finger_locations"]["finger_1"]["x"].as<double>();
        double y = object_scale * config["goal_finger_locations"]["finger_1"]["y"].as<double>();
        double z = object_scale * config["goal_finger_locations"]["finger_1"]["z"].as<double>();
        goal_finger_locations.push_back(Vector3d(x, y, z));
    }
    {
        
        double x = object_scale * config["goal_finger_locations"]["finger_2"]["x"].as<double>();
        double y = object_scale * config["goal_finger_locations"]["finger_2"]["y"].as<double>();
        double z = object_scale * config["goal_finger_locations"]["finger_2"]["z"].as<double>();
        goal_finger_locations.push_back(Vector3d(x, y, z));
    }
    {
        
        double x = object_scale * config["goal_finger_locations"]["finger_3"]["x"].as<double>();
        double y = object_scale * config["goal_finger_locations"]["finger_3"]["y"].as<double>();
        double z = object_scale * config["goal_finger_locations"]["finger_3"]["z"].as<double>();
        goal_finger_locations.push_back(Vector3d(x, y, z));
    }
    {
        
        double x = object_scale * config["goal_finger_locations"]["finger_4"]["x"].as<double>();
        double y = object_scale * config["goal_finger_locations"]["finger_4"]["y"].as<double>();
        double z = object_scale * config["goal_finger_locations"]["finger_4"]["z"].as<double>();
        goal_finger_locations.push_back(Vector3d(x, y, z));
    }
    {
        
        double x = object_scale * config["goal_finger_locations"]["finger_5"]["x"].as<double>();
        double y = object_scale * config["goal_finger_locations"]["finger_5"]["y"].as<double>();
        double z = object_scale * config["goal_finger_locations"]["finger_5"]["z"].as<double>();
        goal_finger_locations.push_back(Vector3d(x, y, z));
    }
    return goal_finger_locations;

}

std::vector<Vector3d> get_finger_locations_by_indices(YAML::Node & config, const std::vector<ContactPoint> & object_surface_pt)
{
    
    std::vector<Vector3d> goal_finger_locations;
    {
        int idx = config["goal_finger_locations"]["finger_1_idx"].as<int>();
        if (idx == -1){
            goal_finger_locations.push_back(Vector3d::Zero());
        }
        else{
            goal_finger_locations.push_back(object_surface_pt[idx].p);
        }
    }
    {
        int idx = config["goal_finger_locations"]["finger_2_idx"].as<int>();
                if (idx == -1){
            goal_finger_locations.push_back(Vector3d::Zero());
        }
        else{
            goal_finger_locations.push_back(object_surface_pt[idx].p);
        }
    }
    {
        int idx = config["goal_finger_locations"]["finger_3_idx"].as<int>();
                if (idx == -1){
            goal_finger_locations.push_back(Vector3d::Zero());
        }
        else{
            goal_finger_locations.push_back(object_surface_pt[idx].p);
        }
    }
    {
        int idx = config["goal_finger_locations"]["finger_4_idx"].as<int>();
                if (idx == -1){
            goal_finger_locations.push_back(Vector3d::Zero());
        }
        else{
            goal_finger_locations.push_back(object_surface_pt[idx].p);
        }
    }
    {
        int idx = config["goal_finger_locations"]["finger_5_idx"].as<int>();
                if (idx == -1){
            goal_finger_locations.push_back(Vector3d::Zero());
        }
        else{
            goal_finger_locations.push_back(object_surface_pt[idx].p);
        }
    }
    return goal_finger_locations;

}
// This script is used to setup planning inhand manipulation experiments with different objects and different hand configs.

const InhandTASK::State2::Action InhandTASK::State2::no_action = InhandTASK::State2::Action(-1, -1);
const InhandTASK::State::Action InhandTASK::State::no_action = -1;

void setup(const std::string &object_name, std::shared_ptr<InhandTASK> task,
           double start_x, double start_y, double start_z,
           double goal_qx, double goal_qy, double goal_qz, double goal_qw,
           double scale)
{
    // hammer to two inhand locations (vertical and horizontal)

    // create world, create environment, an object sliding on the table

    // all data are store in the data folder under inhand_all

    // read surface point
    std::vector<ContactPoint> surface_pts;
    std::ifstream f(std::string(SRC_DIR) +
                    "/data/inhand_goal_finger_locations/" + object_name + ".csv");
    aria::csv::CsvParser parser(f);

    for (auto &row : parser)
    {
        int n_cols = row.size();
        assert(n_cols == 6);

        Vector6d v;
        for (int j = 0; j < 6; ++j)
        {
            v(j) = std::stod(row[j]);
        }
        Vector3d pos;
        pos << v(0) * scale, v(1) * scale, v(2) * scale;
        ContactPoint p(pos, v.tail(3));
        surface_pts.push_back(p);
    }

    std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

    SkeletonPtr object = createFreeObjectfromMesh(object_name, std::string(SRC_DIR) + "/data/inhand_all/data/" + object_name + ".stl", Vector3d(scale, scale, scale));

    world->addObject(object);

    int n_robot_contacts;

    Vector7d x_start;
    Vector7d x_goal;

    // hand_type == "5"
    {
        SkeletonPtr env1 =
            createFixedBox("ground", Vector3d(5, 5, 0.2), Vector3d(0, 0, -1000));
        n_robot_contacts = 5;
        DartPointManipulator *rpt = new DartPointManipulator(n_robot_contacts, 0.1);

        std::vector<Vector6d> workspace_limits;
        {
            // [x_min, x_max, y_min, y_max, z_min, z_max]

            // little finger
            Vector6d wl1;
            wl1 << -3, 0,
                0, 1.5,
                -1, 1;
            workspace_limits.push_back(wl1);

            // ring finger
            Vector6d wl2;
            wl2 << -2, 1,
                0, 1.5,
                -1, 1;
            workspace_limits.push_back(wl2);

            // middle finger
            Vector6d wl3;
            wl3 << -1, 2,
                0, 1.5,
                -1, 1;
            workspace_limits.push_back(wl3);

            // index finger
            Vector6d wl4;
            wl4 << 0, 3,
                0, 1.5,
                -1, 1;
            workspace_limits.push_back(wl4);

            // thumb
            Vector6d wl5;
            wl5 << 0, 3.5,
                -1.5, 1,
                -1, 1;
            workspace_limits.push_back(wl5);
        }

        rpt->set_workspace_limit(workspace_limits);
        rpt->is_patch_contact = true;

        world->addEnvironmentComponent(env1);
        world->addRobot(rpt);

        x_start = randomize_a_pose(start_x, start_y, start_z);
        x_goal << start_x, start_y, start_z, goal_qx, goal_qy, goal_qz, goal_qw;
    }

    // set the task parameters, start, goal, object inertial, etc....

    // x_start << start_x, start_y, start_z, 0, 0, 0, 1;
    // x_goal << start_x, start_y, start_z, 0, 0, -1, 0;

    long int start_finger_idx = -1;
    long int goal_finger_idx = -1;

    double goal_thr = 3.14 * 5 / 180;

    double wa = 1;
    double wt = 1;

    double mu_env = 0.2;
    double mu_mnp = 0.8;

    double charac_len = 1;

    Vector6d f_g;
    f_g << 0, 0, -0.1, 0, 0, 0;

    Matrix6d oi;
    oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
        0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

    // search options

    InhandTASK::SearchOptions rrt_options;

    rrt_options.x_ub << 4, 0, 4;
    rrt_options.x_lb << -3, 0, 0;

    rrt_options.eps_trans = 0.2;
    rrt_options.eps_angle = 3.14 * 20 / 180;
    rrt_options.max_samples = 100;

    rrt_options.goal_biased_prob = 0.8;

    bool is_refine = false;
    double refine_dist = 0.1;

    // pass the world and task parameters to the task through task->initialize

    task->initialize(x_start, x_goal, start_finger_idx, goal_finger_idx, goal_thr,
                     wa, wt, charac_len, mu_env, mu_mnp, f_g, world,
                     n_robot_contacts, surface_pts, rrt_options, is_refine,
                     refine_dist);
}

int main(int argc, char *argv[])
{

    std::string para_path = std::string(SRC_DIR) + "/data/inhand_goal_finger_locations/setup.yaml";
    YAML::Node config = YAML::LoadFile(para_path);

    // parse configurations
    std::string object_name = config["object"].as<std::string>();
    double object_scale = config["object_scale"].as<double>();

    bool visualize_setup = config["visualize_setup"].as<bool>();
    bool visualize_result = config["visualize_result"].as<bool>();

    double start_x = config["start_position"]["x"].as<double>();
    double start_y = config["start_position"]["y"].as<double>();
    double start_z = config["start_position"]["z"].as<double>();

    double goal_qx = config["goal_orientation"]["qx"].as<double>();
    double goal_qy = config["goal_orientation"]["qy"].as<double>();
    double goal_qz = config["goal_orientation"]["qz"].as<double>();
    double goal_qw = config["goal_orientation"]["qw"].as<double>();

    int random_seed = config["random_seed"].as<int>();

    double goal_finger_distance_thr = config["goal_finger_distance_threshold"].as<double>();

    HMP::Level1Tree<InhandTASK::State, InhandTASK::State2,
                    InhandTASK>::HierarchicalComputeOptions compute_options;

    compute_options.l1_1st.max_iterations = 10;
    compute_options.l1.max_iterations = 2;
    compute_options.l2_1st.max_iterations = 10;
    compute_options.l2.max_iterations = 2;
    compute_options.final_l2_1st.max_iterations = 10;
    compute_options.final_l2.max_iterations = 2;

    compute_options.l1.max_time = 30;

    std::srand(random_seed);

    std::shared_ptr<InhandTASK> task = std::make_shared<InhandTASK>();

    setup(object_name, task,
          start_x, start_y, start_z,
          goal_qx, goal_qy, goal_qz, goal_qw,
          object_scale);
    
        // goal finger locations in the object frame
    // std::vector<Vector3d> goal_finger_locations = get_finger_locations(config, object_scale);
    std::vector<Vector3d> goal_finger_locations = get_finger_locations_by_indices(config, task->object_surface_pts);
    task->set_goal_finger_locations(goal_finger_locations, goal_finger_distance_thr);

    if (visualize_setup)
    {
        VisualizeSG(task->m_world, task->start_object_pose, task->goal_object_pose, goal_finger_locations);
        task->m_world->startWindow(&argc, argv);
    }

    InhandTASK::State start_state = task->get_start_state();

    HMP::Level1Tree<InhandTASK::State, InhandTASK::State2, InhandTASK> tree(
        task, start_state, compute_options);

    tree.ita = 0.2;

    HMP::Node<InhandTASK::State> *current_node = tree.search_tree();

    std::vector<InhandTASK::State> object_trajectory;
    std::vector<InhandTASK::State2> action_trajectory;
    tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

    std::vector<Vector7d> object_traj;
    std::vector<VectorXd> mnp_traj;

    std::cout << "Best value " << current_node->m_value << std::endl;

    for (auto &action : action_trajectory)
    {
        std::cout << "Timestep " << action.timestep << std::endl;
        std::cout
            << "Pose "
            << task->saved_object_trajectory[action.timestep].m_pose.transpose()
            << std::endl;
        std::cout << "Fingers ";
        for (int jj : task->get_finger_locations(action.finger_index))
        {
            std::cout << jj << " ";
        }
        std::cout << std::endl;
    }

    double total_distance = 0;
    std::cout << "Finger locations: " << std::endl;
    for (int fidx : task->get_finger_locations(action_trajectory.back().finger_index))
    {
        if (fidx == -1){
            std::cout << "No finger" << std::endl;
            continue;
        }
        std::cout << task->object_surface_pts[fidx].p.transpose() << std::endl;
    }
    std::cout << "Total distance: " << task->get_finger_distance(action_trajectory.back().finger_index) << std::endl;

    output_results(&tree, task, object_trajectory, action_trajectory, current_node->m_value);

    if (visualize_result)
    {
        VializeStateTraj(task->m_world, task, object_trajectory, action_trajectory);
        task->m_world->startWindow(&argc, argv);
    }
}
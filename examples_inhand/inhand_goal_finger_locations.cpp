
#include <yaml-cpp/yaml.h>

#include "../search/level1.h"

#include "../tasks/inhand_task.h"

#include "../mechanics/utilities/utilities.h"
#include "../mechanics/utilities/sample.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#include "../mechanics/utilities/io.h"

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

std::vector<Vector3d> sample_goal_finger_locations(std::shared_ptr<InhandTASK> task, const Vector7d &x)
{
    // print sampled index
    //
    std::vector<Vector3d> sampled_finger_locations;
    std::vector<ContactPoint> object_surface_world;
    Eigen::Matrix3d R;
    R = quat2SO3(x(6), x(3), x(4), x(5));
    Eigen::Vector3d p;
    p = x.head(3);

    for (auto pt : task->object_surface_pts)
    {
        ContactPoint pt_w;
        pt_w.p = R * pt.p + p;
        pt_w.n = R * pt.n;
        object_surface_world.push_back(pt_w);
    }
    std::cout << "sampled finger locations: " << std::endl;

    for (int k = 0; k < task->number_of_robot_contacts; ++k)
    {
        std::vector<int> pws;
        task->m_world->getRobot()->points_in_workspace(k, object_surface_world,
                                                       &pws);

        // sample an int in pws
        if (pws.size() == 0)
        {
            sampled_finger_locations.push_back(Vector3d::Zero());
            continue;
        }
        int random_idx = randi(pws.size());
        std::cout << pws[random_idx] << " " << std::endl;
        sampled_finger_locations.push_back(task->object_surface_pts[pws[random_idx]].p);
    }

    return sampled_finger_locations;
}

// This script is used to setup planning inhand manipulation experiments with different objects and different hand configs.

const InhandTASK::State2::Action InhandTASK::State2::no_action = InhandTASK::State2::Action(-1, -1);
const InhandTASK::State::Action InhandTASK::State::no_action = -1;

void setup(const std::string &object_name, std::shared_ptr<InhandTASK> task,
           double start_x, double start_y, double start_z,
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
            createFixedBox("ground", Vector3d(5, 5, 0.2), Vector3d(0, 0, -1000), Vector3d(0.9, 0.9, 0.9), 0.01);
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
        x_goal = randomize_a_pose(start_x, start_y, start_z);
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

    int random_seed = config["random_seed"].as<int>();

    double goal_finger_distance_thr = config["goal_finger_distance_threshold"].as<double>();

    int number_of_runs = config["number_of_runs"].as<int>();

    std::string save_header = std::string(SRC_DIR) + "/data/inhand_goal_finger_locations/results/" + "5finger_" + object_name;
    std::string save_path = save_header + ".csv";
    std::string save_path_no_finger = save_header + "_no_goal_finger.csv";

    HMP::Level1Tree<InhandTASK::State, InhandTASK::State2,
                    InhandTASK>::HierarchicalComputeOptions compute_options;

    compute_options.l1_1st.max_iterations = 10;
    compute_options.l1.max_iterations = 2;
    compute_options.l2_1st.max_iterations = 10;
    compute_options.l2.max_iterations = 2;
    compute_options.final_l2_1st.max_iterations = 10;
    compute_options.final_l2.max_iterations = 2;

    compute_options.l1.max_time = 30;

    if (number_of_runs > 1)
    {
        for (int iter_run = 0; iter_run < number_of_runs; iter_run++)
        {
            std::srand(std::time(nullptr) + 100 * iter_run);

            VectorXd result_no(12);
            VectorXd result(12);

            // no goal finger locations
            {
                std::shared_ptr<InhandTASK> task = std::make_shared<InhandTASK>();

                setup(object_name, task,
                      start_x, start_y, start_z,
                      object_scale);

                std::vector<Vector3d> goal_finger_locations = sample_goal_finger_locations(task, task->goal_object_pose);
                task->set_goal_finger_locations(goal_finger_locations, goal_finger_distance_thr);
                task->if_goal_finger = false;

                InhandTASK::State start_state = task->get_start_state();

                HMP::Level1Tree<InhandTASK::State, InhandTASK::State2, InhandTASK> tree(
                    task, start_state, compute_options);

                tree.ita = 0.2;

                HMP::Node<InhandTASK::State> *current_node = tree.search_tree();

                std::vector<InhandTASK::State> object_trajectory;
                std::vector<InhandTASK::State2> action_trajectory;
                tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

                double total_distance = task->get_finger_distance(action_trajectory.back().finger_index);

                result_no.head(11) = get_inhand_result(&tree, task, object_trajectory, action_trajectory, current_node->m_value);
                result_no[11] = total_distance / double(task->number_of_robot_contacts);
            }

            {
                std::shared_ptr<InhandTASK> task = std::make_shared<InhandTASK>();

                setup(object_name, task,
                      start_x, start_y, start_z,
                      object_scale);

                std::vector<Vector3d> goal_finger_locations = sample_goal_finger_locations(task, task->goal_object_pose);
                task->set_goal_finger_locations(goal_finger_locations, goal_finger_distance_thr);

                InhandTASK::State start_state = task->get_start_state();

                HMP::Level1Tree<InhandTASK::State, InhandTASK::State2, InhandTASK> tree(
                    task, start_state, compute_options);

                tree.ita = 0.2;

                HMP::Node<InhandTASK::State> *current_node = tree.search_tree();

                std::vector<InhandTASK::State> object_trajectory;
                std::vector<InhandTASK::State2> action_trajectory;
                tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

                double total_distance = task->get_finger_distance(action_trajectory.back().finger_index);

                result.head(11) = get_inhand_result(&tree, task, object_trajectory, action_trajectory, current_node->m_value);
                result[11] = total_distance / double(task->number_of_robot_contacts);
            }
            appendData(save_path_no_finger, result_no.transpose());
            appendData(save_path, result.transpose());
        }

        return 0;
    }

    std::shared_ptr<InhandTASK> task = std::make_shared<InhandTASK>();

    setup(object_name, task,
          start_x, start_y, start_z,
          object_scale);

    // goal finger locations in the object frame
    // std::vector<Vector3d> goal_finger_locations = get_finger_locations(config, object_scale);
    // std::vector<Vector3d> goal_finger_locations = get_finger_locations_by_indices(config, task->object_surface_pts);
    std::vector<Vector3d> goal_finger_locations = sample_goal_finger_locations(task, task->goal_object_pose);
    task->set_goal_finger_locations(goal_finger_locations, goal_finger_distance_thr);

    if (!config["is_goal_finger"].as<bool>())
    {
        task->if_goal_finger = false;
    }

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

    std::cout << "Finger locations: " << std::endl;
    for (int fidx : task->get_finger_locations(action_trajectory.back().finger_index))
    {
        if (fidx == -1)
        {
            std::cout << "No finger" << std::endl;
            continue;
        }
        std::cout << task->object_surface_pts[fidx].p.transpose() << std::endl;
    }

    double total_distance = task->get_finger_distance(action_trajectory.back().finger_index);
    std::cout << "Total distance: " << total_distance << std::endl;
    std::cout << "Average distance: " << total_distance / double(task->number_of_robot_contacts) << std::endl;

    get_inhand_result(&tree, task, object_trajectory, action_trajectory, current_node->m_value);

    if (visualize_result)
    {
        VializeStateTraj(task->m_world, task, object_trajectory, action_trajectory);
        task->m_world->startWindow(&argc, argv);
    }
}
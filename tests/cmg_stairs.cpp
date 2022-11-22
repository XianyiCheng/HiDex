
#include "../search/level1.h"

#include "../tasks/cmg_task.h"

#include "../mechanics/utilities/utilities.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

#include "visualization.h"

void stairs(std::shared_ptr<CMGTASK> task)
{
    // create world, create environment, an object sliding on the table

    std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

    SkeletonPtr object =
        createFreeBox("box_object", Vector3d(1, 1, 1));
    SkeletonPtr env1 =
        createFixedBox("ground_1", Vector3d(10, 4, 1), Vector3d(0, 0, 0.5));
    SkeletonPtr env2 =
        createFixedBox("ground_2", Vector3d(5, 4, 0.98), Vector3d(0, 0, 1.5));
    SkeletonPtr env3 =
        createFixedBox("ground_3", Vector3d(2, 4, 0.98), Vector3d(0, 0, 2.5));

    world->addObject(object);
    world->addEnvironmentComponent(env1);
    world->addEnvironmentComponent(env2);
    world->addEnvironmentComponent(env3);

    int n_robot_contacts = 1;
    DartPointManipulator *rpt = new DartPointManipulator(n_robot_contacts, 0.2);
    rpt->is_patch_contact = true;
    world->addRobot(rpt);

    // set the task parameters, start, goal, object inertial, etc....

    Vector7d x_start;
    Vector7d x_goal;
    x_start << 4, 0, 1+0.5 * 0.9999, 0, 0, 0, 1;
    // x_start << 2.47755 , 0.0264446  ,  1.68066  , 0,0,0,1;
    // x_goal << 2.47755 , 0.0264446  ,  1.68066 + 0.2  , 0.125041 ,-0.0264445 ,0.00948866 ,  0.991754;
    // x_goal << 3, 0, 2+0.5 * 0.9999, 0, 0, 0, 1;
    x_goal << 0, 0, 3+0.5 * 0.9999, 0, 0, 0, 1;
    // x_goal << -3, 0, 1+0.5 * 0.9999, 0, 0, 0, 1;

    double goal_thr = 3.14 * 60 / 180;
    // double goal_thr = 0.05;

    double wa = 0.1;
    double wt = 1;

    double mu_env = 0.4;
    double mu_mnp = 0.8;

    double charac_len = 1;

    Vector6d f_g;
    f_g << 0, 0, -0.1, 0, 0, 0;

    Matrix6d oi;
    oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
        0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

    // search options

    CMGTASK::SearchOptions rrt_options;

    rrt_options.x_ub << 4, 0, 4;
    rrt_options.x_lb << -3, 0, 0;
    rrt_options.sampleSO3 = false;
    rrt_options.sample_rotation_axis = Vector3d(0, 1, 0);

    rrt_options.eps_trans = 1.0;
    rrt_options.eps_angle = 3.14 * 50 / 180;
    rrt_options.max_samples = 200;

    rrt_options.goal_biased_prob = 0.6;

    bool is_refine = false;
    double refine_dist = 0.5;

    // read surface point, add robot contacts
    std::vector<ContactPoint> surface_pts;
    std::ifstream f(std::string(SRC_DIR) +
                    "/data/test_cmg_stairs/surface_contacts.csv");
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
        ContactPoint p(
            v.head(3),
            v.tail(3));
        surface_pts.push_back(p);
    }
    // pass the world and task parameters to the task through task->initialize
    task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                     mu_mnp, oi, f_g, world, n_robot_contacts, CMG_QUASISTATIC,
                     surface_pts, rrt_options, is_refine, refine_dist);
    // VisualizeSG(task->m_world, x_start, x_goal);
}

int main(int argc, char *argv[])
{
    std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

    stairs(task);

    CMGTASK::State start_state = task->get_start_state();

    HMP::Level1Tree<CMGTASK::State, CMGTASK::State2,
                    CMGTASK>::HierarchicalComputeOptions compute_options;

    compute_options.l1_1st.max_iterations = 10;
    compute_options.l1.max_iterations = 2;
    compute_options.l2_1st.max_iterations = 10;
    compute_options.l2.max_iterations = 2;
    compute_options.final_l2_1st.max_iterations = 10;
    compute_options.final_l2.max_iterations = 3;

    HMP::Level1Tree<CMGTASK::State, CMGTASK::State2, CMGTASK> tree(
        task, start_state, compute_options);

    HMP::Node<CMGTASK::State> *current_node = tree.search_tree();

    std::vector<CMGTASK::State> object_trajectory;
    std::vector<CMGTASK::State2> action_trajectory;
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

    VializeStateTraj(task->m_world, task, object_trajectory, action_trajectory);

    std::cout << "Total level 1 tree nodes " << tree.count_total_nodes()
              << std::endl;

    std::cout << "Total shared rrt nodes " << tree.m_task->total_rrt_nodes()
              << std::endl;

    task->m_world->startWindow(&argc, argv);
}


#include "../src/tasks/wholehand_setup.h"

// test how many points are needed to sample on each finger part to get a good estimate of penetration distance

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

    task->initialize();

    std::vector<double> box_l =
        config["box_object"]["shape"].as<std::vector<double>>();
    Vector3d box_shape(box_l[0], box_l[1], box_l[2]);

    // // ---- Test rough IK SDF ----

    Vector7d object_pose = task->start_object_pose;
    std::vector<ContactPoint> repell_contact_points;
    repell_contact_points = task->object_surface_pts;
    for (int k = 0; k < repell_contact_points.size() - 10; k++)
    {
        repell_contact_points.erase(repell_contact_points.begin() + randi(repell_contact_points.size()));
    }

    std::vector<ContactPoint> contact_points;
    double l = 0.1;
    contact_points.push_back(ContactPoint(Vector3d(0.0, 0.0, l / 2), Vector3d(0, 0, -1)));
    contact_points.push_back(ContactPoint(Vector3d(l / 2, 0.0, 0.0), Vector3d(-1, 0, 0)));

    double d_contact = (contact_points[0].p - contact_points[1].p).norm();

    std::cout << "Distance between two contact points " << d_contact << std::endl;

    std::vector<VectorXd> sdf_ik_solutions;

    std::vector<std::string> allowed_parts = task->getRobot()->allowed_part_names;
    std::vector<int> allowed_part_idxes = task->getRobot()->allowed_part_point_idxes;

    {
        std::vector<std::string> part_names = {"base_link", "link_7_tip"};
        std::vector<int> part_p_idxes = {8150, 2649};
        task->getRobot()->roughIKsolutions(part_names, part_p_idxes, contact_points, object_pose, VectorXd::Zero(0), &sdf_ik_solutions);
    }
    {
        std::vector<std::string> part_names = {"base_link", "link_3_tip"};
        std::vector<int> part_p_idxes = {8150, 2649};
        task->getRobot()->roughIKsolutions(part_names, part_p_idxes, contact_points, object_pose, VectorXd::Zero(0), &sdf_ik_solutions);
    }
    {
        std::vector<std::string> part_names = {"link_15_tip", "link_3_tip"};
        std::vector<int> part_p_idxes = {2114, 2649};
        task->getRobot()->roughIKsolutions(part_names, part_p_idxes, contact_points, object_pose, VectorXd::Zero(0), &sdf_ik_solutions);
    }

    std::vector<int> n_samples = {1000000, 2000, 1000, 500, 300, 200, 100, 50, 20, 10, 5, 1};
    for (auto config : sdf_ik_solutions)
    {
        for (int n_sample : n_samples)
        {
            double penetrate_d = task->getRobot()->maxPenetrationDistance(config, object_pose, box_shape, n_sample);
            double average_signed_d = task->getRobot()->averagePenetrateDistance(config, object_pose, box_shape, n_sample);
            std::cout << "Sampled points " << n_sample << ", max penetration distance: " << penetrate_d << ", average signed distance: " << average_signed_d << std::endl;
        }
    }
}
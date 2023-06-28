#include <cstdio>
#include <yaml-cpp/yaml.h>

#include "../src/mechanics/utilities/parser.hpp"

#ifndef DART_UTILS
#define DART_UTILS
#include "../src/mechanics/dart_utils/dart_utils.h"
#endif

#include "../src/mechanics/utilities/io.h"

#define CW 1
#define CCW -1
#define NONE 0

VectorXd world_to(VectorXd config_world, Vector7d object_pose)
{
    Matrix3d R =
        quat2SO3(object_pose(6), object_pose(3), object_pose(4), object_pose(5));
    Matrix3d R_inv = R.transpose();
    Vector3d t = object_pose.segment(0, 3);

    Vector3d pw = config_world.head(3);
    Vector3d nw = config_world.tail(3);
    Vector3d p = R_inv * (pw - t);
    Vector3d n = R_inv * nw;
    VectorXd config(6);
    config << p, n;
    return config;
}

VectorXd to_world(VectorXd config, Vector7d object_pose)
{
    Matrix3d R =
        quat2SO3(object_pose(6), object_pose(3), object_pose(4), object_pose(5));
    Vector3d t = object_pose.segment(0, 3);

    Vector3d p = config.head(3);
    Vector3d n = config.tail(3);
    Vector3d pw = R * p + t;
    Vector3d nw = R * n;
    VectorXd config_world(6);
    config_world << pw, nw;
    return config_world;
}

std::vector<int> get_active_finger_idxes(const VectorXd &config)
{
    std::vector<int> idx;
    if (config.size() < 12)
    {
        return idx;
    }
    double n0 = config.segment<3>(3).norm();
    double n1 = config.segment<3>(9).norm();
    if ((n0 > 0.5) && (n0 < 1.5))
        idx.push_back(0);
    if ((n1 > 0.5) && (n1 < 1.5))
        idx.push_back(1);
}

std::vector<int> getRelocateFingerIdx(const VectorXd &config_before,
                                      const VectorXd &config_after)
{
    std::vector<int> idx;
    if (config_before.size() < 12 || config_after.size() < 12)
    {
        return idx;
    }
    double p0 = (config_before.segment<3>(0) - config_after.segment<3>(0)).norm();
    double p1 = (config_before.segment<3>(6) - config_after.segment<3>(6)).norm();
    if (p0 > 1e-5)
        idx.push_back(0);
    if (p1 > 1e-5)
        idx.push_back(1);
}

int nearest_point(const std::vector<ContactPoint> &pts, Vector3d p,
                  int rotation_dir)
{
    int idx = 0;
    double min_dist = 1e10;
    for (int i = 0; i < pts.size(); ++i)
    {
        if (rotation_dir == CW)
        {
            if (!(pts[i].p.cross(p)[1] > 0))
            {
                continue;
            }
        }
        else if (rotation_dir == CCW)
        {
            // counter clockwise
            if (!(p.cross(pts[i].p)[1] > 0))
            {
                continue;
            }
        }
        double dist = (pts[i].p - p).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }
    return idx;
}

int nearest_point_to_surface(const std::vector<ContactPoint> &pts, Vector3d p,
                             double surface_distance)
{
    int idx = 0;
    double min_dist = 1e10;
    for (int i = 0; i < pts.size(); ++i)
    {
        double dist = (pts[i].p - surface_distance * pts[i].n - p).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }
    return idx;
}

std::vector<ContactPoint>
generate_traj_on_box_surface(ContactPoint p_s, ContactPoint p_g,
                             std::vector<ContactPoint> surface_pts,
                             int rotation_dir, double surface_dist)
{
    std::vector<ContactPoint> traj;
    Vector3d p_cur = p_s.p;
    traj.push_back(ContactPoint(p_s.p - p_s.n * surface_dist, p_s.n));
    double d_prev = 1e10;
    double d_cur = 1e6;
    while ((d_cur > 1e-2))
    {
        d_prev = d_cur;
        int k = nearest_point(surface_pts, p_cur, rotation_dir);
        p_cur = surface_pts[k].p;
        traj.push_back(ContactPoint(p_cur - surface_pts[k].n * surface_dist,
                                    surface_pts[k].n));
        d_cur = (p_cur - p_g.p).norm();
    }
    return traj;
}

template <class Hand>
Vector3d compute_default_location(Vector3d p, Vector3d n, const Vector7d &x,
                                  int finger_idx, Hand *hand, double max_dist)
{
    double step = max_dist / 10;
    VectorXd mnp_config(12);
    mnp_config << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Vector3d p_new;
    p_new = p;
    mnp_config.segment(6 * finger_idx, 3) = p_new;
    mnp_config.segment(6 * finger_idx + 3, 3) = n;
    while ((hand->isIK(mnp_config, x)) && (p_new - p).norm() < max_dist)
    {
        p_new += step * n;
        mnp_config.segment(6 * finger_idx, 3) = p_new;
    }
    p_new += step * n;
    return p_new;
}

template <class Task>
bool check_trajectory_IK_and_collision(const std::vector<ContactPoint> traj,
                                       const Vector7d &x,
                                       std::shared_ptr<Task> task,
                                       int finger_idx)
{

    // return true;

    VectorXd mnp_config(12);
    mnp_config << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    task->m_world->updateObjectPose(x);

    for (auto p : traj)
    {

        mnp_config.segment(6 * finger_idx, 3) = p.p;
        mnp_config.segment(6 * finger_idx + 3, 3) = p.n;

        // if there is no ik solution, not valid
        if (!task->m_world->getRobot()->ifIKsolution(mnp_config, x))
        {
            return false;
        }

        // if the robot collides, not valid
        if (task->m_world->isRobotCollide(mnp_config))
        {
            return false;
        }
    }
    return true;
}

template <class Task, class Hand>
std::vector<VectorXd>
planning_relocation_trajectory(std::shared_ptr<Task> task, Hand *hand,
                               const VectorXd &mnp_config_before_world,
                               const VectorXd &mnp_config_after_world,
                               const Vector7d &x,
                               double distance_to_surface)
{
    // calculate active idxes for before and after
    std::vector<int> active_idx_before =
        get_active_finger_idxes(mnp_config_before_world);
    std::vector<int> active_idx_after =
        get_active_finger_idxes(mnp_config_after_world);

    // calculate relocate idxes for before and after
    std::vector<int> relocate_idx =
        getRelocateFingerIdx(mnp_config_before_world, mnp_config_after_world);

    // define a default place for finger not active
    // calculate the trajectory for each finger
    VectorXd mnp_config_now_world(12);
    mnp_config_now_world = mnp_config_before_world;
    VectorXd mnp_config_now(12);
    mnp_config_now.head(6) = world_to(mnp_config_before_world.head(6), x);
    mnp_config_now.tail(6) = world_to(mnp_config_before_world.tail(6), x);

    std::vector<VectorXd> outputs;

    for (int k : relocate_idx)
    {
        // check if k in active_idx_before
        bool active_before =
            std::find(active_idx_before.begin(), active_idx_before.end(), k) !=
            active_idx_before.end();
        // check if k in active_idx_after
        bool active_after =
            std::find(active_idx_after.begin(), active_idx_after.end(), k) !=
            active_idx_after.end();

        // if k is active before and after, find the trajectory
        double surface_distance = distance_to_surface; // 5mm
        std::vector<ContactPoint> traj_pts;
        VectorXd config_pt_before =
            world_to(mnp_config_before_world.segment(6 * k, 6), x);
        VectorXd config_pt_after =
            world_to(mnp_config_after_world.segment(6 * k, 6), x);
        ContactPoint pt_before = ContactPoint(config_pt_before.segment(0, 3),
                                              config_pt_before.segment(3, 3));
        ContactPoint pt_after = ContactPoint(config_pt_after.segment(0, 3),
                                             config_pt_after.segment(3, 3));
        if (active_before && active_after)
        {

            std::vector<ContactPoint> traj_pts_cw = generate_traj_on_box_surface(
                pt_before, pt_after, task->object_surface_pts, CW, surface_distance);
            std::vector<ContactPoint> traj_pts_ccw = generate_traj_on_box_surface(
                pt_before, pt_after, task->object_surface_pts, CCW, surface_distance);

            bool if_cw = check_trajectory_IK_and_collision(traj_pts_cw, x, task, k);
            bool if_ccw = check_trajectory_IK_and_collision(traj_pts_ccw, x, task, k);
            if (if_cw && if_ccw)
            {
                traj_pts = traj_pts_cw.size() < traj_pts_ccw.size() ? traj_pts_cw
                                                                    : traj_pts_ccw;
            }
            else if (if_cw)
            {
                traj_pts = traj_pts_cw;
            }
            else if (if_ccw)
            {
                traj_pts = traj_pts_ccw;
            }
            else
            {
                std::cout << "No valid relocation trajectory for finger " << k
                          << std::endl;
            }
            traj_pts.push_back(pt_after);
        }
        // if k is active before and not active after, find the trajectory to
        // default location
        else if (active_before && !active_after)
        {
            Vector3d p = pt_before.p;
            Vector3d n_outward = -pt_before.n;
            Vector3d p_default = compute_default_location(p, n_outward, x, k, hand, distance_to_surface);
            traj_pts.push_back(ContactPoint((p + p_default) / 2,
                                            Vector3d(1, 1, 1)));
            traj_pts.push_back(ContactPoint(p_default, Vector3d(1, 1, 1)));
        }

        // if k is not active before and active after, find the trajectory from
        // default location
        else if (!active_before && active_after)
        {
            Vector3d p = pt_before.p;
            // int near_idx = nearest_point(task->object_surface_pts, p, NONE);
            int near_idx = nearest_point_to_surface(task->object_surface_pts, p, surface_distance);

            std::vector<ContactPoint> traj_pts_cw = generate_traj_on_box_surface(
                task->object_surface_pts[near_idx], pt_after,
                task->object_surface_pts, CW, surface_distance);
            std::vector<ContactPoint> traj_pts_ccw = generate_traj_on_box_surface(
                task->object_surface_pts[near_idx], pt_after,
                task->object_surface_pts, CCW, surface_distance);

            std::vector<ContactPoint> traj_pts_next;
            bool if_cw = check_trajectory_IK_and_collision(traj_pts_cw, x, task, k);
            bool if_ccw = check_trajectory_IK_and_collision(traj_pts_ccw, x, task, k);
            if (if_cw && if_ccw)
            {
                traj_pts_next = traj_pts_cw.size() < traj_pts_ccw.size() ? traj_pts_cw
                                                                         : traj_pts_ccw;
            }
            else if (if_cw)
            {
                traj_pts_next = traj_pts_cw;
            }
            else if (if_ccw)
            {
                traj_pts_next = traj_pts_ccw;
            }
            else
            {
                std::cout << "No valid relocation trajectory for finger " << k
                          << std::endl;
            }
            traj_pts.insert(traj_pts.end(), traj_pts_next.begin(),
                            traj_pts_next.end());
            traj_pts.push_back(pt_after);
        }

        for (auto pt : traj_pts)
        {
            VectorXd output(4 + 12 + 7);
            VectorXd pt_config(6);
            pt_config << pt.p, pt.n;
            mnp_config_now.segment(6 * k, 6) = pt_config;
            mnp_config_now_world.segment(6 * k, 6) = to_world(pt_config, x);
            output.head(4) = hand->getHandFrameConfig(mnp_config_now, x);
            output.segment(4, 12) = mnp_config_now_world;
            output.tail(7) = x;
            outputs.push_back(output);
        }
    }

    return outputs;
}

template <class State, class State2, class Task>
MatrixXd get_ddhand_output(const std::vector<State> &object_trajectory,
                           const std::vector<State2> &action_trajectory,
                           std::shared_ptr<Task> task,
                           const std::string &para_path)
{

    YAML::Node config = YAML::LoadFile(para_path);

    std::vector<Vector3d> defaults;
    {
        std::vector<double> loc =
            config["fixed_ddhand"]["default_locations"]["finger_1"].as<std::vector<double>>();
        defaults.push_back(Vector3d(loc[0], loc[1], loc[2]));
    }
    {
        std::vector<double> loc =
            config["fixed_ddhand"]["default_locations"]["finger_2"].as<std::vector<double>>();
        defaults.push_back(Vector3d(loc[0], loc[1], loc[2]));
    }

    Vector3d calibration_location;
    {
        std::vector<double> loc =
            config["fixed_ddhand"]["calibration_location"].as<std::vector<double>>();
        calibration_location = Vector3d(loc[0], loc[1], loc[2]);
    }

    int fingertype;
    if (config["fixed_ddhand"]["finger_type"].as<std::string>() == "vertical")
    {
        fingertype = I_FINGER;
    }
    else if (config["fixed_ddhand"]["finger_type"].as<std::string>() == "horizontal")
    {
        fingertype = L_FINGER;
    }
    else
    {
        std::cout << "No valid finger type specified for fixed_ddhand! Exit program." << std::endl;
        exit(0);
    }

    DartDDHandScalable *hand = new DartDDHandScalable(fingertype, config["fixed_ddhand"]["scale"].as<double>());
    std::vector<double> hand_pose =
        config["fixed_ddhand"]["hand_pose"].as<std::vector<double>>();
    Vector7d hand_pos;
    hand_pos << hand_pose[0], hand_pose[1], hand_pose[2], hand_pose[3],
        hand_pose[4], hand_pose[5], hand_pose[6];
    hand->setHandFrameTransform(hand_pos);

    std::vector<VectorXd> output;
    int t_span = 10;

    // first row: calibration location
    {
        VectorXd output_row(4 + 6 * task->number_of_robot_contacts + 7);
        VectorXd mnp_config(12);
        mnp_config << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        mnp_config.head(3) = calibration_location;
        mnp_config.segment(3, 3) = Vector3d(1, 1, 1);
        output_row.head(4) =
            hand->getHandFrameConfig(mnp_config, task->start_object_pose);
        output_row.segment(4, 6) =
            to_world(mnp_config.head(6), task->start_object_pose);
        output_row.tail(7) = task->start_object_pose;
        output.push_back(output_row);

        if (!hand->isIK(mnp_config, task->start_object_pose))
        {
            std::cout << "[Warning] IK failed at calibration location" << std::endl;
        }
    }

    // second row: default location
    {
        VectorXd output_row(4 + 6 * task->number_of_robot_contacts + 7);
        VectorXd mnp_config(12);
        mnp_config << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        mnp_config.head(3) = defaults[0];
        mnp_config.segment(3, 3) = Vector3d(1, 1, 1);
        mnp_config.segment(6, 3) = defaults[1];
        mnp_config.segment(9, 3) = Vector3d(1, 1, 1);
        output_row.head(4) =
            hand->getHandFrameConfig(mnp_config, task->start_object_pose);
        output_row.segment(4, 6) =
            to_world(mnp_config.head(6), task->start_object_pose);
        output_row.segment(10, 6) =
            to_world(mnp_config.tail(6), task->start_object_pose);
        output_row.tail(7) = task->start_object_pose;
        output.push_back(output_row);

        if (!hand->isIK(mnp_config, task->start_object_pose))
        {
            std::cout << "[Warning] IK failed at default location" << std::endl;
        }
    }

    for (int i = 0; i < action_trajectory.size(); ++i)
    {
        // {
        //   VectorXd output_row(4 + 6 * task->number_of_robot_contacts + 7);
        //   output_row.setZero();
        //   output.push_back(output_row);
        // }
        int t = action_trajectory[i].timestep;
        if (t == -1)
        {
            continue;
        }
        int t_next;
        if (i == action_trajectory.size() - 1)
        {
            t_next = object_trajectory.size() - 1;
        }
        else
        {
            t_next = action_trajectory[i + 1].timestep;
        }

        VectorXd mnp_config = task->get_robot_config_from_action_idx(
            action_trajectory[i].finger_index);

        // if (i > 1)
        {

            VectorXd mnp_config_world(12);
            Vector7d x = output.back().tail(7);

            for (int n_pt = 0; n_pt < task->number_of_robot_contacts; ++n_pt)
            {
                if (std::isnan(mnp_config[6 * n_pt]))
                {
                    mnp_config_world.segment(6 * n_pt, 6) =
                        mnp_config.segment(6 * n_pt, 6);
                    continue;
                }
                Vector3d p = mnp_config.segment(6 * n_pt, 3);
                Vector3d n = mnp_config.segment(6 * n_pt + 3, 3);
                Eigen::Matrix3d R;
                R = quat2SO3(x(6), x(3), x(4), x(5));
                Vector3d p_world = R * p + x.segment(0, 3);
                Vector3d n_world = R * n;
                mnp_config_world.segment(6 * n_pt, 3) = p_world;
                mnp_config_world.segment(6 * n_pt + 3, 3) = n_world;
            }

            std::vector<VectorXd> output_relocation = planning_relocation_trajectory(
                task, hand,
                output.back().segment(4, 6 * task->number_of_robot_contacts),
                mnp_config_world, output.back().tail(7), config["fixed_ddhand"]["relocation_distance_to_surface"].as<double>());

            output.insert(output.end(), output_relocation.begin(),
                          output_relocation.end());
        }

        // get the array of object pose
        std::vector<Vector7d> object_poses;
        if (t_next <= t)
        {
            // for (int kk = 0; kk < t_span; ++kk) {
            object_poses.push_back(object_trajectory[t].m_pose);
        }
        else
        {
            std::vector<Vector7d> object_poses_all;
            // object_poses_all.push_back(object_trajectory[t].m_pose);
            for (int kk = t + 1; kk <= t_next; ++kk)
            {
                if (object_trajectory[kk - 1].m_path.size() > 0)
                {
                    if ((object_trajectory[kk].m_path[0] - object_trajectory[kk - 1].m_path[0]).norm() < 1e-4)
                    {
                        continue;
                    }
                }
                object_poses_all.insert(object_poses_all.end(),
                                        object_trajectory[kk].m_path.begin(),
                                        object_trajectory[kk].m_path.end());
            }
            object_poses = object_poses_all;
            // int n_span = std::max(int(object_poses_all.size())- 2, 0) / 3;
            // object_poses.push_back(object_poses_all[0]);
            // object_poses.push_back(object_poses_all[n_span]);
            // object_poses.push_back(object_poses_all[2 * n_span]);
            // object_poses.push_back(object_poses_all[3 * n_span]);
            // object_poses.push_back(object_poses_all[object_poses_all.size() - 1]);
        }

        // for each object pose, get the array of mnp config

        for (auto x : object_poses)
        {
            VectorXd mnp_config_world(6 * task->number_of_robot_contacts);
            std::vector<int> active_idxes = get_active_finger_idxes(mnp_config);
            for (int n_pt = 0; n_pt < task->number_of_robot_contacts; ++n_pt)
            {
                // if (mnp_config.segment(6 * n_pt,3).norm() < 0.1) {
                //   mnp_config_world.segment(6 * n_pt, 6) =
                //       mnp_config.segment(6 * n_pt, 6);
                //   continue;
                // }
                Vector3d p = mnp_config.segment(6 * n_pt, 3);
                Vector3d n = mnp_config.segment(6 * n_pt + 3, 3);
                Eigen::Matrix3d R;
                R = quat2SO3(x(6), x(3), x(4), x(5));
                Vector3d p_world = R * p + x.segment(0, 3);
                Vector3d n_world = R * n;
                if (n_world.norm() < 0.1)
                {
                    n_world = Vector3d(1, 1, 1);
                }
                mnp_config_world.segment(6 * n_pt, 3) = p_world;
                mnp_config_world.segment(6 * n_pt + 3, 3) = n_world;

                if (std::find(active_idxes.begin(), active_idxes.end(), n_pt) ==
                    active_idxes.end())
                {
                    if (output.size() > 0)
                    {
                        mnp_config_world.segment(6 * n_pt, 6) =
                            output.back().segment(4 + 6 * n_pt, 6);
                        mnp_config.segment(6 * n_pt, 6) = world_to(output.back().segment(4 + 6 * n_pt, 6),
                                                                   output.back().tail(7));
                        mnp_config.segment(6 * n_pt + 3, 3) = Vector3d(1, 1, 1);
                    }
                    else
                    {
                        mnp_config.segment(6 * n_pt, 3) = defaults[n_pt];
                        mnp_config.segment(6 * n_pt + 3, 3) = Vector3d(1, 1, 1);
                        mnp_config_world = to_world(mnp_config, task->start_object_pose);
                    }
                }
            }

            VectorXd output_row(4 + 6 * task->number_of_robot_contacts + 7);

            output_row.segment(0, 4) = hand->getHandFrameConfig(mnp_config, x);
            output_row.segment(4, 6 * task->number_of_robot_contacts) =
                mnp_config_world;
            output_row.segment(4 + 6 * task->number_of_robot_contacts, 7) = x;

            output.push_back(output_row);
        }
    }

    MatrixXd output_mat(output.size(),
                        4 + 6 * task->number_of_robot_contacts + 7);
    for (int i = 0; i < output.size(); ++i)
    {
        output_mat.row(i) = output[i];
    }
    return output_mat;
}

void visualize_ddhand_output_file(std::shared_ptr<WorldTemplate> world,
                                  std::string file_name)
{
    MatrixXd data = openData(file_name);
    int n_data = data.rows();
    int n_pts = (data.cols() - 7 - 4) / 6;
    std::vector<Vector7d> object_traj;
    std::vector<VectorXd> mnp_traj;
    for (int i = 0; i < n_data; ++i)
    {
        // if all elements are zero, skip this row
        if (data.row(i).sum() == 0)
        {
            continue;
        }
        VectorXd mnp_config_world = data.row(i).segment(4, 6 * n_pts);
        VectorXd mnp_config(6 * n_pts);
        Vector7d object_pose = data.row(i).segment(4 + 6 * n_pts, 7);

        Matrix3d R = quat2SO3(object_pose(6), object_pose(3), object_pose(4),
                              object_pose(5));
        Matrix3d R_inv = R.transpose();
        Vector3d t = object_pose.segment(0, 3);
        for (int j = 0; j < n_pts; ++j)
        {
            if (std::isnan(mnp_config_world[6 * j]))
            {
                mnp_config.segment(6 * j, 6) = mnp_config_world.segment(6 * j, 6);
                continue;
            }
            Vector3d pw = mnp_config_world.segment(6 * j, 3);
            Vector3d nw = mnp_config_world.segment(6 * j + 3, 3);
            Vector3d p = R_inv * (pw - t);
            Vector3d n = R_inv * nw;
            mnp_config.segment(6 * j, 3) = p;
            mnp_config.segment(6 * j + 3, 3) = n;
        }
        object_traj.push_back(object_pose);
        mnp_traj.push_back(mnp_config);
    }
    world->setPlaybackTrajectory(object_traj, mnp_traj);
}

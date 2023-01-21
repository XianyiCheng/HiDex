#include <cstdio>
#include <yaml-cpp/yaml.h>

#include "../search/level1.h"

#include "../tasks/cmg_task.h"

#include "../mechanics/utilities/utilities.h"

#include "../mechanics/manipulators/DartDDHand.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

#include "../mechanics/utilities/io.h"
#include "../tasks/visualization.h"

template <class State, class State2, class Task>
MatrixXd get_ddhand_output(const std::vector<State> &object_trajectory,
                           const std::vector<State2> &action_trajectory,
                           std::shared_ptr<Task> task, std::string para_path) {

  YAML::Node config = YAML::LoadFile(para_path);
  double hand_x = config["hand_position"]["x"].as<double>();
  double hand_y = config["hand_position"]["y"].as<double>();
  double hand_z = config["hand_position"]["z"].as<double>();
  DartDDHand *hand = new DartDDHand(L_FINGER);
  Vector7d handx;
  handx << hand_x, hand_y, hand_z, 0, 0, 0, 1;

  hand->setHandFrameTransform(handx);
  std::vector<VectorXd> output;
  int t_span = 10;

  for (int i = 1; i < action_trajectory.size(); ++i) {
    {
      VectorXd output_row(4 + 6 * task->number_of_robot_contacts + 7);
      output_row.setZero();
      output.push_back(output_row);
    }
    int t = action_trajectory[i].timestep;
    int t_next;
    if (i == action_trajectory.size() - 1) {
      t_next = object_trajectory.size() - 1;
    } else {
      t_next = action_trajectory[i + 1].timestep;
    }

    VectorXd mnp_config = task->get_robot_config_from_action_idx(
        action_trajectory[i].finger_index);

    // get the array of object pose
    std::vector<Vector7d> object_poses;
    if (t_next <= t) {
      for (int kk = 0; kk < t_span; ++kk) {
        object_poses.push_back(object_trajectory[t].m_pose);
      }
    } else {
      std::vector<Vector7d> object_poses_all;
      // object_poses_all.push_back(object_trajectory[t].m_pose);
      for (int kk = t + 1; kk <= t_next; ++kk) {
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

    for (auto x : object_poses) {
      VectorXd mnp_config_world(6 * task->number_of_robot_contacts);
      for (int n_pt = 0; n_pt < task->number_of_robot_contacts; ++n_pt) {
        if (std::isnan(mnp_config[6 * n_pt])) {
          mnp_config_world.segment(6 * n_pt, 6) =
              mnp_config.segment(6 * n_pt, 6);
          mnp_config_world(6 * n_pt) = -7777;
          mnp_config_world(6 * n_pt + 1) = -7777;
          mnp_config_world(6 * n_pt + 2) = -7777;
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

      VectorXd output_row(4 + 6 * task->number_of_robot_contacts + 7);

      output_row.segment(0, 4) =
          hand->getHandFrameConfig(mnp_config, x);
      output_row.segment(4, 6 * task->number_of_robot_contacts) =
          mnp_config_world;
      output_row.segment(4 + 6 * task->number_of_robot_contacts, 7) = x;

      output.push_back(output_row);
    }
  }

  MatrixXd output_mat(output.size(),4 + 6 * task->number_of_robot_contacts + 7);
  for (int i = 0; i < output.size(); ++i) {
    output_mat.row(i) = output[i];
  }
  return output_mat;
}

void visualize_output_file(std::shared_ptr<WorldTemplate> world,
                           std::string file_name) {
  MatrixXd data = openData(file_name);
  int n_data = data.rows();
  int n_pts = (data.cols() - 7) / 6;
  std::vector<Vector7d> object_traj;
  std::vector<VectorXd> mnp_traj;
  for (int i = 0; i < n_data; ++i) {
    // if all elements are zero, skip this row
    if (data.row(i).sum() == 0) {
      continue;
    }
    VectorXd mnp_config_world = data.row(i).segment(4, 6 * n_pts);
    VectorXd mnp_config(6 * n_pts);
    Vector7d object_pose = data.row(i).segment(4 + 6 * n_pts, 7);

    Matrix3d R = quat2SO3(object_pose(6), object_pose(3), object_pose(4),
                          object_pose(5));
    Matrix3d R_inv = R.transpose();
    Vector3d t = object_pose.segment(0, 3);
    for (int j = 0; j < n_pts; ++j) {
      if (std::isnan(mnp_config_world[6 * j])) {
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

const CMGTASK::State2::Action CMGTASK::State2::no_action =
    CMGTASK::State2::Action(-1, -1);
const CMGTASK::State::Action CMGTASK::State::no_action = -1;

void setup(std::shared_ptr<CMGTASK> task) {
  // create world, create environment, an object sliding on the table

  std::string para_path =
      std::string(SRC_DIR) + "/data/ddhand/simple_setup.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  double box_lx = config["box_shape"]["lx"].as<double>();
  double box_ly = config["box_shape"]["ly"].as<double>();
  double box_lz = config["box_shape"]["lz"].as<double>();

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", Vector3d(box_lx, box_ly, box_lz),
                    Vector3d(0.7, 0.3, 0.3), 0.45);

  world->addObject(object);

  SkeletonPtr env1 =
      createFixedBox("ground", Vector3d(0.5, 0.5, 0.2), Vector3d(0, 0, -0.1));

  world->addEnvironmentComponent(env1);

  double hand_x = config["hand_position"]["x"].as<double>();
  double hand_y = config["hand_position"]["y"].as<double>();
  double hand_z = config["hand_position"]["z"].as<double>();

  DartDDHand *rpt = new DartDDHand(L_FINGER);
  Vector7d hand_pos;
  hand_pos << hand_x, hand_y, hand_z, 0, 0, 0, 1;

  rpt->setHandFrameTransform(hand_pos);

  world->addRobot(rpt);

  // set the task parameters, start, goal, object inertial, etc....

  Vector7d x_start;
  Vector7d x_goal;
  x_start << config["start_pose"]["x"].as<double>(),
      config["start_pose"]["y"].as<double>(),
      config["start_pose"]["z"].as<double>(),
      config["start_pose"]["qx"].as<double>(),
      config["start_pose"]["qy"].as<double>(),
      config["start_pose"]["qz"].as<double>(),
      config["start_pose"]["qw"].as<double>();
  x_goal << config["goal_pose"]["x"].as<double>(),
      config["goal_pose"]["y"].as<double>(),
      config["goal_pose"]["z"].as<double>(),
      config["goal_pose"]["qx"].as<double>(),
      config["goal_pose"]["qy"].as<double>(),
      config["goal_pose"]["qz"].as<double>(),
      config["goal_pose"]["qw"].as<double>();

  double goal_thr = config["rrt_options"]["goal_thr"].as<double>();

  double wa = 1;
  double wt = 0.3;

  double mu_env = config["mu_env"].as<double>();
  double mu_mnp = config["mu_mnp"].as<double>();

  double charac_len = 10;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  CMGTASK::SearchOptions rrt_options;

  rrt_options.x_ub << config["start_pose"]["x"].as<double>() + box_lx / 2,
      config["start_pose"]["y"].as<double>() + box_ly / 2,
      config["start_pose"]["z"].as<double>() + box_lz / 2;
  rrt_options.x_lb << config["start_pose"]["x"].as<double>() - box_lx / 2,
      config["start_pose"]["y"].as<double>() - box_ly / 2,
      config["start_pose"]["z"].as<double>() - box_lz / 2;

  rrt_options.eps_trans = config["rrt_options"]["eps_trans"].as<double>();
  rrt_options.eps_angle =
      (3.14 / 180.0) * config["rrt_options"]["eps_angle_deg"].as<double>();
  rrt_options.max_samples = config["rrt_options"]["max_samples"].as<int>();

  rrt_options.goal_biased_prob =
      config["rrt_options"]["goal_biased_prob"].as<double>();

  bool is_refine = false;
  double refine_dist = 0.1;

  // read surface point, add robot contacts
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) +
                  "/data/ddhand/cube_surface_contacts.csv");
  aria::csv::CsvParser parser(f);

  for (auto &row : parser) {
    int n_cols = row.size();
    assert(n_cols == 6);

    Vector6d v;
    for (int j = 0; j < 6; ++j) {
      v(j) = std::stod(row[j]);
    }

    Vector3d pos;
    pos << v(0) * box_lx / 2, v(1) * box_ly / 2, v(2) * box_lz / 2;
    // if (abs(v[0]) > 0.8 || abs(v[1] > 0.8)) {
    //   continue;
    // }
    ContactPoint p(pos, -v.tail(3));
    surface_pts.push_back(p);
  }
  std::cout << "surface pts: " << surface_pts.size() << std::endl;
  // pass the world and task parameters to the task through task->initialize

  task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                   mu_mnp, oi, f_g, world, 2, CMG_QUASISTATIC, surface_pts,
                   rrt_options, is_refine, refine_dist);
  // task->grasp_measure_charac_length =
  //     config["grasp_measure_charac_length"].as<double>();
  // task->grasp_measure_charac_length = -1.0;
}

int main(int argc, char *argv[]) {
  std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

  std::string para_path = std::string(SRC_DIR) + "/data/ddhand/config.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  std::string output_file;

  // std::string task_name = config["task"].as<std::string>();
  // output_file = task_name + "_ouput.csv";
  output_file = "output.csv";

  std::string setup_path = std::string(SRC_DIR) + "/data/ddhand/simple_setup.yaml";
  setup(task);
  // change the above code using case statement

  std::string output_file_path =
      std::string(SRC_DIR) + "/data/ddhand/plan_results/" + output_file;

  std::string visualize_option = config["visualize_option"].as<std::string>();

  int random_seed = config["random_seed"].as<int>();

  if (visualize_option == "csv") {
    visualize_output_file(task->m_world, output_file_path);
    task->m_world->startWindow(&argc, argv);
    return 0;
  }

  if (visualize_option == "setup") {
    VisualizeSG(task->m_world, task->start_object_pose, task->goal_object_pose);
    task->m_world->startWindow(&argc, argv);
    return 0;
  }

  CMGTASK::State start_state = task->get_start_state();

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2,
                  CMGTASK>::HierarchicalComputeOptions compute_options;

  compute_options.l1_1st.max_iterations =
      config["l1_1st_max_iterations"].as<int>();
  compute_options.l1.max_iterations = config["l1_max_iterations"].as<int>();
  compute_options.l2_1st.max_iterations =
      config["l2_1st_max_iterations"].as<int>();
  compute_options.l2.max_iterations = config["l2_max_iterations"].as<int>();
  compute_options.final_l2_1st.max_iterations =
      config["final_l2_1st_max_iterations"].as<int>();
  compute_options.final_l2.max_iterations =
      config["final_l2_max_iterations"].as<int>();

  compute_options.l1.max_time = config["max_time"].as<double>();

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2, CMGTASK> tree(
      task, start_state, compute_options);

  HMP::Node<CMGTASK::State> *current_node = tree.search_tree();

  std::vector<CMGTASK::State> object_trajectory;
  std::vector<CMGTASK::State2> action_trajectory;
  tree.get_final_results(current_node, &object_trajectory, &action_trajectory);

  std::vector<Vector7d> object_traj;
  std::vector<VectorXd> mnp_traj;

  std::cout << "Best value " << current_node->m_value << std::endl;

  for (auto &action : action_trajectory) {
    std::cout << "Timestep " << action.timestep << std::endl;
    std::cout
        << "Pose "
        << task->saved_object_trajectory[action.timestep].m_pose.transpose()
        << std::endl;
    std::cout << "Fingers from idx " << action.finger_index << ": ";
    for (int jj : task->get_finger_locations(action.finger_index)) {
      std::cout << jj << " ";
    }
    std::cout << std::endl;
  }

  VisualizeStateTraj(task->m_world, task, object_trajectory, action_trajectory);

  // std::remove(output_file_path.c_str());
  MatrixXd output_mat =
      get_ddhand_output(object_trajectory, action_trajectory, task, setup_path);
  saveData(output_file_path, output_mat);

  std::cout << "Total level 1 tree nodes " << tree.count_total_nodes()
            << std::endl;

  std::cout << "Total shared rrt nodes " << tree.m_task->total_rrt_nodes()
            << std::endl;

  if (visualize_option == "results") {
    task->m_world->startWindow(&argc, argv);
  }
}

#include <cstdio>
#include <yaml-cpp/yaml.h>

#include "../search/level1.h"

#include "../tasks/inhand_task.h"

#include "../mechanics/utilities/utilities.h"

#include "../mechanics/manipulators/DartDeltaManipulator.h"
#include "../mechanics/utilities/parser.hpp"
#include "../mechanics/worlds/DartWorld.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif

#include "../mechanics/utilities/io.h"
#include "visualization.h"

const InhandTASK::State2::Action InhandTASK::State2::no_action =
    InhandTASK::State2::Action(-1, -1);
const InhandTASK::State::Action InhandTASK::State::no_action = -1;

std::vector<Vector3d> read_delta_locations(const YAML::Node &config,
                                           int n_pts) {
  std::vector<Vector3d> delta_locations;
  if (n_pts >= 1) {
    std::vector<double> loc =
        config["delta_locations"]["robot_1"].as<std::vector<double>>();
    delta_locations.push_back(Vector3d(loc[0], loc[1], loc[2]));
  }
  if (n_pts >= 2) {
    std::vector<double> loc =
        config["delta_locations"]["robot_2"].as<std::vector<double>>();
    delta_locations.push_back(Vector3d(loc[0], loc[1], loc[2]));
  }
  if (n_pts >= 3) {
    std::vector<double> loc =
        config["delta_locations"]["robot_3"].as<std::vector<double>>();
    delta_locations.push_back(Vector3d(loc[0], loc[1], loc[2]));
  }
  if (n_pts >= 4) {
    std::vector<double> loc =
        config["delta_locations"]["robot_4"].as<std::vector<double>>();
    delta_locations.push_back(Vector3d(loc[0], loc[1], loc[2]));
  }
  if (n_pts >= 5) {
    std::vector<double> loc =
        config["delta_locations"]["robot_5"].as<std::vector<double>>();
    delta_locations.push_back(Vector3d(loc[0], loc[1], loc[2]));
  }
  if (n_pts >= 6) {

    std::cout << "Don't support more than 5 robots" << std::endl;
    exit(0);
  }
  Vector3d center;
  center.setZero();
  for (int i = 0; i < n_pts; i++) {
    center += delta_locations[i];
  }
  center /= n_pts;
  std::cout << "delta center: " << center.transpose() << std::endl;
  return delta_locations;
}

bool surface_contact_filter(double box_lx, double box_ly, double box_lz,
                            double x, double y, double z, double nx, double ny,
                            double nz, double finger_radius) {
  double x_min = -box_lx / 2 + finger_radius;
  double x_max = box_lx / 2 - finger_radius;
  double y_min = -box_ly / 2 + finger_radius;
  double y_max = box_ly / 2 - finger_radius;
  double z_min = -box_lz / 2 + finger_radius;
  double z_max = box_lz / 2 - finger_radius;

  // no contact on the top and bottom
  if (nz > 0.7 || nz < -0.7) {
    return false;
  }

  if (nx > 0.7 || nx < -0.7) {
    if (y < y_min || y > y_max || z < z_min || z > z_max) {
      return false;
    }
  }
  if (ny > 0.7 || ny < -0.7) {
    if (x < x_min || x > x_max || z < z_min || z > z_max) {
      return false;
    }
  }
  return true;
}

bool table_surface_contact_filter(double box_lx, double box_ly, double box_lz,
                                  double x, double y, double z, double nx,
                                  double ny, double nz, double finger_radius) {
  double x_min = -box_lx / 2 + finger_radius;
  double x_max = box_lx / 2 - finger_radius;
  double y_min = -box_ly / 2 + finger_radius;
  double y_max = box_ly / 2 - finger_radius;
  double z_min = -box_lz / 2 + finger_radius;
  double z_max = box_lz / 2 - finger_radius;

  if (nz > 0.7 || nz < -0.7) {
    if (x < x_min || x > x_max || y < y_min || y > y_max) {
      return false;
    }
  }

  if (nx > 0.7 || nx < -0.7) {
    if (y < y_min || y > y_max || z < z_min || z > z_max) {
      return false;
    }
  }
  if (ny > 0.7 || ny < -0.7) {
    // return false;
    if (x < x_min || x > x_max || z < z_min || z > z_max) {
      return false;
    }
  }

  return true;
}

void inhand_cube(std::shared_ptr<InhandTASK> task) {
  // create world, create environment, an object sliding on the table

  std::string para_path =
      std::string(SRC_DIR) + "/data/delta_array/inhand_cube_setup.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  std::vector<Vector3d> delta_locations = read_delta_locations(config, 5);

  double box_lx = config["box_shape"]["lx"].as<double>();
  double box_ly = config["box_shape"]["ly"].as<double>();
  double box_lz = config["box_shape"]["lz"].as<double>();

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", Vector3d(box_lx, box_ly, box_lz),
                    Vector3d(0.7, 0.3, 0.3), 0.45);

  world->addObject(object);

  SkeletonPtr env1 =
      createFixedBox("palm", Vector3d(3, 3, 0.2), Vector3d(0, 0, -100),
                     Vector3d(0.9, 0.9, 0.9), 0.01);

  world->addEnvironmentComponent(env1);

  // delta robot
  double delta_ws_r = 2.5;
  double delta_ws_h = 6;

  int n_robot_contacts = delta_locations.size();
  double finger_radius = config["finger_radius"].as<double>();
  DartDeltaManipulator *rpt = new DartDeltaManipulator(
      n_robot_contacts, finger_radius, delta_ws_r, delta_ws_h, delta_locations);

  rpt->is_patch_contact = true;
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

  long int start_finger_idx = -1;
  long int goal_finger_idx = -1;

  double goal_thr = 3.14 * 10 / 180;

  double wa = 1;
  double wt = 1;

  double mu_env = 0.2;
  double mu_mnp = config["friction_coefficient"].as<double>();

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  InhandTASK::SearchOptions rrt_options;

  rrt_options.x_ub << config["start_pose"]["x"].as<double>() + box_lx / 2,
      config["start_pose"]["y"].as<double>() + box_ly / 2,
      config["start_pose"]["z"].as<double>() + box_lz / 2;
  rrt_options.x_lb << config["start_pose"]["x"].as<double>() - box_lx / 2,
      config["start_pose"]["y"].as<double>() - box_ly / 2,
      config["start_pose"]["z"].as<double>() - box_lz / 2;

  rrt_options.eps_trans = 0.2;
  rrt_options.eps_angle = 3.14 * 10 / 180;
  rrt_options.max_samples = 50;

  rrt_options.goal_biased_prob = 0.7;

  bool is_refine = false;
  double refine_dist = 0.1;

  // read surface point, add robot contacts
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) +
                  "/data/delta_array/cube_surface_contacts.csv");
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
    if (surface_contact_filter(box_lx, box_ly, box_lz, pos[0], pos[1], pos[2],
                               -v[3], -v[4], -v[5], finger_radius / 2)) {
      ContactPoint p(pos, -v.tail(3));
      surface_pts.push_back(p);
    }
  }
  std::cout << "surface pts: " << surface_pts.size() << std::endl;
  // pass the world and task parameters to the task through task->initialize

  task->initialize(x_start, x_goal, start_finger_idx, goal_finger_idx, goal_thr,
                   wa, wt, charac_len, mu_env, mu_mnp, f_g, world,
                   n_robot_contacts, surface_pts, rrt_options, is_refine,
                   refine_dist);
  task->grasp_measure_charac_length = config["grasp_measure_charac_length"]
                                          .as<double>();
  // task->grasp_measure_charac_length = -1.0;
  // VisualizeSG(task->m_world, x_start, x_goal);
}

void table_cube(std::shared_ptr<InhandTASK> task) {
  // create world, create environment, an object sliding on the table

  std::string para_path =
      std::string(SRC_DIR) + "/data/delta_array/table_cube_setup.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  int n_robot_contacts = config["n_robot_contacts"].as<int>();

  std::vector<Vector3d> delta_locations = read_delta_locations(config, n_robot_contacts);
  Vector3d center;
  center.setZero();
  for (auto &p : delta_locations) {
    center += p;
  }
  center /= double(delta_locations.size());

  double box_lx = config["box_shape"]["lx"].as<double>();
  double box_ly = config["box_shape"]["ly"].as<double>();
  double box_lz = config["box_shape"]["lz"].as<double>();

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", Vector3d(box_lx, box_ly, box_lz),
                    Vector3d(0.7, 0.3, 0.3), 0.45);

  world->addObject(object);

  SkeletonPtr env1 = createFixedBox(
      "palm", Vector3d(50, 50, 0.2),
      Vector3d(center[0], center[1], config["table_height"].as<double>() - 0.1),
      Vector3d(0.9, 0.9, 0.6), 0.6);

  world->addEnvironmentComponent(env1);

  // delta robot
  double delta_ws_r = 2.5;
  double delta_ws_h = 10;

  // int n_robot_contacts = delta_locations.size();
  double finger_radius = config["finger_radius"].as<double>();
  DartDeltaManipulator *rpt = new DartDeltaManipulator(
      n_robot_contacts, finger_radius, delta_ws_r, delta_ws_h, delta_locations);

  rpt->is_patch_contact = true;
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

  long int start_finger_idx = -1;
  long int goal_finger_idx = -1;

  double goal_thr = 3.14 * 10 / 180;

  double wa = 1;
  double wt = 1;

  double mu_env = 0.1;
  double mu_mnp = config["friction_coefficient"].as<double>();

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  InhandTASK::SearchOptions rrt_options;

  rrt_options.x_ub << config["start_pose"]["x"].as<double>() + box_lx / 2,
      config["start_pose"]["y"].as<double>() + box_ly / 2,
      config["start_pose"]["z"].as<double>() + box_lz / 2;
  rrt_options.x_lb << config["start_pose"]["x"].as<double>() - box_lx / 2,
      config["start_pose"]["y"].as<double>() - box_ly / 2,
      config["start_pose"]["z"].as<double>() - box_lz / 2;

  rrt_options.eps_trans = 0.5;
  rrt_options.eps_angle = 3.14 * 50 / 180;
  rrt_options.max_samples = 30;

  rrt_options.goal_biased_prob = 0.7;

  bool is_refine = false;
  double refine_dist = 0.1;

  // read surface point, add robot contacts
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) +
                  "/data/delta_array/cube_surface_contacts.csv");
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
    if (table_surface_contact_filter(box_lx, box_ly, box_lz, pos[0], pos[1],
                                     pos[2], -v[3], -v[4], -v[5],
                                     finger_radius)) {
    //   if (-v[5] < -0.7) {
    //     continue;
    //   }
      ContactPoint p(pos, -v.tail(3));
      surface_pts.push_back(p);
    }
  }
  std::cout << "surface pts: " << surface_pts.size() << std::endl;
  // pass the world and task parameters to the task through task->initialize

  task->initialize(x_start, x_goal, start_finger_idx, goal_finger_idx, goal_thr,
                   wa, wt, charac_len, mu_env, mu_mnp, f_g, world,
                   n_robot_contacts, surface_pts, rrt_options, is_refine,
                   refine_dist);

  task->grasp_measure_charac_length = -1;
}

void inhand_mesh(std::shared_ptr<InhandTASK> task) {
  // create world, create environment, an object sliding on the table

  std::string para_path =
      std::string(SRC_DIR) + "/data/delta_array/mesh_setup.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  std::vector<Vector3d> delta_locations = read_delta_locations(config, 4);

  std::string object_name = config["object"].as<std::string>();
  double object_scale = config["object_scale"].as<double>();

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object = createFreeObjectfromMesh(
      object_name,
      std::string(SRC_DIR) + "/" + config["file_path"].as<std::string>() + "/" +
          object_name + ".stl",
      Vector3d(object_scale, object_scale, object_scale));

  world->addObject(object);

  SkeletonPtr env1 =
      createFixedBox("palm", Vector3d(3, 3, 0.2), Vector3d(0, 0, -100),
                     Vector3d(0.9, 0.9, 0.9), 0.1);

  world->addEnvironmentComponent(env1);

  // delta robot
  double delta_ws_r = 2.5;
  double delta_ws_h = 6;

  int n_robot_contacts = delta_locations.size();
  double finger_radius = config["finger_radius"].as<double>();
  DartDeltaManipulator *rpt = new DartDeltaManipulator(
      n_robot_contacts, finger_radius, delta_ws_r, delta_ws_h, delta_locations);

  rpt->is_patch_contact = true;
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

  long int start_finger_idx = -1;
  long int goal_finger_idx = -1;

  double goal_thr = 3.14 * 10 / 180;

  double wa = 1;
  double wt = 1;

  double mu_env = 0.2;
  double mu_mnp = config["friction_coefficient"].as<double>();

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  InhandTASK::SearchOptions rrt_options;

  rrt_options.x_ub << config["start_pose"]["x"].as<double>() + object_scale / 2,
      config["start_pose"]["y"].as<double>() + object_scale / 2,
      config["start_pose"]["z"].as<double>() + object_scale / 2;
  rrt_options.x_lb << config["start_pose"]["x"].as<double>() - object_scale / 2,
      config["start_pose"]["y"].as<double>() - object_scale / 2,
      config["start_pose"]["z"].as<double>() - object_scale / 2;

  rrt_options.eps_trans = 0.2;
  rrt_options.eps_angle = 3.14 * 20 / 180;
  rrt_options.max_samples = 50;

  rrt_options.goal_biased_prob = 0.7;

  bool is_refine = false;
  double refine_dist = 0.1;

  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) + "/" +
                  config["file_path"].as<std::string>() + "/" + object_name +
                  ".csv");
  aria::csv::CsvParser parser(f);

  for (auto &row : parser) {
    int n_cols = row.size();
    assert(n_cols == 6);

    Vector6d v;
    for (int j = 0; j < 6; ++j) {
      v(j) = std::stod(row[j]);
    }
    Vector3d pos;
    pos << v(0) * object_scale, v(1) * object_scale, v(2) * object_scale;
    ContactPoint p(pos, v.tail(3));
    surface_pts.push_back(p);
  }

  std::cout << "surface pts: " << surface_pts.size() << std::endl;
  // pass the world and task parameters to the task through task->initialize

  task->initialize(x_start, x_goal, start_finger_idx, goal_finger_idx, goal_thr,
                   wa, wt, charac_len, mu_env, mu_mnp, f_g, world,
                   n_robot_contacts, surface_pts, rrt_options, is_refine,
                   refine_dist);

  // VisualizeSG(task->m_world, x_start, x_goal);
}

void planar_manipulation(std::shared_ptr<InhandTASK> task) {
  // create world, create environment, an object sliding on the table

  std::string para_path =
      std::string(SRC_DIR) + "/data/delta_array/planar_manipulation_setup.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  std::vector<Vector3d> delta_locations = read_delta_locations(config, 4);

  Vector3d center;
  center.setZero();
  for (auto &p : delta_locations) {
    center += p;
  }
  center /= double(delta_locations.size());

  double box_lx = config["box_shape"]["lx"].as<double>();
  double box_ly = config["box_shape"]["ly"].as<double>();
  double box_lz = config["box_shape"]["lz"].as<double>();

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", Vector3d(box_lx, box_ly, box_lz),
                    Vector3d(0.7, 0.3, 0.3), 0.45);

  world->addObject(object);

  SkeletonPtr env1 = createFixedBox("palm", Vector3d(50, 50, 0.2),
                                    Vector3d(center[0], center[1], -0.1),
                                    Vector3d(0.9, 0.9, 0.6), 0.6);

  world->addEnvironmentComponent(env1);

  // delta robot

  double delta_ws_r = config["delta_ws_r"].as<double>();
  double delta_ws_h = config["delta_ws_h"].as<double>();

  int n_robot_contacts = delta_locations.size();
  double finger_radius = config["finger_radius"].as<double>();
  DartDeltaManipulator *rpt = new DartDeltaManipulator(
      n_robot_contacts, finger_radius, delta_ws_r, delta_ws_h, delta_locations);

  rpt->is_patch_contact = true;
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

  long int start_finger_idx = -1;
  long int goal_finger_idx = -1;

  double goal_thr = config["rrt_options"]["goal_thr"].as<double>();

  double wa = 1;
  double wt = 1;

  double mu_env = 0.2;
  double mu_mnp = config["friction_coefficient"].as<double>();

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0, 0, -0.1, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  InhandTASK::SearchOptions rrt_options;

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

  bool is_refine = config["is_refine"].as<bool>();
  double refine_dist = config["refine_dist"].as<double>();

  // read surface point, add robot contacts
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) +
                  "/data/delta_array/cube_surface_contacts.csv");
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
    if (surface_contact_filter(box_lx, box_ly, box_lz, pos[0], pos[1], pos[2],
                               -v[3], -v[4], -v[5], finger_radius)) {
      ContactPoint p(pos, -v.tail(3));
      surface_pts.push_back(p);
    }
  }
  std::cout << "surface pts: " << surface_pts.size() << std::endl;

  task->initialize(x_start, x_goal, start_finger_idx, goal_finger_idx, goal_thr,
                   wa, wt, charac_len, mu_env, mu_mnp, f_g, world,
                   n_robot_contacts, surface_pts, rrt_options, is_refine,
                   refine_dist);
  task->grasp_measure_charac_length = config["grasp_measure_charac_length"]
                                          .as<double>();
}

int main(int argc, char *argv[]) {
  std::shared_ptr<InhandTASK> task = std::make_shared<InhandTASK>();

  std::string para_path =
      std::string(SRC_DIR) + "/data/delta_array/config.yaml";
  YAML::Node config = YAML::LoadFile(para_path);

  std::string output_file;

  std::string task_name = config["task"].as<std::string>();
  output_file = task_name + "_ouput.csv";

  if (task_name == "inhand_cube") {
    inhand_cube(task);
  } else if (task_name == "inhand_mesh") {
    inhand_mesh(task);
  } else if (task_name == "table_cube") {
    table_cube(task);
  } else if (task_name == "planar_manipulation") {
    planar_manipulation(task);
  } else {
    std::cout << "Invalid task name" << std::endl;
    return 0;
  }
  // change the above code using case statement

  std::string output_file_path =
      std::string(SRC_DIR) + "/data/delta_array/plan_results/" + output_file;

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

  InhandTASK::State start_state = task->get_start_state();

  HMP::Level1Tree<InhandTASK::State, InhandTASK::State2,
                  InhandTASK>::HierarchicalComputeOptions compute_options;

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

  HMP::Level1Tree<InhandTASK::State, InhandTASK::State2, InhandTASK> tree(
      task, start_state, compute_options);

  HMP::Node<InhandTASK::State> *current_node = tree.search_tree();

  std::vector<InhandTASK::State> object_trajectory;
  std::vector<InhandTASK::State2> action_trajectory;
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

  VisualizeStateTrajectory(task->m_world, task, object_trajectory,
                           action_trajectory);

  std::remove(output_file_path.c_str());
  double outward_radius = config["outward_radius"].as<double>();
  MatrixXd output_mat = get_output(object_trajectory, action_trajectory, task, outward_radius);
  saveData(output_file_path, output_mat);

  std::cout << "Total level 1 tree nodes " << tree.count_total_nodes()
            << std::endl;

  std::cout << "Total shared rrt nodes " << tree.m_task->total_rrt_nodes()
            << std::endl;

  if (visualize_option == "results") {
    task->m_world->startWindow(&argc, argv);
  }
}

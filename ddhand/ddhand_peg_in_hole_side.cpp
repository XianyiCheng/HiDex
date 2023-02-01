#include "ddhand_setup.h"

void setup(std::shared_ptr<CMGTASK> task, const std::string &setup_path) {

  YAML::Node config = YAML::LoadFile(setup_path);

  double box_lx = config["box_shape"]["lx"].as<double>();
  double box_ly = config["box_shape"]["ly"].as<double>();
  double box_lz = config["box_shape"]["lz"].as<double>();

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", Vector3d(box_lx, box_ly, box_lz),
                    Vector3d(0.7, 0.3, 0.3), 0.45);

  world->addObject(object);

  double gap = config["gap"].as<double>(); // > 0.005
  double wall_height = config["wall_height"].as<double>();
  double wall_width = config["wall_width"].as<double>();

  SkeletonPtr wall1 = createFixedBox(
      "wall1", Vector3d(wall_width, box_ly + wall_width * 2, wall_height),
      Vector3d(-(box_lx / 2 + gap + wall_width / 2), 0, wall_height / 2));

  SkeletonPtr wall2 = createFixedBox(
      "wall2", Vector3d(wall_width, box_ly + wall_width * 2, wall_height),
      Vector3d(box_lx / 2 + gap + wall_width / 2, 0, wall_height / 2));

  SkeletonPtr wall3 = createFixedBox(
      "wall3", Vector3d(box_lx + wall_width * 2, wall_width, wall_height),
      Vector3d(0, box_ly / 2 + gap + wall_width / 2, wall_height / 2));

  SkeletonPtr wall4 = createFixedBox(
      "wall4", Vector3d(box_lx + wall_width * 2, wall_width, wall_height),
      Vector3d(0, -(box_ly / 2 + gap + wall_width / 2), wall_height / 2));

  SkeletonPtr ground = createFixedBox("ground", Vector3d(20, 20, wall_height),
                                      Vector3d(0, 0, 1e-5 - wall_height / 2));

  world->addEnvironmentComponent(wall1);
  world->addEnvironmentComponent(wall2);
  world->addEnvironmentComponent(wall3);
  world->addEnvironmentComponent(wall4);
  world->addEnvironmentComponent(ground);

  double hand_x = config["hand_position"]["x"].as<double>();
  double hand_y = config["hand_position"]["y"].as<double>();
  double hand_z = config["hand_position"]["z"].as<double>();

  DartDDHandScalable *rpt =
      new DartDDHandScalable(L_FINGER, config["scale_to_real"].as<double>());
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

  double wa = config["wa"].as<double>();
  double wt = config["wt"].as<double>();

  double mu_env = config["mu_env"].as<double>();
  double mu_mnp = config["mu_mnp"].as<double>();

  double charac_len = 1;

  Vector6d f_g;
  f_g << 0.1, 0, 0, 0, 0, 0;

  Matrix6d oi;
  oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
      0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

  // search options

  CMGTASK::SearchOptions rrt_options;

  rrt_options.x_ub << max(x_start[0], x_goal[0]) + box_lx / 2,
      max(x_start[1], x_goal[1]) + box_ly / 2,
      max(x_start[2], x_goal[2]) + box_lz / 2;
  rrt_options.x_lb << min(x_start[0], x_goal[0]) - box_lx / 2,
      min(x_start[1], x_goal[1]) - box_ly / 2,
      min(x_start[2], x_goal[2]) - box_lz / 2;

  rrt_options.eps_trans = config["rrt_options"]["eps_trans"].as<double>();
  rrt_options.eps_angle =
      (3.14 / 180.0) * config["rrt_options"]["eps_angle_deg"].as<double>();
  rrt_options.max_samples = config["rrt_options"]["max_samples"].as<int>();

  rrt_options.goal_biased_prob =
      config["rrt_options"]["goal_biased_prob"].as<double>();
  rrt_options.sample_rotation_axis << 0, 1, 0;

  bool is_refine = config["is_refine"].as<bool>();
  double refine_dist = config["refine_dist"].as<double>();

  // read surface point, add robot contacts
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) + "/data/ddhand/" +
                  config["surface_contact_file"].as<std::string>());
  aria::csv::CsvParser parser(f);

  {
    std::vector<double> loc = config["initial_contact_locations"]["finger_1"]
                                  .as<std::vector<double>>();
    ContactPoint p(
        Vector3d(box_lx / 2 * loc[0], box_ly / 2 * loc[1], box_lz / 2 * loc[2]),
        Vector3d(loc[3], loc[4], loc[5]));
    surface_pts.push_back(p);
  }

  {
    std::vector<double> loc = config["initial_contact_locations"]["finger_2"]
                                  .as<std::vector<double>>();
    ContactPoint p(
        Vector3d(box_lx / 2 * loc[0], box_ly / 2 * loc[1], box_lz / 2 * loc[2]),
        Vector3d(loc[3], loc[4], loc[5]));
    surface_pts.push_back(p);
  }

  for (auto &row : parser) {
    int n_cols = row.size();
    assert(n_cols == 6);

    Vector6d v;
    for (int j = 0; j < 6; ++j) {
      v(j) = std::stod(row[j]);
    }
    // if (v(2) < 0.75) {
    //   continue;
    // }
    Vector3d pos;
    pos << v(0) * box_lx / 2, v(1) * box_ly / 2, v(2) * box_lz / 2;
    ContactPoint p(pos, v.tail(3));
    surface_pts.push_back(p);
  }
  std::cout << "surface pts: " << surface_pts.size() << std::endl;
  task->start_finger_idx = 2 * surface_pts.size() + 1;
  // pass the world and task parameters to the task through task->initialize

  task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                   mu_mnp, oi, f_g, world, 2, CMG_QUASISTATIC, surface_pts,
                   rrt_options, is_refine, refine_dist);
  // task->grasp_measure_charac_length =
  //     config["grasp_measure_charac_length"].as<double>();
  task->grasp_measure_charac_length = -1.0;
}

int main(int argc, char *argv[]) {

  std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();
  std::string para_path =
      std::string(SRC_DIR) + "/data/ddhand/peg_in_hole_side_setup.yaml";

  setup(task, para_path);
  run(task, para_path);
}

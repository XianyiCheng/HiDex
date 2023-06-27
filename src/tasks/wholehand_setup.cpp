#include "wholehand_setup.h"
void sample_surface_contacts(SkeletonPtr object, std::vector<ContactPoint> *pts, int num_samples, double scale,
                             std::vector<Vector3d> disabled_normal_directions)
{
  BodyNodePtr mbn = object->getBodyNode(0);
  Isometry3d mtf = mbn->getWorldTransform();
  ShapePtr ms = mbn->getShapeNodes().front()->getShape();
  MeshShape *meshShape =
      dynamic_cast<MeshShape *>(ms.get());

  int num_vertices = meshShape->getMesh()->mMeshes[0]->mNumVertices - 1;
  while (num_samples > 0)
  {
    Vector3d p;
    Vector3d n;

    int index = randi(num_vertices);
    aiVector3D point = meshShape->getMesh()->mMeshes[0]->mVertices[index];
    aiVector3D normal = meshShape->getMesh()->mMeshes[0]->mNormals[index];
    p << scale*point[0], scale*point[1], scale*point[2];
    n << -normal[0], -normal[1], -normal[2];

    bool is_disabled = false;
    for (auto &dir : disabled_normal_directions)
    {
      double d = n.transpose() * dir;
      if (d > 0.9)
      {
        is_disabled = true;
        break;
      }
    }
    if (is_disabled)
    {
      continue; // skip this point
    }
    ContactPoint cp(p, n);
    pts->push_back(cp);
    num_samples--;
  }
}

void load_surface_contacts(const std::string &file_name,
                           std::vector<ContactPoint> *pts, const Vector3d &scale,
                           std::vector<Vector3d> disabled_normal_directions,
                           bool negate_normal = false)
{
  std::ifstream f(file_name);
  aria::csv::CsvParser parser(f);

  if (pts->size() > 0)
  {
    pts->clear();
  }

  double normal_dir = 1.0;
  if (negate_normal)
  {
    normal_dir = -1.0;
  }

  for (auto &row : parser)
  {
    if (row.size() != 6)
    {
      continue;
    }
    Vector6d v;
    for (int j = 0; j < 6; ++j)
    {
      v(j) = std::stod(row[j]);
    }

    Vector3d normal = normal_dir * v.tail(3);
    bool is_disabled = false;
    for (auto &dir : disabled_normal_directions)
    {
      double d = normal.transpose() * dir;
      if (d > 0.9)
      {
        is_disabled = true;
        break;
      }
    }
    if (is_disabled)
    {
      continue;
    }

    Vector3d pos;
    pos << v(0) * scale[0], v(1) * scale[1], v(2) * scale[2];
    ContactPoint p(pos, normal_dir * v.tail(3));
    pts->push_back(p);
  }
}

void load_task(std::shared_ptr<WholeHandTASK> task, const YAML::Node &config, const std::string &task_folder)
{
  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  // ---- Load Object ----
  std::cout << "Loading object" << std::endl;
  std::vector<ContactPoint> surface_pts;
  SkeletonPtr object;
  std::string object_type;
  Vector3d scale;

  if (config["box_object"])
  {
    object_type = "box_object";
    std::vector<double> box_l =
        config["box_object"]["shape"].as<std::vector<double>>();
    object =
        createFreeBox("object", Vector3d(box_l[0], box_l[1], box_l[2]),
                      Vector3d(0.7, 0.3, 0.3), 0.45);
    // scale << box_l[0] / 2, box_l[1] / 2, box_l[2] / 2;
    scale << box_l[0], box_l[1], box_l[2];
  }
  else if (config["ellipsoid_object"])
  {
    object_type = "ellipsoid_object";
    std::vector<double> ellipsoid_l =
        config["ellipsoid_object"]["semi_axis"].as<std::vector<double>>();
    object = createFreeEllipsoid(
        "object", Vector3d(ellipsoid_l[0], ellipsoid_l[1], ellipsoid_l[2]));
    scale << ellipsoid_l[0], ellipsoid_l[1], ellipsoid_l[2];
  }
  else if (config["cylinder_object"])
  {
    object_type = "cylinder_object";
    double c_radius = config["cylinder_object"]["radius"].as<double>();
    double c_height = config["cylinder_object"]["height"].as<double>();
    object = createFreeCylinder("object", c_radius, c_height);
    scale << c_radius, c_radius, c_height;
  }
  else if (config["mesh_object"])
  {
    object_type = "mesh_object";
    double scale_l = config["mesh_object"]["scale"].as<double>();
    scale << scale_l, scale_l, scale_l;
    object = createFreeObjectfromMesh(
        "mesh_object",
        path_join(task_folder, config["mesh_object"]["mesh_file"].as<std::string>()),
        scale);
  }
  else
  {
    std::cout << "No object is loaded. Exit program." << std::endl;
    exit(0);
  }
  world->addObject(object);

  std::cout << "Loading contacts" << std::endl;

  {
    int max_contact_points = config["maximum_surface_contact_points"].as<int>();

    bool negate_normal =
        config[object_type.c_str()]["negate_contact_normal"].as<bool>();

    std::vector<Vector3d> disabled_dirs;
    if (config[object_type.c_str()]["disabled_normal_directions"])
    {
      std::vector<std::vector<double>> disabled_dirs_vec =
          config[object_type.c_str()]["disabled_normal_directions"]
              .as<std::vector<std::vector<double>>>();
      for (auto &dir : disabled_dirs_vec)
      {
        disabled_dirs.push_back(Vector3d(dir[0], dir[1], dir[2]));
      }
    }
    std::string contact_file = config[object_type.c_str()]["contact_file"].as<std::string>();
    if (contact_file == "none")
    {
      if (object_type != "mesh_object")
      {
        std::cout << "We only provide on-fly contact sampling for mesh object. Please use the provided contact file for the primitive object in /data/ folder. " << std::endl;
        exit(0);
      }
      sample_surface_contacts(object, &surface_pts, max_contact_points, config["mesh_object"]["scale"].as<double>(), disabled_dirs);
    }
    else
    {
      load_surface_contacts(
          path_join(task_folder, contact_file), &surface_pts, scale, disabled_dirs, negate_normal);

      while (surface_pts.size() > max_contact_points)
      {
        int idx = randi(surface_pts.size() - 1);
        surface_pts.erase(surface_pts.begin() + idx);
      }
    }
  }
  std::cout << "Number of surface contacts: " << surface_pts.size() << std::endl;

  // ---- Load Environment ----
  if (config["environment"])
  {
    std::cout << "Loading environment components" << std::endl;
    int max_env_parts = 20;
    // load blocks
    for (int i = 1; i <= max_env_parts; ++i)
    {
      std::string block_name = "block_" + std::to_string(i);
      std::string ellipsoid_name = "ellipsoid_" + std::to_string(i);
      std::string cylinder_name = "cylinder_" + std::to_string(i);
      if (config["environment"][block_name.c_str()])
      {
        std::vector<double> loc =
            config["environment"][block_name.c_str()]["location"]
                .as<std::vector<double>>();
        std::vector<double> dim =
            config["environment"][block_name.c_str()]["dimension"]
                .as<std::vector<double>>();
        SkeletonPtr env_block =
            createFixedBox(block_name, Vector3d(dim[0], dim[1], dim[2]),
                           Vector3d(loc[0], loc[1], loc[2]));
        world->addEnvironmentComponent(env_block);
      }
      if (config["environment"][ellipsoid_name.c_str()])
      {
        std::vector<double> loc =
            config["environment"][ellipsoid_name.c_str()]["location"]
                .as<std::vector<double>>();
        std::vector<double> dim =
            config["environment"][ellipsoid_name.c_str()]["dimension"]
                .as<std::vector<double>>();
        SkeletonPtr env_ellipsoid =
            createFixedEllipsoid(ellipsoid_name, Vector3d(dim[0], dim[1], dim[2]),
                                 Vector3d(loc[0], loc[1], loc[2]));
        world->addEnvironmentComponent(env_ellipsoid);
      }
      if (config["environment"][cylinder_name.c_str()])
      {
        std::vector<double> loc =
            config["environment"][cylinder_name.c_str()]["location"]
                .as<std::vector<double>>();
        double cylindar_r =
            config["environment"][cylinder_name.c_str()]["radius"]
                .as<double>();
        double cylindar_h = config["environment"][cylinder_name.c_str()]["height"]
                                .as<double>();
        SkeletonPtr env_cylindar =
            createFixedCylindar(cylinder_name, cylindar_r, cylindar_h,
                                Vector3d(loc[0], loc[1], loc[2]));
        world->addEnvironmentComponent(env_cylindar);
      }
    }
  }
  else
  {
    SkeletonPtr env_block =
        createFixedBox("no_environment", Vector3d(0.1, 0.1, 0.1),
                       Vector3d(0, 0, -1000));
    world->addEnvironmentComponent(env_block);
  }

  double charac_len = config["characteristic_length"].as<double>();
  double mu_mnp = config["robot_object_friction_coefficient"].as<double>();
  double mu_env =
      config["environment_object_friction_coefficient"].as<double>();

  std::vector<double> object_w =
      config["object_weight"].as<std::vector<double>>();
  Vector6d f_g = Eigen::Map<Vector6d>(object_w.data());
  std::vector<double> object_I =
      config["object_inertia"].as<std::vector<double>>();
  Matrix6d oi = Eigen::Map<Matrix6d>(object_I.data());

  // ---- Setup the rrt options ----
  double goal_thr = config["rrt_options"]["goal_thr"].as<double>();
  double wa = config["rrt_options"]["weight"]["rotation"].as<double>();
  double wt = config["rrt_options"]["weight"]["translation"].as<double>();
  WholeHandTASK::SearchOptions rrt_options;
  rrt_options.max_samples = config["rrt_options"]["max_samples"].as<int>();
  rrt_options.goal_biased_prob =
      config["rrt_options"]["goal_biased_prob"].as<double>();

  rrt_options.x_ub = Eigen::Map<Vector3d>(
      config["rrt_options"]["sample"]["position_upper_bound"]
          .as<std::vector<double>>()
          .data());
  rrt_options.x_lb = Eigen::Map<Vector3d>(
      config["rrt_options"]["sample"]["position_lower_bound"]
          .as<std::vector<double>>()
          .data());

  rrt_options.eps_trans =
      config["rrt_options"]["extend_epsilon"]["translation"].as<double>();
  rrt_options.eps_angle =
      (3.14 / 180.0) *
      config["rrt_options"]["extend_epsilon"]["rotation_degree"].as<double>();
  rrt_options.sampleSO3 =
      config["rrt_options"]["sample"]["rotation"]["SO3_enabled"].as<bool>();
  rrt_options.sample_rotation_axis =
      Eigen::Map<Vector3d>(config["rrt_options"]["sample"]["rotation"]["axis"]
                               .as<std::vector<double>>()
                               .data());

  // Whether to refine the trajectory in level 2
  bool is_refine = config["refine_object_trajectory"]["enabled"].as<bool>();
  double refine_dist =
      config["refine_object_trajectory"]["distance"].as<double>();

  // Whether to prune invalid transitions in the search
  task->if_transition_pruning = config["transition_pruning"].as<bool>();

  // ---- Setup the robot ----
  if (!config["whole_hand"])
  {
    std::cout << "No whole hand is configured in setup.yaml. Exit program." << std::endl;
  }
  std::string robot_folder_path = path_join(task_folder, config["whole_hand"]["folder_path"].as<std::string>());
  double robot_radius = config["whole_hand"]["radius"].as<double>();
  int n_robot_contacts = config["whole_hand"]["max_num_contacts"].as<int>();
  std::vector<std::string> robot_parts = config["whole_hand"]["allowed_parts"].as<std::vector<std::string>>();
  std::vector<int> robot_parts_idxs = config["whole_hand"]["allowed_part_point_idxes"].as<std::vector<int>>();

  //
  task->set_task_parameters(goal_thr, wa, wt, charac_len, mu_env, mu_mnp, oi,
                            f_g, world, n_robot_contacts, "quasistatic",
                            surface_pts, rrt_options, is_refine, refine_dist);

  std::shared_ptr<DartWholeHandManipulator> robot = std::make_shared<DartWholeHandManipulator>(robot_folder_path, robot_radius);
  robot->preprocess(robot_parts, robot_parts_idxs, n_robot_contacts);

  task->set_robot(robot);
}

void load_start_and_goal_poses(std::shared_ptr<WholeHandTASK> task,
                               const YAML::Node &config)
{
  Vector7d x_start;
  Vector7d x_goal;
  std::vector<double> start_pose =
      config["object_start_pose"].as<std::vector<double>>();
  std::vector<double> goal_pose =
      config["object_goal_pose"].as<std::vector<double>>();
  x_start << start_pose[0], start_pose[1], start_pose[2], start_pose[3],
      start_pose[4], start_pose[5], start_pose[6];
  x_goal << goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3],
      goal_pose[4], goal_pose[5], goal_pose[6];
  task->set_start_and_goal(x_start, x_goal);
}

void load_reward_functions(std::shared_ptr<WholeHandTASK> task,
                           const YAML::Node &config)
{
  task->grasp_measure_charac_length =
      config["grasp_measure_charac_length"].as<double>();
  task->action_prob_L2 = config["action_probability_L2"].as<std::string>();

  std::shared_ptr<RewardFunction> r1;
  std::shared_ptr<RewardFunction> r2;

  std::string l1 = config["level_1_reward"].as<std::string>();
  std::string l2 = config["level_2_reward"].as<std::string>();

  if (l1 == "CMGLevel1Reward")
  {
    r1 = std::make_shared<CMGLevel1Reward>();
  }
  else if (l1 == "InhandLevel1Reward")
  {
    r1 = std::make_shared<InhandLevel1Reward>();
  }
  else
  {
    std::cout << "No valid level 1 reward function specified. Exit program."
              << std::endl;
    exit(0);
  }

  if (l2 == "CMGLevel2Reward")
  {
    r2 = std::make_shared<CMGLevel1Reward>();
  }
  else if (l2 == "InhandLevel2Reward")
  {
    r2 = std::make_shared<InhandLevel2Reward>();
  }
  else
  {
    std::cout << "No valid level 2 reward function specified. Exit program."
              << std::endl;
    exit(0);
  }

  task->set_reward_functions(r1, r2);
}

void load_mcts_options(HMP::HierarchicalComputeOptions &compute_options,
                       const YAML::Node &config)
{
  compute_options.l1.max_time = config["mcts_options"]["max_time"].as<double>();
  compute_options.l1_1st.max_iterations =
      config["mcts_options"]["l1_1st_max_iterations"].as<int>();
  compute_options.l1.max_iterations =
      config["mcts_options"]["l1_max_iterations"].as<int>();
  compute_options.l2_1st.max_iterations =
      config["mcts_options"]["l2_1st_max_iterations"].as<int>();
  compute_options.l2.max_iterations =
      config["mcts_options"]["l2_max_iterations"].as<int>();
  compute_options.final_l2_1st.max_iterations =
      config["mcts_options"]["final_l2_1st_max_iterations"].as<int>();
  compute_options.final_l2.max_iterations =
      config["mcts_options"]["final_l2_max_iterations"].as<int>();
}

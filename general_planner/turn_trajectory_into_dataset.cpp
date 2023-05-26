// final_pose.csv 4x4
// environment_pcd_points.csv N_envx6
// object_pose_{t}.csv 4x4
// object_pcd_points_world_{t}.csv N_objx6
// robot_contact_points_world_{t}.csv N_robotx6
// environment_contact_points_world_{t}.csv N_envx6
// environment_contact_mode_{t}.csv 1xN_env

#include "../src/tasks/setup.h"

#define SPARSE 0
#define DENSE 1

bool is_same_vector(VectorXd v1, VectorXd v2)
{

  if (v1.size() != v2.size())
  {
    return false;
  }
  double eps = 1e-6;
  return (v1 - v2).norm() < eps;
}

void load_surface_contacts(const std::string &file_name,
                           std::vector<ContactPoint> *pts, double scale_x,
                           double scale_y, double scale_z,
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
    pos << v(0) * scale_x, v(1) * scale_y, v(2) * scale_z;
    ContactPoint p(pos, normal_dir * v.tail(3));
    pts->push_back(p);
  }
}

void save_contact_points(const std::vector<ContactPoint> &pts,
                         const std::string &file_name)
{
  int n = pts.size();
  MatrixXd data_mat(n, 6);
  for (int i = 0; i < n; i++)
  {
    data_mat.row(i) << pts[i].p.transpose(), pts[i].n.transpose();
  }
  saveData(file_name, data_mat);
}

int main(int argc, char *argv[])
{
  std::string config_file;
  std::string traj_file;

  std::string output_folder;

  int sparsity;

  if (argc > 4)
  {
    config_file = argv[1];
    traj_file = argv[2];
    output_folder = argv[3];
    mkdir(output_folder.c_str(), 0755);

    std::string sparsity_str = argv[4];
    if (sparsity_str == "sparse")
    {
      std::cout << "Using sparse trajectory" << std::endl;
      sparsity = SPARSE;
    }
    else if (sparsity_str == "dense")
    {
      std::cout << "Using dense trajectory" << std::endl;
      sparsity = DENSE;
    }
    else
    {
      std::cout << "Please specify sparse or dense" << std::endl;
      return 0;
    }
  }
  else
  {
    std::cout << "Please specify the path to the setup.yaml file and the path "
                 "to the trajectory file, the path to ouput folder, and sparse or dense."
              << std::endl;
    return 0;
  }

  YAML::Node config = YAML::LoadFile(config_file);

  // This file has normal pointing outwards, need negate them when loading
  std::string unit_cube_file =
      std::string(SRC_DIR) + "/data/cube_surface_contacts.csv";

  std::vector<Vector3d> disabled_dirs;

  std::vector<ContactPoint> env_pts;

  // ---- Load Environment ----
  if (config["environment"])
  {
    std::cout << "Loading environment blocks" << std::endl;
    for (int i = 1; i <= 20; ++i)
    {
      std::string block_name = "block_" + std::to_string(i);
      if (config["environment"][block_name.c_str()])
      {

        std::vector<double> loc =
            config["environment"][block_name.c_str()]["location"]
                .as<std::vector<double>>();

        std::vector<double> dim =
            config["environment"][block_name.c_str()]["dimension"]
                .as<std::vector<double>>();

        // load unit cube surface points,
        std::vector<ContactPoint> env_block_pts;
        load_surface_contacts(unit_cube_file, &env_block_pts, dim[0] / 2,
                              dim[1] / 2, dim[2] / 2, disabled_dirs, true);
        for (int i = 0; i < env_block_pts.size(); ++i)
        {
          env_block_pts[i].p += Vector3d(loc[0], loc[1], loc[2]);
        }
        env_pts.insert(env_pts.end(), env_block_pts.begin(),
                       env_block_pts.end());
      }
    }
    save_contact_points(env_pts, output_folder + "/environment_pcd_points.csv");
  }

  // Load object surface contacts in the body frame

  // ---- Load Object ----
  std::vector<ContactPoint> surface_pts;

  std::cout << "Loading object" << std::endl;

  if (config["box_object"])
  {

    std::vector<double> box_l =
        config["box_object"]["shape"].as<std::vector<double>>();

    std::cout << "Loading contacts" << std::endl;

    bool negate_normal =
        config["box_object"]["negate_contact_normal"].as<bool>();

    load_surface_contacts(unit_cube_file, &surface_pts, box_l[0] / 2,
                          box_l[1] / 2, box_l[2] / 2, disabled_dirs,
                          negate_normal);
  }
  else if (config["mesh_object"])
  {

    double scale = config["mesh_object"]["scale"].as<double>();

    std::cout << "Loading contacts" << std::endl;

    std::vector<Vector3d> disabled_dirs;

    bool negate_normal =
        config["mesh_object"]["negate_contact_normal"].as<bool>();

    load_surface_contacts(
        config["mesh_object"]["contact_file"].as<std::string>(), &surface_pts,
        scale, scale, scale, disabled_dirs, negate_normal);
  }
  else
  {
    std::cout << "No object is loaded. Exit program." << std::endl;
    exit(0);
  }

  // ---- Load Trajectory ----
  std::cout << "Loading trajectory" << std::endl;

  std::vector<std::vector<double>> data = openVectorData(traj_file);

  int n_data = data.size();
  int n_traj = n_data - 2;
  int n_robot_pts = int(data[1][0]);

  if (sparsity == DENSE)
  {
    for (int i = 2; i < n_data; ++i)
    {

      int n_env_pts = (data[i].size() - 7 - 6 * n_robot_pts) / 7;

      Vector7d object_pose;
      object_pose << data[i][0], data[i][1], data[i][2], data[i][3],
          data[i][4], data[i][5], data[i][6];

      std::vector<ContactPoint> robot_pts_o;
      for (int k = 0; k < n_robot_pts; ++k)
      {
        Vector6d v;
        v << data[i][7 + 6 * k], data[i][8 + 6 * k], data[i][9 + 6 * k],
            data[i][10 + 6 * k], data[i][11 + 6 * k], data[i][12 + 6 * k];
        robot_pts_o.push_back(ContactPoint(v.head(3), v.tail(3)));
      }

      std::vector<ContactPoint> env_pts_o;
      for (int k = 0; k < n_env_pts; ++k)
      {
        Vector6d v;
        v << data[i][7 + 6 * n_robot_pts + 6 * k],
            data[i][8 + 6 * n_robot_pts + 6 * k],
            data[i][9 + 6 * n_robot_pts + 6 * k],
            data[i][10 + 6 * n_robot_pts + 6 * k],
            data[i][11 + 6 * n_robot_pts + 6 * k],
            data[i][12 + 6 * n_robot_pts + 6 * k];
        env_pts_o.push_back(ContactPoint(v.head(3), v.tail(3)));
      }

      VectorXd env_contact_mode(n_env_pts);
      for (int k = 0; k < n_env_pts; ++k)
      {
        env_contact_mode(k) = data[i][7 + 6 * n_robot_pts + 6 * n_env_pts + k];
      }

      std::vector<ContactPoint> object_pts_w =
          transform_contact_points(surface_pts, object_pose);
      std::vector<ContactPoint> robot_pts_w =
          transform_contact_points(robot_pts_o, object_pose);
      std::vector<ContactPoint> env_pts_w =
          transform_contact_points(env_pts_o, object_pose);

      save_contact_points(object_pts_w, output_folder +
                                            "/object_pcd_points_world_" +
                                            std::to_string(i - 2) + ".csv");
      save_contact_points(robot_pts_w, output_folder +
                                           "/robot_contact_points_world_" +
                                           std::to_string(i - 2) + ".csv");
      save_contact_points(env_pts_w, output_folder +
                                         "/environment_contact_points_world_" +
                                         std::to_string(i - 2) + ".csv");
      saveData(output_folder + "/environment_contact_mode_" +
                   std::to_string(i - 2) + ".csv",
               env_contact_mode.transpose());
      saveData(output_folder + "/object_pose_" + std::to_string(i - 2) + ".csv",
               object_pose.transpose());

      if (i == n_data - 1)
      {
        saveData(output_folder + "/final_object_pose.csv", object_pose.transpose());
      }
    }
  }
  else
  {
    VectorXd pre_env_contact_mode(0);
    int k_save = -1;

    for (int i = 2; i < n_data; ++i)
    {
      int n_env_pts = (data[i].size() - 7 - 6 * n_robot_pts) / 7;
      
      VectorXd env_contact_mode(n_env_pts);
      for (int k = 0; k < n_env_pts; ++k)
      {
        env_contact_mode(k) = data[i][7 + 6 * n_robot_pts + 6 * n_env_pts + k];
      }

      bool if_save = (i==2) | (i==n_data-1) | (!is_same_vector(env_contact_mode, pre_env_contact_mode));

      if (!if_save)
      {
        continue;
      }
      k_save++;
      pre_env_contact_mode = env_contact_mode;

      Vector7d object_pose;
      object_pose << data[i][0], data[i][1], data[i][2], data[i][3],
          data[i][4], data[i][5], data[i][6];

      std::vector<ContactPoint> robot_pts_o;
      for (int k = 0; k < n_robot_pts; ++k)
      {
        Vector6d v;
        v << data[i][7 + 6 * k], data[i][8 + 6 * k], data[i][9 + 6 * k],
            data[i][10 + 6 * k], data[i][11 + 6 * k], data[i][12 + 6 * k];
        robot_pts_o.push_back(ContactPoint(v.head(3), v.tail(3)));
      }

      std::vector<ContactPoint> env_pts_o;
      for (int k = 0; k < n_env_pts; ++k)
      {
        Vector6d v;
        v << data[i][7 + 6 * n_robot_pts + 6 * k],
            data[i][8 + 6 * n_robot_pts + 6 * k],
            data[i][9 + 6 * n_robot_pts + 6 * k],
            data[i][10 + 6 * n_robot_pts + 6 * k],
            data[i][11 + 6 * n_robot_pts + 6 * k],
            data[i][12 + 6 * n_robot_pts + 6 * k];
        env_pts_o.push_back(ContactPoint(v.head(3), v.tail(3)));
      }

      std::vector<ContactPoint> object_pts_w =
          transform_contact_points(surface_pts, object_pose);
      std::vector<ContactPoint> robot_pts_w =
          transform_contact_points(robot_pts_o, object_pose);
      std::vector<ContactPoint> env_pts_w =
          transform_contact_points(env_pts_o, object_pose);

      save_contact_points(object_pts_w, output_folder +
                                            "/object_pcd_points_world_" +
                                            std::to_string(k_save) + ".csv");
      save_contact_points(robot_pts_w, output_folder +
                                           "/robot_contact_points_world_" +
                                           std::to_string(k_save) + ".csv");
      save_contact_points(env_pts_w, output_folder +
                                         "/environment_contact_points_world_" +
                                         std::to_string(k_save) + ".csv");
      saveData(output_folder + "/environment_contact_mode_" +
                   std::to_string(k_save) + ".csv",
               env_contact_mode.transpose());
      saveData(output_folder + "/object_pose_" + std::to_string(k_save) + ".csv",
               object_pose.transpose());

      if (i == n_data - 1)
      {
        saveData(output_folder + "/final_object_pose.csv", object_pose.transpose());
      }
    }

    //   transform_contact_points()
  }
}
// #include "../mechanics/mechanics.h"

// #include <algorithm>
// #include <iostream>
// #include <math.h>
// #include <mcts.h>
// #include <string>

// #include "../search/level1.h"

// #include "../tasks/cmg_task.h"

// #include "../mechanics/utilities/utilities.h"

// #include "../mechanics/manipulators/DartPointManipulator.h"
// #include "../mechanics/utilities/parser.hpp"
// #include "../mechanics/worlds/DartWorld.h"

// #ifndef DART_UTILS
// #define DART_UTILS
// #include "../mechanics/dart_utils/dart_utils.h"
// #endif

// #include "visualization.h"

// void peg()
// {
//   std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();
//   // Test with two fingers and one finger quasidynamics

//   double box_length = 2.0;
//   double box_height = 4.0;

//   double wall_width = 1.0;

//   std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

//   double gap = 0.1;

//   SkeletonPtr object =
//       createFreeBox("box_object", Vector3d(box_length, box_length, box_height));

//   SkeletonPtr wall1 =
//       createFixedBox("wall1", Vector3d(wall_width, box_length + wall_width * 2, box_height),
//                      Vector3d(-(box_length / 2 + gap + wall_width / 2), 0, box_height / 2));

//   SkeletonPtr wall2 =
//       createFixedBox("wall2", Vector3d(wall_width, box_length + wall_width * 2, box_height),
//                      Vector3d(box_length / 2 + gap + wall_width / 2, 0, box_height / 2));

//   SkeletonPtr wall3 = createFixedBox("wall3", Vector3d(box_length + wall_width * 2, wall_width, box_height),
//                                      Vector3d(0, box_length / 2 + gap + wall_width / 2, box_height / 2));

//   SkeletonPtr wall4 = createFixedBox("wall4", Vector3d(box_length + wall_width * 2, wall_width, box_height),
//                                      Vector3d(0, -(box_length / 2 + gap + wall_width / 2), box_height / 2));

//   SkeletonPtr ground =
//       createFixedBox("ground", Vector3d(10, 10, 1), Vector3d(0, 0, 1e-4 - 0.5));

//   world->addObject(object);
//   world->addEnvironmentComponent(wall1);
//   world->addEnvironmentComponent(wall2);
//   world->addEnvironmentComponent(wall3);
//   world->addEnvironmentComponent(wall4);
//   world->addEnvironmentComponent(ground);

//   int n_robot_contacts = 3;
//   DartPointManipulator *rpt =
//       new DartPointManipulator(n_robot_contacts, gap * 1.5);
//   rpt->is_patch_contact = true;
//   world->addRobot(rpt);

//   // set the task parameters, start, goal, object inertial, etc....

//   Vector7d x_start;
//   Vector7d x_goal;
//   x_start << 0, 0, box_height / 2, 0, 0, 0, 1;

//   x_goal << 0, -gap, box_height * 2.1, 0, 0, 0, 1;

//   double goal_thr = box_length * 3.14 * 10 / 180;

//   double wa = 1.0;
//   double wt = 1.0;

//   double mu_env = 0.1;
//   double mu_mnp = 0.9;

//   double charac_len = 1;

//   Vector6d f_g;
//   f_g << 0, 0, -0.1, 0, 0, 0;

//   Matrix6d oi;
//   oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
//       0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

//   // search options

//   CMGTASK::SearchOptions rrt_options;

//   rrt_options.x_ub << box_length, box_length, box_height * 2;
//   rrt_options.x_lb << -box_length, -box_length, 0.0;

//   rrt_options.eps_trans = 2.0;
//   // rrt_options.eps_angle = 3.14 * 95 / 180;
//   // rrt_options.eps_trans = 0.10;
//   rrt_options.eps_angle = 3.14 * 35 / 180;
//   rrt_options.max_samples = 50;
//   // rrt_options.sampleSO3 = false;
//   // rrt_options.sample_rotation_axis << 1, 0, 0;

//   rrt_options.goal_biased_prob = 0.7;

//   bool if_refine = false;
//   bool refine_dist = 0.5;

//   // read surface point, add robot contacts
//   std::vector<ContactPoint> surface_pts;
//   std::ifstream f(std::string(SRC_DIR) +
//                   "/data/test_cmg_peg/surface_contacts.csv");
//   aria::csv::CsvParser parser(f);

//   for (auto &row : parser)
//   {
//     int n_cols = row.size();
//     assert(n_cols == 6);

//     Vector6d v;
//     for (int j = 0; j < 6; ++j)
//     {
//       v(j) = std::stod(row[j]);
//     }
//     ContactPoint p(v.head(3), v.tail(3));
//     surface_pts.push_back(p);
//   }
//   // pass the world and task parameters to the task through task->initialize
//   task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
//                    mu_mnp, oi, f_g, world, n_robot_contacts, CMG_QUASISTATIC,
//                    surface_pts, rrt_options, if_refine, refine_dist);

//   std::vector<ContactPoint> mnps;
//   {
//     std::vector<int> fingertip_idx;
//     fingertip_idx.push_back(3);
//     std::vector<ContactPoint> fingertips;
//     for (int idx : fingertip_idx)
//     {
//       if (idx != -1)
//       {
//         fingertips.push_back(task->object_surface_pts[idx]);
//       }
//     }
//     task->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
//   }
//   std::vector<ContactPoint> envs;
//   {
//     {
//       ContactPoint cp(Vector3d(1, 1, -1), Vector3d(0, 1, 0));
//       envs.push_back(cp);
//     }
//     {
//       ContactPoint cp(Vector3d(-1, 1, -1), Vector3d(0, 1, 0));
//       envs.push_back(cp);
//     }
//     {
//       ContactPoint cp(Vector3d(-1, -1, -1), Vector3d(0, 1, 0));
//       envs.push_back(cp);
//     }
//     {
//       ContactPoint cp(Vector3d(1, -1, -1), Vector3d(0, 1, 0));
//       envs.push_back(cp);
//     }
//   }

//   Vector6d v;
//   v << 0, 0, 0.1, 0, 0, 0;
//   Vector6d f_ext_w;
//   f_ext_w << 0, 0, -1, 0, 0, 0;

//   Vector7d pose;
//   pose << 0, 0, 0, 0, 0, 0, 1;

//   ContactConstraints cons(2);
//   bool result = isQuasistatic(mnps, envs, v, f_ext_w, pose, 0.1, 0.6, &cons);
//   std::cout << "result: " << result << std::endl;
  
// }

// void bookshelf()
// {
//   // Test with two fingers and one finger quasidynamics

//   std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

//   double box_length = 4.0;
//   double box_width = 1.0;

//   std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

//   double gap = 0.1;

//   SkeletonPtr object =
//       createFreeBox("box_object", Vector3d(box_width, box_length, box_length));
//   SkeletonPtr book1 =
//       createFixedBox("book1", Vector3d(box_width, box_length, box_length),
//                      Vector3d(-box_width - gap, 0, box_length / 2));
//   SkeletonPtr book2 =
//       createFixedBox("book2", Vector3d(box_width, box_length, box_length),
//                      Vector3d(box_width + gap, 0, box_length / 2));
//   SkeletonPtr back = createFixedBox("back", Vector3d(4, 1, 4),
//                                     Vector3d(0, box_length / 2 + 0.5 + gap, 2));
//   SkeletonPtr ground =
//       createFixedBox("ground", Vector3d(10, 10, 1), Vector3d(0, 0, 1e-4 - 0.5));

//   world->addObject(object);
//   world->addEnvironmentComponent(book1);
//   world->addEnvironmentComponent(book2);
//   world->addEnvironmentComponent(back);
//   world->addEnvironmentComponent(ground);

//   int n_robot_contacts = 3;
//   DartPointManipulator *rpt =
//       new DartPointManipulator(n_robot_contacts, gap * 1.5);
//   rpt->is_patch_contact = true;
//   world->addRobot(rpt);

//   // set the task parameters, start, goal, object inertial, etc....

//   Vector7d x_start;
//   Vector7d x_goal;
//   x_start << 0, 0, box_length / 2, 0, 0, 0, 1;
//   x_goal << 0, -1.0, box_length, 0, 0, 0, 1;

//   double goal_thr = box_length * 3.14 * 10 / 180;

//   double wa = 1.0;
//   double wt = 1.0;

//   double mu_env = 0.1;
//   double mu_mnp = 0.9;

//   double charac_len = 1;

//   Vector6d f_g;
//   f_g << 0, 0, -0.1, 0, 0, 0;

//   Matrix6d oi;
//   oi << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1.0 / 6,
//       0, 0, 0, 0, 0, 0, 1.0 / 6, 0, 0, 0, 0, 0, 0, 1.0 / 6;

//   // search options

//   CMGTASK::SearchOptions rrt_options;

//   rrt_options.x_ub << box_width, box_width, box_length * 3;
//   rrt_options.x_lb << -box_width, -box_length * 1.5, 0.0;

//   rrt_options.eps_trans = 1.0;
//   // rrt_options.eps_angle = 3.14 * 95 / 180;
//   // rrt_options.eps_trans = 0.10;
//   rrt_options.eps_angle = 3.14 * 35 / 180;
//   rrt_options.max_samples = 50;
//   // rrt_options.sampleSO3 = false;
//   // rrt_options.sample_rotation_axis << 1, 0, 0;

//   rrt_options.goal_biased_prob = 0.5;

//   bool if_refine = false;
//   bool refine_dist = 0.5;

//   // read surface point, add robot contacts
//   std::vector<ContactPoint> surface_pts;
//   std::ifstream f(std::string(SRC_DIR) +
//                   "/data/test_cmg_bookshelf/surface_contacts.csv");
//   aria::csv::CsvParser parser(f);

//   for (auto &row : parser)
//   {
//     int n_cols = row.size();
//     assert(n_cols == 6);

//     Vector6d v;
//     for (int j = 0; j < 6; ++j)
//     {
//       v(j) = std::stod(row[j]);
//     }
//     ContactPoint p(v.head(3), v.tail(3));
//     surface_pts.push_back(p);
//   }
//   // pass the world and task parameters to the task through task->initialize
//   task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
//                    mu_mnp, oi, f_g, world, n_robot_contacts, CMG_QUASISTATIC,
//                    surface_pts, rrt_options, if_refine, refine_dist);
//   // VisualizeSG(task->m_world, x_start, x_goal);
//   std::vector<ContactPoint> mnps;
//   {
//     std::vector<int> fingertip_idx;
//     fingertip_idx.push_back(3);
//     std::vector<ContactPoint> fingertips;
//     for (int idx : fingertip_idx)
//     {
//       if (idx != -1)
//       {
//         fingertips.push_back(task->object_surface_pts[idx]);
//       }
//     }
//     task->m_world->getRobot()->Fingertips2PointContacts(fingertips, &mnps);
//   }
//   std::vector<ContactPoint> envs;
//   {
//     {
//       ContactPoint cp(Vector3d(1, 1, -1), Vector3d(0, 0, 1));
//       envs.push_back(cp);
//     }
//     {
//       ContactPoint cp(Vector3d(-1, 1, -1), Vector3d(0, 0, 1));
//       envs.push_back(cp);
//     }
//     {
//       ContactPoint cp(Vector3d(-1, -1, -1), Vector3d(0, 0, 1));
//       envs.push_back(cp);
//     }
//     {
//       ContactPoint cp(Vector3d(1, -1, -1), Vector3d(0, 0, 1));
//       envs.push_back(cp);
//     }
//   }

//   Vector6d v;
//   v << 0, 0, 0.1, 0, 0, 0;
//   Vector6d f_ext_w;
//   f_ext_w << 0, 0, -1, 0, 0, 0;

//   Vector7d pose;
//   pose << 0, 0, 0, 0, 0, 0, 1;

//   ContactConstraints cons(2);
//   bool result = isQuasistatic(mnps, envs, v, f_ext_w, pose, 0.1, 0.6, &cons);
//   std::cout << "result: " << result << std::endl;
// }

// void compare()
// {

//   std::vector<Vector6d> mnps;
//   std::vector<Vector6d> envs;

//   // {
//   //   Vector6d cp;
//   //   cp << -1,0,0.2,1,0,0;
//   //   mnps.push_back(cp);
//   // }

//   // {
//   //   Vector6d cp;
//   //   cp << 1,0,0.2,-1,0,0;
//   //   mnps.push_back(cp);
//   // }
//   {
//     Vector6d cp;
//     cp << 1, 0, 0, -1, 0, 0;
//     mnps.push_back(cp);
//   }
//   // envs
//   {
//     {
//       Vector6d cp;
//       cp << 1, 1, -1, 0, 0, 1;
//       envs.push_back(cp);
//     }
//     {
//       Vector6d cp;
//       cp << -1, 1, -1, 0, 0, 1;
//       envs.push_back(cp);
//     }
//     {
//       Vector6d cp;
//       cp << -1, -1, -1, 0, 0, 1;
//       envs.push_back(cp);
//     }
//     {
//       Vector6d cp;
//       cp << 1, -1, -1, 0, 0, 1;
//       envs.push_back(cp);
//     }
//   }

//   std::vector<ContactPoint> mnp_contacts;
//   std::vector<ContactPoint> env_contacts;

//   for (auto pt : mnps)
//   {
//     ContactPoint cp(pt.head(3), pt.tail(3));
//     mnp_contacts.push_back(cp);
//   }

//   for (auto pt : envs)
//   {
//     ContactPoint cp(pt.head(3), pt.tail(3));
//     env_contacts.push_back(cp);
//   }

//   Vector6d f_ext_w;
//   f_ext_w << 0, 0, -1, 0, 0, 0;

//   Vector7d pose;
//   pose << 0, 0, 0, 0, 0, 0, 1;

//   Vector6d v;
//   v << 1, -0.1, 0, 0, 0, 0;

//   Eigen::VectorXi env_mode(12);
//   // env_mode << 0, 0, 0, 0, -1, 1, -1, 1, -1, 1, -1, 1;
//   env_mode << 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0;

//   bool is_balance =
//       force_balance(mnps, envs, env_mode, f_ext_w, pose, 0.3, 0.6);

//   bool is_quasistatic =
//       quasistatic_check_2(mnps, envs, v, pose, f_ext_w, 0.3, 0.6);

//   std::cout << "Force balance with mode " << is_balance << std::endl;
//   std::cout << "Force balance with complementarity " << is_quasistatic
//             << std::endl;
// }

// void grasp()
// {
//   std::vector<Vector6d> mnps;
//   std::vector<Vector6d> envs;

//   {
//     Vector6d cp;
//     cp << -1, 0, 0.2, 1, 0, 0;
//     mnps.push_back(cp);
//   }

//   {
//     Vector6d cp;
//     cp << 1, 0, 0.2, -1, 0, 0;
//     mnps.push_back(cp);
//   }
//   Vector7d pose;
//   pose << 0, 0, 0, 0, 0, 0, 1;
//   Vector6d v;
//   v << 0.1, 0, 0.1, 0.1, 0, 0.1;
//   Vector6d f_ext_w;
//   f_ext_w << 0, 0, -1, 0, 0, 0;

//   bool is_quasistatic =
//       quasistatic_check_2(mnps, envs, v, pose, f_ext_w, 0.3, 0.6);

//   std::cout << "Force balance with complementarity " << is_quasistatic
//             << std::endl;
// }

int main() { return 0; }

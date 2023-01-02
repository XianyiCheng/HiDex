
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

#include "../search/level2baseline.h"

int TOTAL_TIMESTEP = 20;

// #include "../search/level2fp.h"
const CMGTASK::State2::Action CMGTASK::State2::no_action = CMGTASK::State2::Action(-1, -1);
const CMGTASK::State::Action CMGTASK::State::no_action = -1;

void pushing(std::shared_ptr<CMGTASK> task)
{
  // create world, create environment, an object sliding on the table

  double box_length = 1.0;

  std::shared_ptr<DartWorld> world = std::make_shared<DartWorld>();

  SkeletonPtr object =
      createFreeBox("box_object", box_length * Vector3d(1, 1, 1));
  SkeletonPtr env1 =
      createFixedBox("ground", Vector3d(30, 30, 0.2), Vector3d(30, 30, -0.1));

  world->addObject(object);
  world->addEnvironmentComponent(env1);

  int n_robot_contacts = 1;
  DartPointManipulator *rpt =
      new DartPointManipulator(n_robot_contacts, box_length * 0.2);
  // rpt->is_patch_contact = true;
  world->addRobot(rpt);

  // set the task parameters, start, goal, object inertial, etc....

  Vector7d x_start;
  Vector7d x_goal;
  x_start << 0, 0, box_length / 2 * 0.9999, 0, 0, 0, 1;
  x_goal << box_length * 5, 0, box_length / 2 * 0.9999, 0, 0, 0, 1;
  // x_goal << 0.1, 0, box_length/2 * 0.9999, 0, 0.7071, 0, 0.7071;

  double goal_thr = box_length * 3.14 * 30 / 180;

  double wa = 0.3;
  double wt = 10;

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

  rrt_options.x_ub << 5, 5, 1;
  rrt_options.x_lb << -5, -5, 0.0;

  rrt_options.eps_trans = 0.2;
  rrt_options.eps_angle = 3.14 * 95 / 180;
  rrt_options.max_samples = 20;

  rrt_options.goal_biased_prob = 0.7;

  bool is_refine = false;
  double refine_dist = 0.15;

  // read surface point, add robot contacts
  std::vector<ContactPoint> surface_pts;
  std::ifstream f(std::string(SRC_DIR) +
                  "/data/test_cmg_pushing/surface_contacts.csv");
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
    ContactPoint p(box_length / 2 * v.head(3), v.tail(3));
    surface_pts.push_back(p);
  }
  // pass the world and task parameters to the task through task->initialize
  task->initialize(x_start, x_goal, goal_thr, wa, wt, charac_len, mu_env,
                   mu_mnp, oi, f_g, world, n_robot_contacts, CMG_QUASISTATIC,
                   surface_pts,
                   rrt_options, is_refine, refine_dist);
}

void ours(std::vector<double> *results)
{
  std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

  pushing(task);

  std::cout << "Total number of actions " << task->n_finger_combinations << std::endl;

  // test level2 tree
  std::vector<Vector7d> test_object_traj;
  for (int k = 0; k < TOTAL_TIMESTEP; k++)
  {
    CMGTASK::State new_state;
    new_state.m_pose << 0, -TOTAL_TIMESTEP*0.5/2 + 0.5 * k, 0.9999 * 1 / 2, 0, 0, 0, 1;
    Vector7d pose_ = new_state.m_pose;
    pose_[1] += 0.25;
    new_state.m_path.push_back(new_state.m_pose);
    new_state.m_path.push_back(pose_);
    test_object_traj.push_back(new_state.m_pose);
    task->m_world->getObjectContacts(&new_state.envs, new_state.m_pose);
    task->saved_object_trajectory.push_back(new_state);
  }

  task->m_world->setObjectTrajectory(test_object_traj);

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2,
                  CMGTASK>::HierarchicalComputeOptions compute_options;

  compute_options.l2_1st.max_iterations = 20;
  compute_options.l2.max_iterations = 10;
  compute_options.l2.max_time = 1.0;

  HMP::Level2TreeFP<CMGTASK::State2, CMGTASK> tree2(task,
                                                    task->get_start_state2());

  tree2.ita = 0.1;

  HMP::Node<CMGTASK::State2> *final_node_2 =
      tree2.search_tree(compute_options.l2_1st, compute_options.l2);

  std::vector<VectorXd> mnp_traj;
  std::vector<CMGTASK::State2> action_trajectory =
      tree2.backtrack_state_path(final_node_2);

  double score = task->evaluate_path(action_trajectory);
  std::cout << "Score " << score << std::endl;

  for (auto &action : action_trajectory)
  {
    std::cout << "Timestep " << action.timestep << std::endl;
    std::cout << "Pose " << task->saved_object_trajectory[action.timestep].m_pose.transpose()
              << std::endl;
    std::cout << "Fingers ";
    for (int jj :
         task->get_finger_locations(action.finger_index))
    {
      std::cout << jj << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "Results: " << std::endl;

  std::cout << "Solution found time " << tree2.solution_found_time << std::endl;
  std::cout << "Total time " << tree2.total_time << std::endl;
  std::cout << "If solution found " << (score > 0) << std::endl;
  // std::cout << "Total number of finger transitions " << action_trajectory.size() - 1
  //           << std::endl;
  std::cout << "Total number of finger transitions " << task->total_finger_change_ratio(action_trajectory)
            << std::endl;

  std::cout << tree2.solution_found_time << " " << tree2.total_time << " " << (score > 0) << " " << task->total_finger_change_ratio(action_trajectory) << std::endl;

  results->push_back(tree2.solution_found_time);
  results->push_back(tree2.total_time);
  results->push_back(double(score > 0));
  results->push_back(task->total_finger_change_ratio(action_trajectory));

  // VializeStateTraj(task->m_world, task, task->saved_object_trajectory, action_trajectory);

  // int a = 1;
  // char** aa;
  // task->m_world->startWindow(&a, aa);
}

void baseline(std::vector<double> *results)
{
  std::shared_ptr<CMGTASK> task = std::make_shared<CMGTASK>();

  pushing(task);

  std::cout << "Total number of actions " << task->n_finger_combinations << std::endl;

  // test level2 tree
  std::vector<Vector7d> test_object_traj;
  for (int k = 0; k < TOTAL_TIMESTEP; k++)
  {
    CMGTASK::State new_state;
    new_state.m_pose << 0, -TOTAL_TIMESTEP*0.5/2 + 0.5 * k, 0.9999 * 1 / 2, 0, 0, 0, 1;
    Vector7d pose_ = new_state.m_pose;
    pose_[1] += 0.25;
    new_state.m_path.push_back(new_state.m_pose);
    new_state.m_path.push_back(pose_);
    test_object_traj.push_back(new_state.m_pose);
    task->m_world->getObjectContacts(&new_state.envs, new_state.m_pose);
    task->saved_object_trajectory.push_back(new_state);
  }

  task->m_world->setObjectTrajectory(test_object_traj);

  HMP::Level1Tree<CMGTASK::State, CMGTASK::State2,
                  CMGTASK>::HierarchicalComputeOptions compute_options;

  compute_options.l2_1st.max_iterations = 50;
  compute_options.l2.max_iterations = 10;
  compute_options.l2.max_time = 60.0;
  compute_options.l2_1st.max_time = 60.0;

  HMP::Level2TreeBaseline<CMGTASK::State2, CMGTASK> tree2(task,
                                                          task->get_start_state2());

  tree2.ita = 0.2;

  HMP::Node<CMGTASK::State2> *final_node_2 =
      tree2.search_tree(compute_options.l2_1st, compute_options.l2);

  std::vector<VectorXd> mnp_traj;
  std::vector<CMGTASK::State2> action_trajectory =
      tree2.backtrack_state_path(final_node_2);

  double score = task->evaluate_path(action_trajectory);
  std::cout << "Score " << score << std::endl;

  for (auto &action : action_trajectory)
  {
    std::cout << "Timestep " << action.timestep << std::endl;
    std::cout << "Pose " << task->saved_object_trajectory[action.timestep].m_pose.transpose()
              << std::endl;
    std::cout << "Fingers ";
    for (int jj :
         task->get_finger_locations(action.finger_index))
    {
      std::cout << jj << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "Results: " << std::endl;

  std::cout << "Solution found time " << tree2.solution_found_time << std::endl;
  std::cout << "Total time " << tree2.total_time << std::endl;
  std::cout << "If solution found " << (score > 0) << std::endl;
  std::cout << "Total number of finger transitions " << task->total_finger_change_ratio(action_trajectory)
            << std::endl;

  std::cout << tree2.solution_found_time << " " << tree2.total_time << " " << (score > 0) << " " << task->total_finger_change_ratio(action_trajectory) << std::endl;

  results->push_back(tree2.solution_found_time);
  results->push_back(tree2.total_time);
  results->push_back(double(score > 0));
  results->push_back(task->total_finger_change_ratio(action_trajectory));
  // VializeStateTraj(task->m_world, task, task->saved_object_trajectory, action_trajectory);

  // int a = 1;
  // char** aa;
  // task->m_world->startWindow(&a, aa);
}

int main(int argc, char *argv[])
{

  std::vector<double> results_ours;
  std::vector<double> results_baseline;

  int max_iterations = 10;

  for (int i = 0; i < max_iterations; i++)
  {
    // set_rand_seed();
    std::srand(10000*i + 13546);
    ours(&results_ours);
    baseline(&results_baseline);
  }
  
  std::cout << "Results (ours): " << std::endl;
  for (int i = 0; i < max_iterations; i++)
  {
    std::cout << results_ours[4*i] << " ";
    std::cout << results_ours[4*i+1] << " ";
    std::cout << results_ours[4*i+2] << " ";
    std::cout << results_ours[4*i+3] << std::endl;
  }
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "Results (baseline): " << std::endl;
  std::cout << std::endl;
  for (int i = 0; i < max_iterations; i++)
  {
    std::cout << results_baseline[4*i] << " ";
    std::cout << results_baseline[4*i+1] << " ";
    std::cout << results_baseline[4*i+2] << " ";
    std::cout << results_baseline[4*i+3] << std::endl;
  }
  return 0;
}

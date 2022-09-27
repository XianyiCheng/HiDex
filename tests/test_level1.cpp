// #include "../search/base.h"
#include "../search/level1.h"

#include "../tasks/test_task_l1.h"

#include "../mechanics/utilities/utilities.h"
// this header file includes ObjectState (State), and
// TestTaskL1 (Task)

int main() {

  // Test it with an example of planning to push an object in the plane

  std::shared_ptr<TestTaskL1> task =
      std::make_shared<TestTaskL1>(); // std::make_shared
  // define the task
  // environment, object, world model: collision detection ...
  // ...
  // Task search parameters

  Vector7d start_object_pose;
  start_object_pose << 0, 0, 0, 0, 0, 0, 1;
  Vector7d goal_object_pose;
  goal_object_pose << 1, 0, 0, 0, 0, 0, 1;

  task->initialize(start_object_pose, goal_object_pose,
                   TestTaskL1::SearchOptions(
                       Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(2, 2, 2)));

  // generate start_state from task
  TestTaskL1::State start_state = task->get_start_state();

  HMP::Level1Tree<TestTaskL1::State,
                  TestTaskL1>
      tree(task, start_state);

  HMP::ComputeOptions compute_options;
  compute_options.max_iterations = 100;

  HMP::Node<TestTaskL1::State> *current_node =
      tree.m_root_node.get();

  while (!tree.is_terminal(current_node)) {
    tree.grow_tree(current_node, compute_options);
    current_node = tree.best_child(current_node);
  }

  // backtrack the tree to get the path
  while (current_node != nullptr) {
    std::cout << "Node type: " << current_node->m_type << " Pose "
              << current_node->m_state.m_pose.transpose()
              << " Value: " << current_node->m_value
              << " Visits: " << current_node->m_visits << std::endl;
    current_node = current_node->m_parent;
  }
}
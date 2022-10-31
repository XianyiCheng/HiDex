#include "base.h"
#ifndef UTILS_H
#define UTILS_H
#include "../mechanics/utilities/utilities.h"
#endif

#include <chrono>
namespace HMP {
// 'State' is a different state than the 'State' in level1tree
// but 'Task' should be the same task
// State2 is defined in Task
template <typename State, typename Task>
class Level2Tree : public Tree<State, Task> {
public:
  Level2Tree() {}
  Level2Tree(std::shared_ptr<Task> task, State start_state)
      : Tree<State, Task>(task, start_state) {
    this->m_root_node->number_of_next_actions =
        this->m_task->get_number_of_robot_actions(start_state);
    this->m_root_node->m_state.is_valid = true;
  }
  ~Level2Tree() {}

  virtual void grow_tree(Node<State> *grow_node, const MCTSOptions &options) {

    // std::mt19937_64 random_engine(initial_seed);

    for (int iter = 1;
         iter <= options.max_iterations || options.max_iterations < 0; ++iter) {
      // std::cout << "Iter " << iter << std::endl;
      Node<State> *node = grow_node;

      while (!this->is_terminal(node)) {
        int action;
        action = this->select_action(node);
        node = this->next_node(node, action);
        node->number_of_next_actions =
            this->m_task->get_number_of_robot_actions(node->m_state);
      }

      double reward = this->get_result(node);
      // std::cout << "Evaluation: " << reward << std::endl;

      this->backprop_reward(node, reward);
    }
  }

  virtual State generate_next_state(Node<State> *node, int action) {
    State new_state = node->m_state;
    new_state.do_action(action);
    new_state.is_valid = this->m_task->is_valid(new_state, node->m_state);
    return new_state;
  }

  virtual double get_result(Node<State> *node) {

    // backtrack to get the a std::vector<State> for all mode nodes (except for
    // the last node)

    if (!this->is_terminal(node)) {
      // we can also learn a valua function... and return the value
      return 0.0;
    }
    if (!node->m_state.is_valid) {
      return 0.0;
    }

    if (node->m_visits > 0) {
      return node->m_value;
    }
    std::vector<State> state_path;

    state_path.push_back(node->m_state);

    Node<State> *node_ = node;

    while (node_ != 0) {
      state_path.push_back(node_->m_state);
      node_ = node_->m_parent;
    }

    std::reverse(state_path.begin(), state_path.end());

    double path_score = this->m_task->evaluate_path(state_path);

    return path_score;
  }

  virtual bool is_terminal(Node<State> *node) {
    return this->m_task->is_terminal(node->m_state);
  }

  virtual int select_action(Node<State> *node) {
    // sample based action selection
    int K = 50;

    std::vector<int> sampled_actions;

    if (node->number_of_next_actions < K) {
      // order the actions randomly
      for (int i = 0; i < node->number_of_next_actions; ++i) {
        sampled_actions.push_back(i);
      }
      // a random seed from the system clock
      auto rng = std::default_random_engine{};
      rng.seed(std::chrono::system_clock::now().time_since_epoch().count());
      std::shuffle(std::begin(sampled_actions), std::end(sampled_actions), rng);
    } else {
      // uniformly sample K actions
      for (int i = 0; i < K; ++i) {
        sampled_actions.push_back(randi(node->number_of_next_actions));
      }
    }

    // for each action, evaluate its heuristics
    std::vector<double> action_values;
    double total_value = 0.0;
    for (auto k : sampled_actions) {
      double heu_k;
      if (node->m_parent == nullptr) {
        heu_k = 1.0 /
                double(sampled_actions.size()); // todo: change this for
                                                // heuristics on initial actions
      } else {
        heu_k = this->m_task->action_heuristics_level2(k, node->m_state,
                                                       node->m_parent->m_state);
      }
      action_values.push_back(heu_k);
      total_value += heu_k;
    }

    // normalize the heuristics
    for (int i = 0; i < action_values.size(); ++i) {
      action_values[i] /= total_value;
    }

    // compute U for each action
    std::vector<double> U_values;
    for (int i = 0; i < sampled_actions.size(); ++i) {
      double child_value = 0.0;
      int child_visits = 0;
      for (auto child : node->m_children) // TODO: improve this
      {
        if (child->m_action == sampled_actions[i]) {
          child_value = child->m_value;
          child_visits = child->m_visits;
          break;
        }
      }
      double U = child_value + this->ita * action_values[i] *
                                   std::sqrt(double(node->m_visits)) /
                                   double(1 + child_visits);
      U_values.push_back(U);
    }

    // select the largest U value
    double U_max = -1.0;
    int action_idx = -1;
    for (int i = 0; i < U_values.size(); ++i) {
      if (U_values[i] > U_max) {
        U_max = U_values[i];
        action_idx = sampled_actions[i];
      }
    }

    return action_idx;
  }

  Node<State> *search_tree(const MCTSOptions &compute_option_1st_iter,
                           const MCTSOptions &compute_options) {

    Node<State> *current_node = this->m_root_node.get();

    if(!this->is_terminal(current_node)) {
      this->grow_tree(current_node, compute_option_1st_iter);
      current_node = this->best_child(current_node);
    }

    while (!this->is_terminal(current_node)) {
      this->grow_tree(current_node, compute_options);
      current_node = this->best_child(current_node);
    }

    // double final_best_reward = current_node->m_value;
    std::cout << "terminal node state valid? " << current_node->m_state.is_valid
              << std::endl;
    this->m_task->is_valid(current_node->m_state,
                           current_node->m_parent->m_state);
    return current_node;
  }
};

} // namespace HMP
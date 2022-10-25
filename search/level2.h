#include "base.h"
#ifndef UTILS_H
#define UTILS_H
#include "../mechanics/utilities/utilities.h"
#endif
namespace HMP
{
  // 'State' is a different state than the 'State' in level1tree
  // but 'Task' should be the same task
  // State2 is defined in Task
  template <typename State, typename Task>
  class Level2Tree : public Tree<State, Task>
  {
  public:
    Level2Tree() {}
    Level2Tree(std::shared_ptr<Task> task, State start_state)
        : Tree<State, Task>(task, start_state)
    {
      this->m_root_node->number_of_next_actions =
          this->m_task->get_number_of_robot_actions(start_state);
      this->m_root_node->m_state.is_valid = true;
    }
    ~Level2Tree() {}

    virtual void grow_tree(Node<State> *grow_node, const MCTSOptions &options)
    {

      // std::mt19937_64 random_engine(initial_seed);

      for (int iter = 1;
           iter <= options.max_iterations || options.max_iterations < 0; ++iter)
      {
        // std::cout << "Iter " << iter << std::endl;
        Node<State> *node = grow_node;

        while (!this->is_terminal(node))
        {
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

    virtual State generate_next_state(Node<State> *node, int action)
    {
      State new_state = node->m_state;
      new_state.do_action(action);
      new_state.is_valid = this->m_task->is_valid(new_state, node->m_state);
      return new_state;
    }

    virtual double get_result(Node<State> *node)
    {

      // backtrack to get the a std::vector<State> for all mode nodes (except for
      // the last node)

      if (!this->is_terminal(node))
      {
        // we can also learn a valua function... and return the value
        return 0.0;
      }
      if (!node->m_state.is_valid)
      {
        return 0.0;
      }

      if (node->m_visits > 0)
      {
        return node->m_value;
      }
      std::vector<State> state_path;

      state_path.push_back(node->m_state);

      Node<State> *node_ = node;

      while (node_ != 0)
      {
        state_path.push_back(node_->m_state);
        node_ = node_->m_parent;
      }

      std::reverse(state_path.begin(), state_path.end());

      double path_score = this->m_task->evaluate_path(state_path);

      return path_score;
    }

    virtual bool is_terminal(Node<State> *node)
    {
      return this->m_task->is_terminal(node->m_state);
    }

    virtual int select_action(Node<State> *node)
    {
      // must replace this because of too much robot actions
      // current solution: first look at the moves within its childred
      // then look at unexplored moves
      // TODO: need to improve this!

      if ((randd() > 0.5) && node->m_parent!=nullptr){
        if (node->m_parent->m_action != -1){
          return node->m_parent->m_action;
        }
      }

      double U_unexplored = 0.0 + this->ita * (1.0 / double(node->number_of_next_actions)) *
                                   std::sqrt(double(node->m_visits)) / (1 + 0);

      double U_max = -1.0;

      // TODO: change this -1 thing, the m_task->action_selection_policy_level2
      // should return something like a distribution, the U should be based on the
      // returned policy

      int action_idx = -1;

      std::vector<int> explored_actions;

      for (auto child : node->m_children)
      {
        double U = child->m_value + this->ita *
                                        (1.0 / double(node->number_of_next_actions)) *
                                        std::sqrt(double(node->m_visits)) /
                                        double(1 + child->m_visits);
        if (U > U_max)
        {
          U_max = U;
          action_idx = child->m_action;
        }
        explored_actions.push_back(child->m_action);
      }

      if ((U_unexplored > U_max) && (explored_actions.size() < node->number_of_next_actions)){
        // randomly select an unexplored action
        std::vector<int> unexplored_actions;
        for (int i = 0; i < node->number_of_next_actions; ++i){
          if (std::find(explored_actions.begin(), explored_actions.end(), i) == explored_actions.end()){
            unexplored_actions.push_back(i);
          }
        }
        action_idx = unexplored_actions[randi(unexplored_actions.size())];
      }

      if (action_idx == -1){
        std::cout << "Error: action_idx == -1" << std::endl;
        exit(0);
      }
      return action_idx;
    }

    Node<State> *search_tree(const MCTSOptions &compute_option_1st_iter,
                             const MCTSOptions &compute_options)
    {

      Node<State> *current_node = this->m_root_node.get();
      // int iter = 0;
      // while (!this->is_terminal(current_node)) {
      //   if (iter == 0) {
      //     this->grow_tree(
      //         current_node,
      //         compute_option_1st_iter); // grow the tree to the end
      //         (simulation)
      //                                   // for compute_options.max_iteration
      //   } else {
      //     this->grow_tree(current_node, compute_options);
      //   }
      //   current_node = this->best_child(current_node);
      //   iter++;
      // }

      while (!this->is_terminal(current_node))
      {
        this->grow_tree(current_node, compute_options);
        current_node = this->best_child(current_node);
      }

      // double final_best_reward = current_node->m_value;
      std::cout << "terminal node state valid? " << current_node->m_state.is_valid << std::endl;
      this->m_task->is_valid(current_node->m_state, current_node->m_parent->m_state);
      return current_node;
    }
  };

} // namespace HMP
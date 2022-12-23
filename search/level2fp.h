#pragma once
#ifndef SEARCH_BASE_H
#define SEARCH_BASE_H
#include "base.h"
#endif
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
  class Level2TreeFP : public Tree<State, Task>
  {
  public:
    typedef typename Tree<State, Task>::Action Action;

    Level2TreeFP() {}
    Level2TreeFP(std::shared_ptr<Task> task, State start_state)
    {
      this->m_root_node = std::make_unique<Node<State>>(start_state, State::no_action, nullptr);
      this->m_current_node = this->m_root_node.get();
      this->m_task = task;
      this->m_root_node->number_of_next_actions =
          this->m_task->get_number_of_actions(start_state);
      this->m_root_node->m_state.is_valid = true;
    }
    ~Level2TreeFP() {}

    virtual void grow_tree(Node<State> *grow_node, const MCTSOptions &options)
    {

      // std::mt19937_64 random_engine(initial_seed);

      for (int iter = 1;
           iter <= options.max_iterations || options.max_iterations < 0; ++iter)
      {
        // std::cout << "Iter " << iter << std::endl;
        Node<State> *node = grow_node;

        int max_attempts = 10;
        int attempts = 0;

        while (!this->is_terminal(node) && attempts < max_attempts)
        {
          attempts++;
          Action action;
          action = this->select_action(node);

          // if cannot find a feasible action
          // break and evaluate the heuristics of this path
          if (action == State::no_action)
          {
            break;
          }
          node = this->next_node(node, action);
          // node->number_of_next_actions =
          //     this->m_task->get_number_of_actions(node->m_state);
          if (node->m_state.t_max == -1)
          {
            node->m_state.t_max =
                this->m_task->max_forward_timestep(node->m_state);
          }
        }

        double reward = this->get_result(node);
        // std::cout << "Evaluation: " << reward << std::endl;
        if (reward > 0)
        {
          if (this->found_positive_reward == false)
          {
            this->solution_found_time = this->elasped_time();
            this->found_positive_reward = true;
          }
        }

        this->backprop_reward(node, reward);

        // this->update_heuristics(node);
      }
    }

    // void update_heuristics(Node<State> *node) {
    //   // TODO: update the heuristics of the path
    //   // heu = total_timesteps - #finger_changes
    //   Node<State> *node_ = node;
    //   double total_finger_changes = 0;
    //   while (node_->m_parent != nullptr) {
    //     total_finger_changes += 1.0;
    //     node_ = node_->m_parent;
    //   }
    //   double heu = double(node->m_state.t_max) + 1.0 - total_finger_changes;

    //   node_ = node;
    //   while (node_->m_parent != nullptr) {
    //     if (node_->m_heuristics < heu) {
    //       node_->m_heuristics = heu;
    //     }
    //     node_ = node_->m_parent;
    //   }
    // }

    virtual State generate_next_state(Node<State> *node,
                                      Action action)
    {
      State new_state = node->m_state;
      new_state.do_action(action);
      // this->m_task->do_action(new_state, action);
      new_state.is_valid =
          true; // make sure it is valid for every state in select_action
      new_state.t_max = -1;
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

      double total_finger_changes = this->m_task->total_finger_change_ratio(
          this->backtrack_state_path(node));

      // double reward = double(node->m_state.t_max) + 1.0 - total_finger_changes;

      double x = total_finger_changes / double(node->m_state.t_max);
      double y = 10.80772595 * x + -4.59511985;
      double reward = 1.0 / (1.0 + std::exp(y));

      return reward;

      // std::vector<State> state_path;

      // state_path.push_back(node->m_state);

      // Node<State> *node_ = node;

      // while (node_ != 0)
      // {
      //   state_path.push_back(node_->m_state);
      //   node_ = node_->m_parent;
      // }

      // std::reverse(state_path.begin(), state_path.end());

      // double path_score = this->m_task->evaluate_path(state_path);

      // return path_score;
    }

    virtual bool is_terminal(Node<State> *node)
    {
      return this->m_task->is_terminal(node->m_state);
    }

    virtual Action select_action(Node<State> *node)
    {

      // select a child or create a new child

      // for a new child
      // select timestep
      // select finger_idx
      // it means that we change to this finger_idx at this timestep
      // output action

      // compute U for each existing child
      std::vector<double> U_values;

      for (auto child : node->m_children) // TODO: improve this
      {

        double child_visits = double(child->m_visits);

        double Q_value =
            (child->m_value * child_visits + child->m_value_estimate) /
            (child_visits + 1.0);

        double U = Q_value + this->ita * std::sqrt(double(node->m_visits)) /
                                 double(1 + child_visits);
        U_values.push_back(U);
      }

      // select the largest U value
      double U_max = -1.0;
      Action selected_action = State::no_action;
      for (int i = 0; i < U_values.size(); ++i)
      {
        if (U_values[i] > U_max)
        {
          U_max = U_values[i];
          selected_action = node->m_children[i]->m_action;
        }
      }

      double U_unexplored =
          0.0 + this->ita * std::sqrt(double(node->m_visits)) / double(1);
      if (U_unexplored < U_max)
      {
        return selected_action;
      }

      // cretae a new child
      int t = this->m_task->select_finger_change_timestep(node->m_state);

      bool if_valid = false;

      int finger_idx;

      int max_sample = 300;

      std::vector<long int> sampled_finger_idxes;

      // TODO: here sample finger idxes based on State2
      this->m_task->sample_likely_feasible_finger_idx(
          node->m_state, t, max_sample, &sampled_finger_idxes);

      for (int k_sample = 0; k_sample < sampled_finger_idxes.size(); k_sample++)
      {
        finger_idx = sampled_finger_idxes[k_sample];

        // finger_idx =
        //     randi(this->m_task->get_number_of_robot_actions(node->m_state));

        Action new_action = Action(t, finger_idx);

        State new_state = this->generate_next_state(
            node, new_action);

        if ((new_state.timestep < 0) || (new_state.timestep >= this->m_task->saved_object_trajectory.size()))
        {
          std::cout << "timestep is issue, debug here" << std::endl;
          State new_state_ = this->generate_next_state(
              node, new_action);
        }

        // is valid transition & valid for at least one timestep
        if (this->m_task->is_finger_valid(finger_idx, t))
        {
          if (this->m_task->is_valid_transition(new_state, node->m_state))
          {
            if_valid = true;
            break;
          }
        }
      }

      if (if_valid)
      {
        // std::cout << "select action " << finger_idx << " at timestep " << t
        // << std::endl;
        selected_action = Action(t, finger_idx);
      }
      else
      {
        selected_action = State::no_action;
      }

      return selected_action;
    }

    Node<State> *search_tree(const MCTSOptions &compute_option_1st_iter,
                             const MCTSOptions &compute_options)
    {
      bool if_check_time = (compute_options.max_time > 0);
      this->m_start_time = std::chrono::system_clock::now();

      bool if_early_stop = false;
      // Using time point and system_clock

      Node<State> *current_node = this->m_root_node.get();

      if (!this->is_terminal(current_node))
      {
        // std::cout << "first iter in search tree" << std::endl;
        this->grow_tree(current_node, compute_option_1st_iter);
        if (current_node->m_children.size() > 0)
        {
          current_node = this->best_child(current_node);
        }
      }

      // int iter = 0;

      while (!this->is_terminal(current_node))
      {
        // std::cout << "search tree iter: " << iter << std::endl;
        this->grow_tree(current_node, compute_options);
        if (current_node->m_children.size() == 0)
        {
          break;
        }
        current_node = this->best_child(current_node);

        // early stop when found a solution and time is up
        if (this->if_check_time && this->found_positive_reward)
        {
          this->check_time(compute_options.max_time);
          if (this->time_up)
          {
            if_early_stop = true;
            this->total_time = this->elasped_time();
            std::cout << "Time out" << std::endl;
          }
        }

        if (if_early_stop)
        {
          while (current_node->m_children.size() > 0)
          {
            current_node = this->best_child(current_node);
          }
          break;
        }
      }

      // double final_best_reward = current_node->m_value;
      std::cout << "Is solution found? " << this->found_positive_reward
                << std::endl;
      // std::cout << "terminal node state valid? " <<
      // current_node->m_state.is_valid
      //           << std::endl;
      // this->m_task->is_valid(current_node->m_state,
      //                        current_node->m_parent->m_state);
      this->total_time = this->elasped_time();
      return current_node;
    }
  };

} // namespace HMP
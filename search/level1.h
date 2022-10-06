#include "level2.h"
#include <algorithm>

namespace HMP
{

  // level one tree should be compatible with many different tasks
  template <typename State, typename State2, typename Task>
  class Level1Tree : public Tree<State, Task>
  {
  public:
    double m_alpha = 0.5; // the parameter for expanding new continuous node

    Level1Tree() {}
    Level1Tree(std::shared_ptr<Task> task, State start_state)
        : Tree<State, Task>(task, start_state)
    {
      // specify the type of the root node
      // this->m_root_node = std::make_unique<Node<State>>(start_state, -1, nullptr); // already done in Tree
      this->m_root_node->m_type = "pose";
      this->m_root_node->number_of_next_actions =
          this->m_root_node->m_state.modes.size();
    }
    ~Level1Tree() {}

    virtual State generate_next_state(Node<State> *node, int action)
    {
      // this function is only used to generate new state from node type "pose", (to choose a mode)
      State new_state;
      if (node->m_type == "pose")
      {
        // from state.contact_modes
        new_state = node->m_state;
        new_state.do_action(action);
      }
      else
      { // node->m_type == "mode"
        //
        // Error, new state for node type "mode" can only be generated from
        // RRT
        std::cerr << "Error in generate_next_state. Wrong node type. New state "
                     "for node type mode can only be generated from RRT"
                  << std::endl;
      }

      return new_state;
    }

    virtual double get_result(Node<State> *node)
    {

      // evaluate the result initialing another tree search

      // backtrack to get the a std::vector<State> for all mode nodes (except for
      // the last node)

      if (!this->is_terminal(node))
      {
        return 0.0;
      }
      std::vector<State> state_path;

      state_path.push_back(node->m_state);

      Node<State> *node_ = node;

      while (node_ != 0)
      {
        if (node_->m_type == "mode")
        {
          state_path.push_back(node_->m_state);
        }
        node_ = node_->m_parent;
      }

      std::reverse(state_path.begin(), state_path.end());

      // create a level 2 tree to search for the robot contact path
      // do this->m_task->evaluate_path within level2
      this->m_task->saved_object_trajectory = state_path;

      Level2Tree<State2, Task> tree2(this->m_task, this->m_task->get_start_state2());

      MCTSOptions compute_options_2;
      compute_options_2.max_iterations =
          30; // maximum iteration for search from the root node

      double final_best_reward = tree2.search_tree(compute_options_2)->m_value;

      // try to capture all the task specific evaluation in the task->evaluate_path
      // only consider add reward terms here if they are a MCTS thing
      double path_score = final_best_reward; 
      this->m_task->saved_object_trajectory.clear();

      return path_score;
    }

    virtual bool is_terminal(Node<State> *node)
    {
      if (node->m_type == "mode")
      {
        return false;
      }
      else
      {
        // node type is "pose"
        if (node->m_parent == nullptr)
        {
          // root node cannot be a terminal node
          return false;
        }
        else
        {
          // all non-terminal pose node have at least one children (from the
          // initial rrt tree growth)
          return node->m_children.empty();
        }
      }
    }

    virtual void grow_tree(Node<State> *grow_node,
                           const MCTSOptions &options)
    {

      // std::mt19937_64 random_engine(initial_seed);

      for (int iter = 1;
           iter <= options.max_iterations || options.max_iterations < 0; ++iter)
      {
        std::cout << "Iter " << iter << std::endl;
        Node<State> *node = grow_node;

        while (!this->is_terminal(node))
        {
          int action;
          if (node->m_type == "pose")
          {
            action = this->select_action(node);
            node = this->next_node(node, action);

            // if the node hasn't been initialized yet
            if (node->m_type.empty())
            {
              node->m_type = "mode";
              node->number_of_next_actions = 0;
            }
          }

          else if (node->m_type == "mode")
          {
            if (node->number_of_next_actions <= pow(node->m_visits, m_alpha))
            {
              
              // rrt adds a path of nodes to the node, and return a terminal node
              std::vector<State> state_path =
                  this->m_task->search_a_new_path(node->m_state);

              if (!state_path.empty())
              {

                for (int k = 0; k < state_path.size() - 1; ++k)
                {
                  // need to create a "pose" and "mode" node
                  auto state_ = state_path[k];
                  {
                    // add a pose mode under a mode node
                    auto new_node =
                        this->add_child(node, node->number_of_next_actions, state_);
                    node->number_of_next_actions++;
                    new_node->m_type = "pose";
                    new_node->number_of_next_actions = state_.modes.size();
                    node = new_node;
                  }

                  {
                    auto new_node =
                        this->add_child(node, state_.m_mode_idx, state_);
                    new_node->m_type = "mode";
                    node = new_node;
                  }
                }
                // return a path of states
                // generate a series of nodes from states
                auto state_ = state_path.back();
                {
                  // add a pose mode under a mode node
                  auto new_node =
                      this->add_child(node, node->number_of_next_actions, state_);
                  node->number_of_next_actions++;
                  new_node->m_type = "pose";

                  node = new_node; // the last node has node->number_of_next_actions
                                   // = 0
                }
              }
              break;
            }
            else
            {
              action = this->select_action(node);
              node = this->next_node(node, action);

              if (this->is_terminal(node))
              {

                std::cout << "Got to the end through node selection" << std::endl;
              }
            }
          }
          else
          {
            // error information: wrong node type
            std::cerr << "Level1Tree::grow_tree. Incorrect state type. The state "
                         "type can only be pose "
                         "or mode. "
                      << std::endl;
            exit(-1);
          }
        }

        double reward = this->get_result(node);
        // std::cout << "Evaluation: " << reward << std::endl;

        this->backprop_reward(node, reward);
      }
    }

    Node<State> * search_tree(MCTSOptions compute_options)
    {
      Node<State> *current_node = this->m_root_node.get();

      int iter = 0;
      while (!this->is_terminal(current_node))
      {
        if (iter < 5)
        {
          this->grow_tree(current_node, compute_options);
        }
        iter++;
        current_node = this->best_child(current_node);
      }
      // double best_final_reward = current_node->m_value;
      return current_node;
    }

    void get_final_results(Node<State>* best_terminal_node, std::vector<State>* object_trajectory, std::vector<State2>* action_trajectory){
      if (!this->is_terminal(best_terminal_node))
      {
        std::cerr<< "This is not a terminal node!" << std::endl;
        return;
      }

      object_trajectory->push_back(best_terminal_node->m_state);

      Node<State> *node_ = best_terminal_node;

      while (node_ != 0)
      {
        if (node_->m_type == "mode")
        {
          object_trajectory->push_back(node_->m_state);
        }
        node_ = node_->m_parent;
      }

      std::reverse(object_trajectory->begin(), object_trajectory->end());

      this->m_task->saved_object_trajectory = *object_trajectory;

      Level2Tree<State2, Task> tree2(this->m_task, this->m_task->get_start_state2());
      tree2.ita = 0.1;

      MCTSOptions compute_options_2;
      compute_options_2.max_iterations =
          20; // maximum iteration for search from the root node

      Node<State2>* final_node_2 = tree2.search_tree(compute_options_2);

      *action_trajectory = tree2.backtrack_state_path(final_node_2);

      return;

    }
  };
} // namespace HMP
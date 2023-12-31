#include <algorithm>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <chrono>

namespace HMP
{

  struct MCTSOptions
  {
    int number_of_threads;
    int max_iterations;
    double max_time;
    bool verbose;

    MCTSOptions()
        : number_of_threads(8), max_iterations(10000),
          max_time(-1.0), // default is no time limit.
          verbose(false)
    {
    }
  };

  template <typename State>
  class Node
  {

  public:
    typedef typename State::Action Action;
    double m_value = 0.0;
    double m_value_estimate = 0.0;
    int m_visits = 0;
    double m_heuristics = 0.0;

    std::string m_type; // store any type information for interleaving node types

    Node *m_parent;
    Action m_action = State::no_action(); // action that lead to this node
    int number_of_next_actions = 0;     // action actions are stored in state
    int number_of_invalid_attempts = 0; // number of invalid new attempts for search_a_new_path
    State m_state;
    std::vector<Node *> m_children;

    Node() : m_value(0), m_visits(0), m_parent(nullptr) {}

    Node(const State &state_) { m_state = state_; }

    Node(const State &state_, Action action_, Node *parent_)
    {
      m_state = state_;
      m_action = action_;
      m_parent = parent_;
    }

    ~Node()
    {
      for (auto child : m_children)
      {
        delete child;
      }
    }

    void update(double reward)
    {
      m_value = (m_visits * m_value + reward) / (m_visits + 1);
      m_visits++;
    }
  };

  // An abstract class for Tree1 and Tree2
  template <typename State, typename Task>
  class Tree
  {

  public:
    // time check
    std::chrono::time_point<std::chrono::system_clock> m_start_time; // timer
    bool time_up = false;
    bool if_check_time = false;
    double solution_found_time = 0;
    double total_time = 0;

    typedef typename State::Action Action;
    bool found_positive_reward = false;
    double ita = 0.1;             // hyper-parameter controlling the degree of exploration
    std::shared_ptr<Task> m_task; // a shared pointer for Task
    std::unique_ptr<Node<State>>
        m_root_node;             // unique pointer for root node to store everything
    Node<State> *m_current_node; // observe pointer for current node

    Tree() {}
    Tree(std::shared_ptr<Task> task_, State start_state)
    {
      this->m_root_node = std::make_unique<Node<State>>(start_state, State::no_action(), nullptr);
      this->m_current_node = this->m_root_node.get();
      m_task = task_;
    }
    ~Tree() {}

    int number_of_nodes_in_subtree(Node<State> *node)
    {
      int n = node->m_children.size();
      for (auto child : node->m_children)
      {
        n += this->number_of_nodes_in_subtree(child);
      }
      return n;
    }

    int number_of_tree_nodes()
    {
      return this->number_of_nodes_in_subtree(this->m_root_node.get());
    }

    void backprop_reward(Node<State> *node, double reward)
    {
      while (node != nullptr)
      {
        node->update(reward);
        node = node->m_parent;
      }
    }

    Node<State> *add_child(Node<State> *node, Action action_, const State &state_)
    {

      if (State::is_no_action(action_))
      {
        std::cout << "Node<State> *add_child: has no action" << std::endl;
        exit(0);
      }
      auto child_node = new Node<State>(state_, action_, node);
      node->m_children.push_back(child_node);
      return child_node;
    }

    Node<State> *next_node(Node<State> *node, Action action)
    {

      // check is the action already has a child
      // if so, return the child
      for (auto child : node->m_children)
      {
        if (child->m_action == action)
        {
          return child;
        }
      }

      // else generate the next state
      State next_state = this->generate_next_state(node, action);

      // add the child
      if (State::is_no_action(action))
      {
        std::cout << "next_node: has no action" << std::endl;
        exit(0);
      }
      Node<State> *next_node = new Node<State>(next_state, action, node);
      node->m_children.push_back(next_node);
      return next_node;
    }

    std::vector<State> backtrack_state_path(Node<State> *terminal_node)
    {
      std::vector<State> path;
      Node<State> *node = terminal_node;
      while (node != nullptr)
      {
        path.push_back(node->m_state);
        node = node->m_parent;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    int count_subtree_nodes(Node<State> *root_node)
    {
      int n_total = 1;
      for (auto n : root_node->m_children)
      {
        n_total += count_subtree_nodes(n);
      }
      return n_total;
    }

    int count_total_nodes()
    {
      return count_subtree_nodes(this->m_root_node.get());
    }

    virtual Node<State> *best_child(Node<State> *node)
    {
      // return *std::max_element(
      //     node->m_children.begin(), node->m_children.end(),
      //     [](Node<State> *a, Node<State> *b) { return a->m_visits <
      //     b->m_visits; });
      // ;

      // max value
      return *std::max_element(
          node->m_children.begin(), node->m_children.end(),
          [](Node<State> *a, Node<State> *b)
          { return a->m_value < b->m_value; });
      ;
    }

    // virtual unsigned long int select_action(Node<State> *node)
    // {
    //   // TODO: return an action that either unexplored or have the best UCT

    //   unsigned long int action_idx = 0;

    //   if (node->number_of_next_actions == 0)
    //   {
    //     std::cerr
    //         << "Error in Tree::select_action (base.h). No next action found. "
    //         << std::endl;
    //     exit(-1);
    //     return -1;
    //   }

    //   double U_max = -1.0;

    //   for (int k = 0; k < node->number_of_next_actions; ++k)
    //   {

    //     Node<State> *new_node = this->next_node(node, k);

    //     // double U = ??*new_node->m_value + ??*new_node->m_value_estimate + ita * (new_node->m_heuristics / double(node->number_of_next_actions)) *
    //     //  std::sqrt(double(node->m_visits)) /
    //     //  (1 + double(new_node->m_visits));
    //     double U = new_node->m_value + ita * (1 / double(node->number_of_next_actions)) *
    //                                        std::sqrt(double(node->m_visits)) /
    //                                        (1 + double(new_node->m_visits));

    //     if (U > U_max)
    //     {
    //       U_max = U;
    //       action_idx = k;
    //     }
    //   }
    //   return action_idx;
    // }

    double elasped_time()
    {
      std::chrono::time_point<std::chrono::system_clock> current_time =
          std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds =
          current_time - this->m_start_time;
      double time_now = double(elapsed_seconds.count());
      return time_now;
    }

    void check_time(double max_time)
    {

      double time_now = this->elasped_time();
      if (time_now > max_time)
      {
        this->time_up = true;
      }
    }

    virtual Action select_action(Node<State> *node) = 0;
    virtual void grow_tree(Node<State> *grow_node,
                           const MCTSOptions &options) = 0;
    virtual State generate_next_state(Node<State> *node, Action action) = 0;
    virtual double get_result(Node<State> *node) = 0;
    virtual bool is_terminal(Node<State> *node) = 0;
  };

} // namespace HMP

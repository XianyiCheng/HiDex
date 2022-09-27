#include <algorithm>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace HMP {

struct ComputeOptions {
  int number_of_threads;
  int max_iterations;
  double max_time;
  bool verbose;

  ComputeOptions()
      : number_of_threads(8), max_iterations(10000),
        max_time(-1.0), // default is no time limit.
        verbose(false) {}
};

template <typename State> class Node {

public:
  double m_value = 0.0;
  int m_visits = 0;
  double m_heuristics = 0.0;

  std::string m_type; // store any type information for interleaving node types

  Node *m_parent;
  int m_action = -1;
  int number_of_next_actions = 0; // action actions are stored in state
  State m_state;
  std::vector<Node *> m_children;

  Node() : m_value(0), m_visits(0), m_parent(nullptr) {}

  Node(const State &state_) { m_state = state_; }

  Node(const State &state_, int action_, Node *parent_) : Node(state_) {
    m_action = action_;
    m_parent = parent_;
  }

  ~Node() {
    for (auto child : m_children) {
      delete child;
    }
  }

  void update(double reward) {
    m_value = (m_visits * m_value + reward) / (m_visits + 1);
    m_visits++;
  }
};

// An abstract class for Tree1 and Tree2
template <typename State, typename Task> class Tree {

public:
  std::shared_ptr<Task> m_task; // a shared pointer for Task
  std::unique_ptr<Node<State>>
      m_root_node; // unique pointer for root node to store everything
  Node<State> *m_current_node; // observe pointer for current node

  Tree() {}
  Tree(std::shared_ptr<Task> task_, State start_state) {
    this->m_root_node = std::make_unique<Node<State>>(start_state, -1, nullptr);
    this->m_current_node = this->m_root_node.get();
    m_task = task_;
  }
  ~Tree() {}

  void backprop_reward(Node<State> *node, double reward) {
    while (node != nullptr) {
      node->update(reward);
      node = node->m_parent;
    }
  }

  Node<State> *add_child(Node<State> *node, int action_, const State &state_) {
    auto child_node = new Node<State>(state_, action_, node);
    node->m_children.push_back(child_node);
    return child_node;
  }

  Node<State> *next_node(Node<State> *node, int action) {

    // check is the action already has a child
    // if so, return the child
    for (auto child : node->m_children) {
      if (child->m_action == action) {
        return child;
      }
    }

    // else generate the next state
    State next_state = this->generate_next_state(node, action);

    Node<State> *next_node = new Node<State>(next_state, action, node);
    node->m_children.push_back(next_node);
    return next_node;
  }

  virtual Node<State> *best_child(Node<State> *node) {
    // return *std::max_element(
    //     node->m_children.begin(), node->m_children.end(),
    //     [](Node<State> *a, Node<State> *b) { return a->m_visits <
    //     b->m_visits; });
    // ;

    // max value
    return *std::max_element(
        node->m_children.begin(), node->m_children.end(),
        [](Node<State> *a, Node<State> *b) { return a->m_value < b->m_value; });
    ;
  }

  virtual int select_action(Node<State> *node) {
    // TODO: return an action that either unexplored or have the best UCT

    double ita = 0.1; // hyper-parameter controlling the degree of exploration
    int action_idx = 0;

    if (node->number_of_next_actions == 0) {
      std::cerr
          << "Error in Tree::select_action (base.h). No next action found. "
          << std::endl;
      exit(-1);
      return -1;
    }

    double U_max = -1.0;

    for (int k = 0; k < node->number_of_next_actions; ++k) {

      Node<State> *new_node = this->next_node(node, k);

      double U = new_node->m_value + ita * (1 / node->number_of_next_actions) *
                                         std::sqrt(double(node->m_visits)) /
                                         (1 + new_node->m_visits);

      if (U > U_max) {
        U_max = U;
        action_idx = k;
      }
    }
    return action_idx;
  }

  virtual void grow_tree(Node<State> *grow_node,
                         const ComputeOptions &options) = 0;
  virtual State generate_next_state(Node<State> *node, int action) = 0;
  virtual double get_result(Node<State> *node) const = 0;
  virtual bool is_terminal(Node<State> *node) = 0;
};

} // namespace HMP

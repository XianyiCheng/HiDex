`/search`: for the mcts search algorithm, should not include anything else except for the c++ standard library
`/tasks`: template classes Task, can include things from mechanics, etc. should not include anything from `/search`

### `/search` 

#### `base.h`

namespace HMP

`ComputeOptions`: compute options for mcts for tree growing

template class `Node`: 
this class should be universal for all the trees. Don't create inheritance or anything. Need a template `State`. `State` must have: operator `=`, a copy constructor, a default constructor

template class `Tree`:
An abstract class for trees in level 1 & 2. 
Need customized `Task` or different computation to realize pure virtual functions: `grow_tree`, `generate_next_state`, `get_result`, `is_terminal`. 
You can also choose to change functions: `best_child`, `select_action`. 
Do not change `backprop_reward`, `add_child`, `next_node`.

#### `level1.h`

`Level1Tree`: a derived class from `HMP::Tree`. Perform search interleaved by discrete ("mode") and continuous ("pose") spaces.
Requirements for `State`: 
```
struct State
  {
    (something store the config object_config);
    int m_mode_idx = -1; // the mode idx chosen for this state, to the next state
    std::vector<whatever data type> modes;

    State() {}

    State(const State &state_) {// implement copy constructor}

    void do_action(int action);

    State &operator=(const State &state_) {//implement operator = }
  };
```

Requirements for `Task`:
```
class Task
{

public:
  // it's better to define the task spefic state here
  
  Task() {}

  std::vector<State> search_a_new_path(const State &start_state)
  {
    // search a new path towards the end, given the START_STATE and M_MODE_IDX!!!

    // during the search, it figure out the constriants (modes) for the states

    // every state need to be associated with m_mode_idx (mode to this state)
  }

  double evaluate_path(const std::vector<State> &path)
  {
    // return the REWARD of the path: larger reward -> better path
  }
  private:
    // add whatever members you want
};
```
### `/tasks` 
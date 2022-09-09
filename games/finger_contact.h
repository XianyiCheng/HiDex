/*
class GameState
{
public:
    typedef int Move;
    static const Move no_move = ...

    void do_move(Move move);
    template<typename RandomEngine>
    void do_random_move(*engine);
    bool has_moves() const;
    std::vector<Move> get_moves() const;

    // Returns a value in {0, 0.5, 1}.
    // This should not be an evaluation function, because it will only be
    // called for finished games. Return 0.5 to indicate a draw.
    double get_result(int current_player_to_move) const;

    int player_to_move;

    // ...
private:
    // ...
};
*/

#include <algorithm>
#include <iostream>
#include <math.h>
#include <mcts.h>
#include <string>

#include "../mechanics/mechanics.h"

#include "../mechanics/utilities/io.h"

#include "../mechanics/utilities/parser.hpp"
class ObjectTrajectory {
public:
  // constructor: load from file: pose trajectory, env contact trajectory, env
  // contact mode trajectory, candidate surface points (including no_contact)
  // todo
  ObjectTrajectory() {}
  ObjectTrajectory(std::string folder_name, int number_of_fingers,
                   double environment_friction_coeff,
                   double finger_friction_coeff)
      : number_of_fingers(number_of_fingers),
        mu_env(environment_friction_coeff), mu_mnp(finger_friction_coeff) {

    {
      std::ifstream f(folder_name + "/surface_contacts.csv");
      aria::csv::CsvParser parser(f);

      for (auto &row : parser) {
        int n_cols = row.size();
        assert(n_cols == 6);

        Vector6d v;
        for (int j = 0; j < 6; ++j) {
          v(j) = std::stod(row[j]);
        }
        surface_points.push_back(v);
      }
    }

    {
      std::ifstream f(folder_name + "/object_pose_trajectory.csv");
      aria::csv::CsvParser parser(f);

      for (auto &row : parser) {
        int n_cols = row.size();
        assert(n_cols == 7);

        Vector7d v;
        for (int j = 0; j < 7; ++j) {
          v(j) = std::stod(row[j]);
        }
        object_pose_trajectory.push_back(v);
      }
    }

    {
      std::ifstream f(folder_name + "/environment_contact_trajectory.csv");
      aria::csv::CsvParser parser(f);

      for (auto &row : parser) {
        std::vector<Vector6d> cps;
        int n_cols = row.size();
        assert(n_cols % 6 == 0);

        int n_pts = int(n_cols / 6);

        for (int k = 0; k < n_pts; ++k) {
          Vector6d cp;
          for (int j = 0; j < 6; ++j) {
            cp(j) = std::stod(row[6 * k + j]);
          }
          cps.push_back(cp);
        }
        environment_contact_trajectory.push_back(cps);
      }
    }

    {
      std::ifstream f(folder_name + "/environment_contact_mode_trajectory.csv");
      aria::csv::CsvParser parser(f);

      for (auto &row : parser) {
        int n_cols = row.size();

        VectorXi v(n_cols);
        for (int j = 0; j < n_cols; ++j) {
          v(j) = std::stoi(row[j]);
        }
        environment_contact_mode_trajectory.push_back(v);
      }
    }

    total_timesteps = object_pose_trajectory.size();
    number_of_finger_location_combinations =
        pow(surface_points.size(), number_of_fingers);
  }

  bool valid_state(int timestep, int finger_location_index) {
    Vector6d gravity;
    gravity << 0, 0, -1, 0, 0, 0;

    std::vector<int> finger_locations =
        get_finger_locations(finger_location_index);

    std::vector<Vector6d> finger_contacts =
        get_finger_contacts(finger_locations);

    bool is_balance = force_balance(
        finger_contacts, environment_contact_trajectory.at(timestep),
        environment_contact_mode_trajectory.at(timestep), gravity,
        object_pose_trajectory.at(timestep), mu_env, mu_mnp);

    finger_contacts.clear();
    finger_locations.clear();

    return is_balance;
  }

  bool valid_state_transition(int timestep, int current_finger_location_index,
                              int next_finger_location_index) {
    // VectorXi relocate_mode(
    //     environment_contact_mode_trajectory[timestep].size());
    // relocate_mode.setZero();

    // std::vector<int> current_finger_locations =
    //     get_finger_locations(current_finger_location_index);
    // std::vector<int> next_finger_locations =
    //     get_finger_locations(next_finger_location_index);

    // std::vector<int> remain_finger_locations;

    // for (int k = 0; k < current_finger_locations.size(); ++k)
    // {
    //   if (current_finger_locations[k] == next_finger_locations[k])
    //   {
    //     remain_finger_locations.push_back(current_finger_locations[k]);
    //   }
    // }

    // Vector6d gravity;
    // gravity << 0, 0, -1, 0, 0, 0;

    // bool is_transition_balance =
    //     force_balance(get_finger_contacts(remain_finger_locations),
    //                   environment_contact_trajectory[timestep],
    //                   environment_contact_mode_trajectory[timestep], gravity,
    //                   object_pose_trajectory[timestep], mu_env, mu_mnp);

    // return is_transition_balance;
    return true;
  }

  int total_timesteps;
  int number_of_finger_location_combinations;

private:
  //
  std::vector<int> get_finger_locations(int finger_location_index) {

    int N = surface_points.size();
    int n = number_of_fingers;
    int x = finger_location_index;

    std::vector<int> finger_locations;
    for (int k = 0; k < n; ++k) {
      int a = int(x / pow(N, (n - k - 1)));
      x -= a * (int)pow(N, (n - k - 1));

      finger_locations.push_back(a);
    }
    return finger_locations;
  }

  std::vector<Vector6d>
  get_finger_contacts(const std::vector<int> &finger_locations) {
    // if contact is 0,0,0,0,0,0, (finger_location_index == 0) mean no contacts,

    std::vector<Vector6d> finger_contacts;
    for (int p : finger_locations) {
      if (p != 0) {
        finger_contacts.push_back(surface_points[p]);
      }
    }
    return finger_contacts;
  }

  std::vector<Vector6d> surface_points; // in the object frame

  std::vector<std::vector<Vector6d>>
      environment_contact_trajectory; // in the object frame
  std::vector<VectorXi> environment_contact_mode_trajectory;
  std::vector<Vector7d> object_pose_trajectory; // in the world frame

  double mu_env;
  double mu_mnp;

  int number_of_fingers;
};

class FingerContactState {
public:
  typedef int Move;
  static const Move no_move = -1;

  FingerContactState(ObjectTrajectory &obj_traj)
      : object_trajectory(obj_traj), timestep(0), finger_index(0),
        move_steplength(1), is_valid(true), player_to_move(1) {}

  FingerContactState()
      : timestep(0), finger_index(0), move_steplength(1), is_valid(true),
        player_to_move(1) {}

  void do_move(Move move) {
    // std::cout << "do move " << move << std::endl;
    // TODO check if valid
    if (finger_index != move){
      n_transition += 1;
    }
    bool is_valid_transition =
        object_trajectory.valid_state_transition(timestep, finger_index, move);

    timestep += move_steplength;

    if (timestep > object_trajectory.total_timesteps - 1) {
      timestep = object_trajectory.total_timesteps - 1;
    }

    finger_index = move;
    bool is_valid_state = object_trajectory.valid_state(timestep, finger_index);
    is_valid = is_valid_transition & is_valid_state;

    // if (finger_index == 2){
    //   is_valid = true;
    // } else {
    //   is_valid = false;
    // }
    // is_valid = true;

    return;
  }

  template <typename RandomEngine> void do_random_move(RandomEngine *engine) {
    dattest(has_moves());

    std::vector<int> moves = get_moves();
    std::uniform_int_distribution<Move> random_indexs(0, moves.size() - 1);

    auto move = moves[random_indexs(*engine)];
    // std::cout << "random move " << move << std::endl;
    // Todo: can also use this to filter out invalid moves
    do_move(move);
    return;
  }

  bool has_moves() const {
    // End states don't have moves.
    // End state definition: (1) timestep = N_end (2) non valid state: there is
    // no force balance, or valid finger transitions
    if (timestep >= object_trajectory.total_timesteps-1) {
      return false;
    }
    if (!is_valid) {
      return false;
    }
    return true;
  }

  std::vector<Move> get_moves() const {
    std::vector<Move> moves;

    if (has_moves()) {
      
      for (int k = 0;
           k < object_trajectory.number_of_finger_location_combinations; ++k) {
        moves.push_back(k);
      }
    }

    return moves;
  }

  // Returns a value in {0, 0.5, 1}.
  // This should not be an evaluation function, because it will only be
  // called for finished games. Return 0.5 to indicate a draw.
  double get_result(int current_player_to_move) const {
    // return 1 if current state is valid
    // std::cout << "get result " << is_valid << std::endl;
    // std::cout << "current finger index " << finger_index << std::endl;
    if (is_valid) {
      return 1.0 - 0.2*n_transition/(double)object_trajectory.total_timesteps;
    } else {
      return 0.0;
    }
    
  }

  int player_to_move;

  void set_move_steplength(int h) {
    move_steplength = h;
    return;
  }

private:
  int timestep;
  int finger_index; // current finger index
  int move_steplength;
  bool is_valid;
  ObjectTrajectory object_trajectory;
  double n_transition=0;
};

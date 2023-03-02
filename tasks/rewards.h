
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <stdarg.h>
#include <string>
#include <vector>



class RewardFunction {
public:
  RewardFunction(){}
  RewardFunction(std::vector<std::string> feature_names) {
    this->feature_names = feature_names;
  }
  virtual double get(const std::vector<double> &features) = 0;
  std::vector<std::string> get_feature_names() { return feature_names; }

protected:
  std::vector<std::string> feature_names;
};

class ExampleReward : public RewardFunction {
public:
// ExampleReward(){}
    ExampleReward(std::vector<std::string> feature_names) : RewardFunction(feature_names) {}
  double
  get(const std::vector<double> &features) override {
    return 0.0;
  }
};


class CMGLevel1Reward : public RewardFunction {
public:
  CMGLevel1Reward() {
    this->feature_names = {"object_travel_distance_ratio", "object_path_size", "finger_change_ratio", 
                           "number_environment_contact_changes"};
  }

  double
  get(const std::vector<double> &features) override {

    int n_features = features.size();

    if (n_features!= this->feature_names.size()) {
      std::cout << "Error in reward function: number of features does not match number of feature names" << std::endl;
      return 0.0;
    }

    double travel_distance_ratio = features[0];
    double path_size = features[1];
    double total_finger_change_ratio = features[2];
    double number_environment_contact_changes = features[3];

    // double dist = this->travel_distance(object_state_path);
    // double best_dist =
    //     this->shared_rrt->dist(this->start_object_pose, this->goal_object_pose);
    double x_dist = travel_distance_ratio;
    double y_dist = 3.39617221 * x_dist - 7.59164285;

    double x_path = path_size;
    double y_path = 0.64872688 * x_path - 4.52948518;

    // double total_finger_changes =
    //     this->total_finger_change_ratio(robot_contact_state_path);

    // double x_finger =
    //     total_finger_changes / double(robot_contact_state_path.back().t_max);
    double x_finger = total_finger_change_ratio;
    double y_finger = 11.14845406 * x_finger - 4.59804752;

    // int n_env_changes = this->number_environment_contact_changes(path);
    double x_env_changes = number_environment_contact_changes;
    double y_env_changes = 1.31 * x_env_changes - 4.41;

    double r_dist = 1.0 / (1.0 + std::exp(y_dist));
    double r_path = 1.0 / (1.0 + std::exp(y_path));
    double r_finger = 1.0 / (1.0 + std::exp(y_finger));
    double r_env_changes = 1.0 / (1.0 + std::exp(y_env_changes));

    double reward =
        0.4 * r_dist + 0.3 * r_path + 0.4 * r_finger; // + 0.3 * r_env_changes;

    return reward;
  }

};


class CMGLevel2Reward : public RewardFunction {
public:
  CMGLevel2Reward() {}

  double
  get(const std::vector<double> &features) override {
    return 0.0;
  }
};


class InhandLevel1Reward
    : public RewardFunction {
public:
  InhandLevel1Reward() {}

  double
  get(const std::vector<double> &features) override {
    return 0.0;
  }
};


class InhandLevel2Reward
    : public RewardFunction {
public:
  InhandLevel2Reward() {}

  double
  get(const std::vector<double> &features) override {
    return 0.0;
  }
};

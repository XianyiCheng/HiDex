
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <stdarg.h>
#include <string>
#include <vector>

class RewardFunction
{
public:
  RewardFunction() {}
  RewardFunction(std::vector<std::string> feature_names)
  {
    this->feature_names = feature_names;
  }
  bool check_feature_size(const std::vector<double> &features)
  {
    int n_features = features.size();

    if (n_features != this->feature_names.size())
    {
      std::cout << "Error in reward function: number of features does not match number of feature names" << std::endl;
      return false;
    }
    return true;
  }

  std::vector<std::string> get_feature_names() { return feature_names; }

  virtual double get(const std::vector<double> &features) = 0;

protected:
  std::vector<std::string> feature_names;
};

class ExampleReward : public RewardFunction
{
public:
  // ExampleReward(){}
  ExampleReward(std::vector<std::string> feature_names) : RewardFunction(feature_names) {}
  double
  get(const std::vector<double> &features) override
  {
    return 0.0;
  }
};

class CMGLevel1Reward : public RewardFunction
{
public:
  CMGLevel1Reward()
  {
    this->feature_names = {"object_travel_distance_ratio", "object_path_size", "average_finger_change_ratio",
                           "number_environment_contact_changes"};
  }

  double
  get(const std::vector<double> &features) override
  {

    if (!check_feature_size(features))
    {
      return 0.0;
    }

    double travel_distance_ratio = features[0];
    double path_size = features[1];
    double average_finger_change_ratio = features[2];
    double number_environment_contact_changes = features[3];

    double x_dist = travel_distance_ratio;
    double y_dist = 3.39617221 * x_dist - 7.59164285;

    double x_path = path_size;
    double y_path = 0.64872688 * x_path - 4.52948518;

    double x_finger = average_finger_change_ratio;
    double y_finger = 11.14845406 * x_finger - 4.59804752;

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

class CMGLevel2Reward : public RewardFunction
{
public:
  CMGLevel2Reward()
  {
    this->feature_names = {"average_finger_change_ratio", "average_grasp_centroid_distance"};
  }

  double
  get(const std::vector<double> &features) override
  {
    if (!check_feature_size(features))
    {
      return 0.0;
    }

    double finger_change_ratio = features[0];
    double average_grasp_centroid_distance = features[1];
    double reward = 0.0;

    double x_finger = finger_change_ratio;
    double y_finger = 10.80772595 * x_finger + -4.59511985;
    double reward_finger = 1.0 / (1.0 + std::exp(y_finger));

    if (average_grasp_centroid_distance > 0.0)
    {
      double x_grasp = average_grasp_centroid_distance;
      double y_grasp = 6.90675 * x_grasp - 6.90675;
      double reward_grasp = 1.0 / (1.0 + std::exp(y_grasp));
      reward = 0.5 * reward_finger + 0.5 * reward_grasp;
    }
    else
    {
      reward = reward_finger;
    }
    return reward;
  }
};

class InhandLevel1Reward
    : public RewardFunction
{
public:
  InhandLevel1Reward()
  {
    this->feature_names = {"object_travel_distance_ratio", "object_path_size", "average_finger_change_ratio"};
  }

  double
  get(const std::vector<double> &features) override
  {
    if (!check_feature_size(features))
    {
      return 0.0;
    }

    double travel_distance_ratio = features[0];
    double path_size = features[1];
    double average_finger_change_ratio = features[2];

    double x_dist = travel_distance_ratio;
    double y_dist = 3.39617221 * x_dist - 7.59164285;

    double x_path = path_size;
    double y_path = 0.64872688 * x_path - 4.52948518;

    double x_finger = average_finger_change_ratio;
    double y_finger = 11.14845406 * x_finger - 4.59804752;

    double r_dist = 1.0 / (1.0 + std::exp(y_dist));
    double r_path = 1.0 / (1.0 + std::exp(y_path));
    double r_finger = 1.0 / (1.0 + std::exp(y_finger));

    double reward = 0.4 * r_dist + 0.4 * r_path + 0.2 * r_finger;

    return reward;
  }
};

class InhandLevel2Reward
    : public RewardFunction
{
public:
  InhandLevel2Reward()
  {
    this->feature_names = {"average_finger_change_ratio", "average_grasp_centroid_distance", "average_distance_to_goal_fingertips"};
  }

  double
  get(const std::vector<double> &features) override
  {
    if (!check_feature_size(features))
    {
      return 0.0;
    }

    double finger_change_ratio = features[0];
    double average_grasp_centroid_distance = features[1];
    double average_distance_to_goal_fingertips = features[2];

    double reward = 0.0;

    double x_finger = finger_change_ratio;
    double y_finger = 10.80772595 * x_finger + -4.59511985;
    double reward_finger = 1.0 / (1.0 + std::exp(y_finger));

    double x_grasp = average_grasp_centroid_distance;
    double y_grasp = 6.90675 * x_grasp - 6.90675;
    double reward_grasp = 1.0 / (1.0 + std::exp(y_grasp));

    double reward_finger_distance =
        1.0 / (1 + std::exp(3.62 * average_distance_to_goal_fingertips - 2.09));

    if ((average_grasp_centroid_distance >= 0.0) && (average_distance_to_goal_fingertips >= 0.0))
    {
      reward = 0.1 * reward_finger + 0.1 * reward_grasp + 0.8 * reward_finger_distance;
    }
    else if (average_grasp_centroid_distance >= 0.0)
    {
      reward = 0.5 * reward_finger + 0.5 * reward_grasp;
    }
    else if (average_distance_to_goal_fingertips >= 0.0)
    {
      reward = 0.2 * reward_finger + 0.8 * reward_finger_distance;
    }
    else
    {
      reward = reward_finger;
    }

    return reward;
  }
};

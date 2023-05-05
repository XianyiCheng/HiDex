#include <yaml-cpp/yaml.h>

#include "task.h"
#include "../search/level1mcts.h"

#include "../mechanics/utilities/sample.h"
#include "../mechanics/utilities/utilities.h"
#include "../mechanics/contacts/contact_kinematics.h"

#include "../mechanics/manipulators/DartPointManipulator.h"
#include "../mechanics/manipulators/DartDeltaManipulator.h"
// #include "../mechanics/manipulators/DartDDHandScalable.h"

#include "../mechanics/utilities/parser.hpp"

#ifndef _DART_WORLD
#define _DART_WORLD
#include "../mechanics/worlds/DartWorld.h"
#endif

#include "../mechanics/utilities/io.h"

#ifndef DART_UTILS
#define DART_UTILS
#include "../mechanics/dart_utils/dart_utils.h"
#endif



void load_task(std::shared_ptr<TASK> task, const YAML::Node &config);

void load_start_and_goal_poses(std::shared_ptr<TASK> task, const YAML::Node &config);

void load_reward_functions(std::shared_ptr<TASK> task, const YAML::Node &config);

void load_mcts_options(HMP::HierarchicalComputeOptions& options, const YAML::Node &config);


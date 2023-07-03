#pragma once

#include "scenario/scenario.hpp"

namespace luhsoccer::scenario::book {

extern const Scenario ONE_OBSTACLE;
extern const Scenario SWITCH;
extern const Scenario SWITCH_OFFSET;
extern const Scenario THREE_SWITCH;
extern const Scenario SIX_SWITCH;
extern const Scenario EIGHT_SWITCH;

extern const Scenario MULTI_OBSTACLE;
extern const Scenario CROSS;

extern const Scenario MULTI_OBSTACLE_RANDOM;

extern const Scenario OFF_CROSS;

extern const Scenario RANDOM_SORT;

extern const Scenario OFF_CROSS_ENEMY;

extern const Scenario DIVB_SETUP;

extern const Scenario PASS_AND_RECEIVE;

extern const std::map<std::string, const Scenario&> BOOK;

}  // namespace luhsoccer::scenario::book
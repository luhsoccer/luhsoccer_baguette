#pragma once

#include <string>
#include <vector>
#include <time/time.hpp>
#include <optional>

namespace luhsoccer::game_data_provider {

// @todo change this
struct TeamInfo {
    std::string name{"UNKNOWN"};
    unsigned int score{0};
    unsigned int red_cards{0};
    unsigned int yellow_cards{0};
    std::vector<time::TimePoint> yellow_card_times{};
    unsigned int timeouts{0};
    time::TimePoint timeout_time{};
    unsigned int goalkeeper{0};
    std::optional<unsigned int> foul_counter;
    std::optional<unsigned int> ball_placement_failures;
    std::optional<bool> can_place_ball;
    std::optional<unsigned int> max_allowed_bots;
    std::optional<bool> bot_substitution_intent;
    std::optional<bool> ball_placement_failures_reached;
};

}  // namespace luhsoccer::game_data_provider
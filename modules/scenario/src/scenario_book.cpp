#include "scenario/scenario_book.hpp"
#include "common_types.hpp"

#include <vector>
#include <algorithm>
#include <random>

namespace luhsoccer::scenario::book {

const auto RAND_POS_FROM_BALL = [](const std::shared_ptr<const transform::WorldModel> &, const time::TimePoint &) {
    constexpr double MIN_DIST = 0.3;
    constexpr double MAX_DIST = 1.0;

    double angle = (double)rand() / RAND_MAX * 2 * L_PI;

    double dist = (double)rand() / RAND_MAX;
    dist = MIN_DIST + dist * (MAX_DIST - MIN_DIST);

    return transform::Position("ball", std::cos(angle) * dist, std::sin(angle) * dist);
};

const auto RAND_POS_FROM_CENTRE = [](const std::shared_ptr<const transform::WorldModel> &, const time::TimePoint &) {
    constexpr double MIN_X = -0.5;
    constexpr double MAX_X = 0.5;
    constexpr double MIN_Y = -0.5;
    constexpr double MAX_Y = 0.5;

    double x = (double)rand() / RAND_MAX;
    x = MIN_X + x * (MAX_X - MIN_X);

    double y = (double)rand() / RAND_MAX;
    y = MIN_Y + y * (MAX_Y - MIN_Y);

    return transform::Position("", x, y);
};

const Scenario ONE_OBSTACLE("one_obstacle",         //
                            {{"", 1.0, 0.0, 0.0}},  //
                            {{"", 0.0, 0.0, 0.0}},  //
                            {{skills::BodSkillNames::GO_TO_POINT, ScenarioTaskData(true, {}, {{"", -1.0, 0.0, 0.0}})}});
const Scenario SWITCH("switch",                                     //
                      {{"", 2.0, 0.0, 0.0}, {"", -2.0, 0.0, 0.0}},  //
                      {},                                           //
                      {{skills::BodSkillNames::GO_TO_POINT,
                        ScenarioTaskData(true,                   //
                                         {},                     //
                                         {{"", -2.0, 0.0, 0.0}}  //
                                         )},
                       {skills::BodSkillNames::GO_TO_POINT,
                        ScenarioTaskData(true,                  //
                                         {},                    //
                                         {{"", 2.0, 0.0, 0.0}}  //
                                         )}});
const Scenario SWITCH_OFFSET("switch-offset",                                //
                             {{"", 1.8, -0.3, 0.0}, {"", -1.8, -0.3, 0.0}},  //
                             {},                                             //
                             {{skills::BodSkillNames::GO_TO_POINT,
                               ScenarioTaskData(true,                   //
                                                {},                     //
                                                {{"", -2.0, 0.3, 0.0}}  //
                                                )},
                              {skills::BodSkillNames::GO_TO_POINT,
                               ScenarioTaskData(true,                  //
                                                {},                    //
                                                {{"", 2.0, 0.3, 0.0}}  //
                                                )}});
const Scenario THREE_SWITCH("three-switch",                                                      //
                            {{"", 0.0, 1.0, 0.0}, {"", 0.7, -0.7, 0.0}, {"", -0.7, -0.7, 0.0}},  //
                            {},                                                                  //
                            {{skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                   //
                                               {},                     //
                                               {{"", 0.0, -1.0, 0.0}}  //
                                               )},
                             {skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                   //
                                               {},                     //
                                               {{"", -0.7, 0.7, 0.0}}  //
                                               )},
                             {skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                  //
                                               {},                    //
                                               {{"", 0.7, 0.7, 0.0}}  //
                                               )}});
const Scenario SIX_SWITCH("six-switch",  //
                          {{"", 0.0, 1.0, 0.0},
                           {"", 0.7, -0.7, 0.0},
                           {"", -0.7, -0.7, 0.0},
                           {"", 0.0, -1.0, 0.0},
                           {"", -0.7, 0.7, 0.0},
                           {"", 0.7, 0.7, 0.0}},  //
                          {},                     //
                          {{skills::BodSkillNames::GO_TO_POINT,
                            ScenarioTaskData(true,                   //
                                             {},                     //
                                             {{"", 0.0, -1.0, 0.0}}  //
                                             )},
                           {skills::BodSkillNames::GO_TO_POINT,
                            ScenarioTaskData(true,                   //
                                             {},                     //
                                             {{"", -0.7, 0.7, 0.0}}  //
                                             )},
                           {skills::BodSkillNames::GO_TO_POINT,
                            ScenarioTaskData(true,                  //
                                             {},                    //
                                             {{"", 0.7, 0.7, 0.0}}  //
                                             )},
                           {skills::BodSkillNames::GO_TO_POINT,
                            ScenarioTaskData(true,                  //
                                             {},                    //
                                             {{"", 0.0, 1.0, 0.0}}  //
                                             )},
                           {skills::BodSkillNames::GO_TO_POINT,
                            ScenarioTaskData(true,                   //
                                             {},                     //
                                             {{"", 0.7, -0.7, 0.0}}  //
                                             )},
                           {skills::BodSkillNames::GO_TO_POINT,
                            ScenarioTaskData(true,                    //
                                             {},                      //
                                             {{"", -0.7, -0.7, 0.0}}  //
                                             )}});
const Scenario EIGHT_SWITCH("eight-switch",  //
                            {{"", 0.0, 1.0, 0.0},
                             {"", 0.7, -0.7, 0.0},
                             {"", -0.7, -0.7, 0.0},
                             {"", 0.0, -1.0, 0.0},
                             {"", -0.7, 0.7, 0.0},
                             {"", 0.7, 0.7, 0.0},
                             {"", 1.0, 0.0, 0.0},
                             {"", -1.0, 0.0, 0.0}},  //
                            {},                      //
                            {{skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                   //
                                               {},                     //
                                               {{"", 0.0, -1.0, 0.0}}  //
                                               )},
                             {skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                   //
                                               {},                     //
                                               {{"", -0.7, 0.7, 0.0}}  //
                                               )},
                             {skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                  //
                                               {},                    //
                                               {{"", 0.7, 0.7, 0.0}}  //
                                               )},
                             {skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                  //
                                               {},                    //
                                               {{"", 0.0, 1.0, 0.0}}  //
                                               )},
                             {skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                   //
                                               {},                     //
                                               {{"", 0.7, -0.7, 0.0}}  //
                                               )},
                             {skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                    //
                                               {},                      //
                                               {{"", -0.7, -0.7, 0.0}}  //
                                               )},
                             {skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                   //
                                               {},                     //
                                               {{"", -1.0, 0.0, 0.0}}  //
                                               )},
                             {skills::BodSkillNames::GO_TO_POINT,
                              ScenarioTaskData(true,                  //
                                               {},                    //
                                               {{"", 1.0, 0.0, 0.0}}  //
                                               )}});

const Scenario MULTI_OBSTACLE("multi-obstacle",        //
                              {{"", -3.0, 0.0, 0.0}},  //
                              {{"", -1.0, 0.7, 0.0},
                               {"", -1.5, -0.15, 0.0},
                               {"", -1.5, 0.2, 0.0},
                               {"", 0.0, 0.1, 0.0},
                               {"", 0.0, -0.3, 0.0},
                               {"", 0.7, 0.2, 0.0},
                               {"", 1.5, 0.7, 0.0}},  //
                              {{skills::BodSkillNames::GO_TO_POINT,
                                ScenarioTaskData(true,                  //
                                                 {},                    //
                                                 {{"", 3.0, 0.0, 0.0}}  //
                                                 )}});
const Scenario CROSS("cross",                                            //
                     {{"", -2.0, 0.0, 0.0}, {"", 0.0, -2.0, L_PI / 2}},  //
                     {},                                                 //
                     {{skills::BodSkillNames::GO_TO_POINT_ORIGINAL,
                       ScenarioTaskData(true,                  //
                                        {},                    //
                                        {{"", 2.0, 0.0, 0.0}}  //
                                        )},
                      {skills::BodSkillNames::GO_TO_POINT_ORIGINAL,
                       ScenarioTaskData(true,                       //
                                        {},                         //
                                        {{"", 0.0, 2.0, L_PI / 2}}  //
                                        )}});

const std::vector<std::pair<double, double>> RANDOM_POSES = {{
    {0.483, -0.106},  {-0.159, -0.913}, {0.221, -0.706},  {-0.463, 0.723},  {-0.818, -0.133}, {0.530, 0.461},
    {-0.448, 0.813},  {-1.004, -1.000}, {0.534, 0.657},   {-0.225, 0.277},  {0.550, -0.535},  {0.619, -0.429},
    {-1.307, 0.738},  {-1.420, 0.188},  {-0.201, 0.213},  {0.670, 0.938},   {-0.639, -0.489}, {0.553, 0.307},
    {-0.873, 0.304},  {-0.883, -0.035}, {-1.493, -0.859}, {-0.917, 0.518},  {-0.171, 0.606},  {-1.387, -0.646},
    {-0.943, 0.955},  {-0.178, 0.025},  {-0.704, 0.778},  {0.410, -0.664},  {0.107, 0.846},   {-0.312, 0.580},
    {0.382, 0.312},   {-1.316, -0.929}, {0.920, -0.484},  {-1.351, -0.695}, {-1.491, -0.524}, {-0.604, -0.077},
    {-0.965, -0.028}, {-0.351, 0.557},  {-0.143, 0.701},  {-0.234, 0.180},  {0.430, -0.277},  {-0.922, 0.179},
    {0.855, -0.525},  {-0.093, -0.538}, {0.127, 0.615},   {-0.527, -0.879}, {-1.159, 0.437},  {0.497, -0.714},
    {0.277, -0.845},  {-1.145, -0.892}, {-0.495, -0.400}, {-0.816, 0.231},  {-0.457, 0.517},  {-1.231, 0.896},
    {-0.146, -0.061}, {-1.157, 0.954},  {-0.084, 0.372},  {-0.306, -0.738}, {0.803, 0.206},   {-0.244, -0.922},
    {0.538, 0.464},   {0.488, 0.409},   {-1.309, -0.475}, {-1.498, 0.702},  {-1.216, -0.631}, {-0.494, 0.574},
    {0.874, -0.766},  {-0.512, -0.953}, {-0.152, -0.384}, {0.784, -0.548},  {-0.818, 0.539},  {-0.667, 0.920},
    {0.761, 0.706},   {-1.256, 0.243},  {-0.857, -0.114}, {0.475, -0.396},  {0.695, 0.991},   {-0.981, 0.889},
    {-0.718, 0.745},  {-1.315, 0.502},  {0.657, 0.267},   {-0.850, -0.392}, {-0.348, -0.725}, {-1.440, -0.242},
    {0.445, 0.400},   {-1.403, -0.233}, {0.362, -0.937},  {0.109, 0.824},   {-1.415, -0.617}, {-0.944, -0.380},
    {-1.233, -0.771}, {0.086, 0.482},   {-0.676, 0.547},  {-0.449, -0.401}, {-0.948, 0.668},  {-0.937, -0.652},
    {0.961, 0.491},   {-0.829, 0.270},  {-0.917, 0.545},  {-0.665, -0.789}, {0.954, 0.487},   {-1.353, 0.519},
    {0.688, -0.703},  {-0.036, 0.210},  {0.192, 0.336},   {-0.712, -0.255}, {0.032, -0.227},  {-0.787, 0.180},
    {-0.054, -0.358}, {0.110, 0.919},   {0.707, -0.914},  {-0.600, 0.408},  {-1.365, -0.724}, {-0.573, -0.448},
    {0.753, 0.564},   {-0.052, 0.150},  {0.013, -0.944},  {-0.784, -0.582}, {-1.170, -0.931}, {0.354, 0.269},
}};
// const auto RAND_POS = [](const std::shared_ptr<const transform::WorldModel> &, const time::TimePoint &) {
//     constexpr double MIN_X = -2.0;
//     constexpr double MAX_X = 2.0;
//     constexpr double MIN_Y = -1.5;
//     constexpr double MAX_Y = 1.5;

//     double x = (double)rand() / RAND_MAX;
//     x = MIN_X + x * (MAX_X - MIN_X);

//     double y = (double)rand() / RAND_MAX;
//     y = MIN_Y + y * (MAX_Y - MIN_Y);

//     return transform::Position("", x, y);
// };
const auto RAND_POS = [](const std::shared_ptr<const transform::WorldModel> &, const time::TimePoint &) {
    static size_t num_pos = 0;
    if (num_pos >= RANDOM_POSES.size()) {
        num_pos = 0;
    }

    transform::Position pos("", RANDOM_POSES[num_pos].first, RANDOM_POSES[num_pos].second);
    num_pos++;
    return pos;
};
const Scenario MULTI_OBSTACLE_RANDOM("multi-obstacle-random",                                                   //
                                     {{"", -3.0, 0.0}},                                                         //
                                     {{RAND_POS}, {RAND_POS}, {RAND_POS}, {RAND_POS}, {RAND_POS}, {RAND_POS}},  //
                                     {{skills::BodSkillNames::GO_TO_POINT,
                                       ScenarioTaskData(true,                  //
                                                        {},                    //
                                                        {{"", 3.0, 0.0, 0.0}}  //
                                                        )}});

const Scenario OFF_CROSS("off-cross",  //
                         {{"", 1.0, 0.5, 0.0}, {"", 1.01, -0.741, 0.15 * L_PI}, {"", 2.004, -1.862, 0.3 * L_PI}},
                         {{"", 2.5, -0.81, 1.3 * L_PI},
                          {"", 2.77, -1.035, 1.3 * L_PI},
                          {"", 3.021, -1.287, 1.3 * L_PI},
                          {"", 2.5, 0.5, 1.0 * L_PI}},
                         {{skills::BodSkillNames::GO_TO_POINT,
                           ScenarioTaskData(true,                             //
                                            {},                               //
                                            {{"", 3.21, -2.03, 0.25 * L_PI}}  //
                                            )},
                          {skills::BodSkillNames::GO_TO_POINT,
                           ScenarioTaskData(true,                              //
                                            {},                                //
                                            {{"", 3.175, 0.499, -0.1 * L_PI}}  //
                                            )},
                          {skills::BodSkillNames::GO_TO_POINT,
                           ScenarioTaskData(true,                              //
                                            {},                                //
                                            {{"", 1.058, 1.754, -0.3 * L_PI}}  //
                                            )}});

const Scenario OFF_CROSS_ENEMY("off-cross2",  //
                               {{"", 1.0, 0.5, 0.0},
                                // {"", 1.01, -0.741, 0.15 * L_PI},
                                {"", 2.004, -1.862, 0.3 * L_PI},
                                {"", 1.0, 0.5, 0.0},
                                // {"", 3.0, -0.741, 0.15 * L_PI},
                                {"", 1.0, -1.862, 0.3 * L_PI}},
                               {},
                               {
                                   {skills::BodSkillNames::GO_TO_POINT,
                                    ScenarioTaskData(true,                             //
                                                     {},                               //
                                                     {{"", 3.21, -2.03, 0.25 * L_PI}}  //
                                                     )},
                                   //  {skills::BodSkillNames::GO_TO_POINT,
                                   //   ScenarioTaskData(true,                              //
                                   //                    {},                                //
                                   //                    {{"", 3.175, 0.499, -0.1 * L_PI}}  //
                                   //                    )},
                                   {skills::BodSkillNames::GO_TO_POINT,
                                    ScenarioTaskData(true,                              //
                                                     {},                                //
                                                     {{"", 1.358, 1.754, -0.3 * L_PI}}  //
                                                     )},
                                   {skills::BodSkillNames::MARK_ENEMY_TO_BALL,
                                    ScenarioTaskData(false,  //
                                                     {{0, Team::ALLY}})},
                                   //  {skills::BodSkillNames::MARK_ENEMY_TO_GOAL,
                                   //   ScenarioTaskData(false,  //
                                   //                    {{1, Team::ALLY}})},
                                   {skills::BodSkillNames::MARK_ENEMY_TO_BALL,
                                    ScenarioTaskData(false,  //
                                                     {{1, Team::ALLY}})},
                               });

constexpr double MIN_Y = -1.0;
constexpr double MAX_Y = 1.0;
constexpr double X_START = -1.0;
constexpr double X_END = 1.0;
constexpr double NUM_ROBOTS = 6;

const auto RAND_POS_SORT = [](const std::shared_ptr<const transform::WorldModel> &, const time::TimePoint &) {
    static std::mutex mtx;
    const std::lock_guard<std::mutex> lock(mtx);
    static std::vector<int> numbers;
    static int index;
    if (numbers.size() == 0 || index == NUM_ROBOTS) {
        numbers.clear();
        for (int i = 0; i < NUM_ROBOTS; i++) {
            numbers.push_back(i);
        }
        // shuffle elements using a random number generator
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(numbers.begin(), numbers.end(), g);
        index = 0;
    }

    return transform::Position("", X_START, MIN_Y + (MAX_Y - MIN_Y) / (NUM_ROBOTS - 1) * numbers[index++]);
};
const Scenario RANDOM_SORT(
    "random-sort",                                                                                           //
    {{RAND_POS_SORT}, {RAND_POS_SORT}, {RAND_POS_SORT}, {RAND_POS_SORT}, {RAND_POS_SORT}, {RAND_POS_SORT}},  //
    {},                                                                                                      //
    {
        {skills::BodSkillNames::GO_TO_POINT,
         ScenarioTaskData(true,                                                                   //
                          {},                                                                     //
                          {{"", X_END, X_START + (X_END - X_START) / (NUM_ROBOTS - 1) * 0, 0.0}}  //
                          )},
        {skills::BodSkillNames::GO_TO_POINT,
         ScenarioTaskData(true,                                                                   //
                          {},                                                                     //
                          {{"", X_END, X_START + (X_END - X_START) / (NUM_ROBOTS - 1) * 1, 0.0}}  //
                          )},
        {skills::BodSkillNames::GO_TO_POINT,
         ScenarioTaskData(true,                                                                   //
                          {},                                                                     //
                          {{"", X_END, X_START + (X_END - X_START) / (NUM_ROBOTS - 1) * 2, 0.0}}  //
                          )},
        {skills::BodSkillNames::GO_TO_POINT,
         ScenarioTaskData(true,                                                                   //
                          {},                                                                     //
                          {{"", X_END, X_START + (X_END - X_START) / (NUM_ROBOTS - 1) * 3, 0.0}}  //
                          )},
        {skills::BodSkillNames::GO_TO_POINT,
         ScenarioTaskData(true,                                                                   //
                          {},                                                                     //
                          {{"", X_END, X_START + (X_END - X_START) / (NUM_ROBOTS - 1) * 4, 0.0}}  //
                          )},
        {skills::BodSkillNames::GO_TO_POINT,
         ScenarioTaskData(true,                                                                   //
                          {},                                                                     //
                          {{"", X_END, X_START + (X_END - X_START) / (NUM_ROBOTS - 1) * 5, 0.0}}  //
                          )},
    });

const Scenario MULTI_OBSTACLE_REAL("multi_obstacle_real",             //
                                   {{"", 1.6694, -2.143, L_PI / 2}},  //
                                   {{"", -1.0, 0.7, 0.0},
                                    {"", 1.46, -0.2, 0.0},
                                    {"", 0.0, 0.1, 0.0},
                                    {"", 0.0, -0.3, 0.0},
                                    {"", 0.7, 0.2, 0.0},
                                    {"", 1.5, 0.7, 0.0}},  //
                                   {{skills::BodSkillNames::GO_TO_POINT,
                                     ScenarioTaskData(true,                  //
                                                      {},                    //
                                                      {{"", 3.0, 0.0, 0.0}}  //
                                                      )}});
const Scenario DIVB_SETUP{"DivB-Setup",
                          {//
                           {"", -4.35, -2.85, L_PI / 2},
                           {"", -4.05, -2.85, L_PI / 2},
                           {"", -3.75, -2.85, L_PI / 2},
                           {"", -3.45, -2.85, L_PI / 2},
                           {"", -3.15, -2.85, L_PI / 2},
                           {"", -2.85, -2.85, L_PI / 2}},
                          {//
                           {"", 4.35, -2.85, L_PI / 2},
                           {"", 4.05, -2.85, L_PI / 2},
                           {"", 3.75, -2.85, L_PI / 2},
                           {"", 3.45, -2.85, L_PI / 2},
                           {"", 3.15, -2.85, L_PI / 2},
                           {"", 2.85, -2.85, L_PI / 2}},
                          {}};

const Scenario PASS_AND_RECEIVE{
    "PassAndReceive",
    {{RAND_POS_FROM_BALL}, {RAND_POS_FROM_CENTRE}},
    {},
    {{skills::BodSkillNames::PASS_BALL_TO_ROBOT, ScenarioTaskData(true, {{1, Team::ALLY}}, {}, {}, {})},
     {skills::BodSkillNames::RECEIVE_BALL, ScenarioTaskData(true, {}, {})}},
    true};

const Scenario WALL_AT_PENALTY_AREA(
    "WallAtPenaltyArea",                                                  //
    {{"", -3.0, 0.0, 0.0}, {"", -3.0, -0.5, 0.0}, {"", -3.0, 0.5, 0.0}},  //
    {{"", -0.5, 0.0, 0.0}},                                               //
    {{skills::BodSkillNames::WALL_AT_PENALTY_AREA,
      ScenarioTaskData(
          true,
          {{0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}},
          {}, {0, 3})},
     {skills::BodSkillNames::WALL_AT_PENALTY_AREA,
      ScenarioTaskData(
          true,
          {{0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}},
          {}, {1, 3})},
     {skills::BodSkillNames::WALL_AT_PENALTY_AREA,
      ScenarioTaskData(
          true,
          {{0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}, {0, Team::ENEMY}},
          {}, {2, 3})}},
    true);

const std::map<std::string, const Scenario &> BOOK{
    {ONE_OBSTACLE.getName(), ONE_OBSTACLE},
    {SWITCH.getName(), SWITCH},
    {SWITCH_OFFSET.getName(), SWITCH_OFFSET},
    {THREE_SWITCH.getName(), THREE_SWITCH},
    {SIX_SWITCH.getName(), SIX_SWITCH},
    {EIGHT_SWITCH.getName(), EIGHT_SWITCH},
    {MULTI_OBSTACLE.getName(), MULTI_OBSTACLE},
    {MULTI_OBSTACLE_REAL.getName(), MULTI_OBSTACLE_REAL},
    {CROSS.getName(), CROSS},
    {MULTI_OBSTACLE_RANDOM.getName(), MULTI_OBSTACLE_RANDOM},
    {OFF_CROSS.getName(), OFF_CROSS},
    {OFF_CROSS_ENEMY.getName(), OFF_CROSS_ENEMY},
    {RANDOM_SORT.getName(), RANDOM_SORT},
    {DIVB_SETUP.getName(), DIVB_SETUP},
    {PASS_AND_RECEIVE.getName(), PASS_AND_RECEIVE},
    {WALL_AT_PENALTY_AREA.getName(), WALL_AT_PENALTY_AREA},
};

}  // namespace luhsoccer::scenario::book
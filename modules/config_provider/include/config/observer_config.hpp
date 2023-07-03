#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct ObserverConfig : public Config {
    ObserverConfig() : Config("observer_config_file.toml", datatypes::ConfigType::SHARED) {}

    /* ------------------ PASS PROBABILITY ------------------ */
    static constexpr double RECEIVER_MIN_GOAL_DIST = 1.5;
    DoubleParam receiver_min_goal_dist = createDoubleParam(
        "receiver_min_goal_dist", "The minimum distance the receiving robot has to be away from the goal",
        "pass_receiver", RECEIVER_MIN_GOAL_DIST, 0.0, 5.0);

    static constexpr double PASS_PROBABILITY_MAX_ENEMY_VEL = 1.5;  // in m/s
    DoubleParam pass_prob_max_enemy_vel = createDoubleParam(
        "pass_prob_max_enemy_vel", "The assumed max velocity an enemy will try to intercept the ball with",
        "pass_receiver", PASS_PROBABILITY_MAX_ENEMY_VEL, 0.0, 5.0);

    static constexpr double PASS_PROBABILITY_ALLY_ROTATION_FACTOR = 0.2;
    DoubleParam pass_prob_rotation_factor =
        createDoubleParam("pass_prob_rotation_factor", "The assumed speed our robots rotate around the ball",
                          "pass_receiver", PASS_PROBABILITY_ALLY_ROTATION_FACTOR, 0.0, 2.0);

    /* ------------------ GOAL PROBABILITY ------------------*/
    static constexpr double GOAL_PROBABILITY_GOAL_ANGLE_HALVE_POINT = 3.3;  // In Degrees
    DoubleParam goal_prob_goal_angle_halve_point =
        createDoubleParam("goal_prob_goal_angle_halve_point",
                          "The angle at which the goal probability due to the robot-goal angle will be halved",
                          "goal_probability", GOAL_PROBABILITY_GOAL_ANGLE_HALVE_POINT, 0.0, 90.0);

    static constexpr double GOAL_PROBABILITY_DISTANCE_DROPOFF_POINT = 8.0;  // In Meters
    DoubleParam goal_prob_distance_dropoff_point = createDoubleParam(
        "goal_prob_distance_dropoff_point", "The distance at which a steep dropoff will occur (in the calculation)",
        "goal_probability", GOAL_PROBABILITY_DISTANCE_DROPOFF_POINT, 0.0, 9.0);

    /* ------------------- THREAT SCORE ------------------- */
    static constexpr double THREAT_SCORE_BALL_DISTANCE_FACTOR = 4.0;
    DoubleParam threat_score_ball_distance_factor =
        createDoubleParam("threat_score_ball_distance_factor",
                          "The factor with which the ball distance is multiplied (while calculating the threat score)",
                          "threat_score", THREAT_SCORE_BALL_DISTANCE_FACTOR, 0, 20.0);

    static constexpr double THREAT_SCORE_GOAL_DISTANCE_FACTOR = 9.0;
    DoubleParam threat_score_goal_distance =
        createDoubleParam("threat_score_goal_distance",
                          "The factor with which the goal distance is multiplied (while calculating the threat score)",
                          "threat_score", THREAT_SCORE_GOAL_DISTANCE_FACTOR, 0, 20.0);

    static constexpr double THREAT_SCORE_GOAL_ROTATION_FACTOR = 1;
    DoubleParam threat_score_goal_rotation =
        createDoubleParam("threat_score_goal_rotation",
                          "The factor with which the goal-rotation is multiplied (while calculating the threat score)",
                          "threat_score", THREAT_SCORE_GOAL_ROTATION_FACTOR, 0, 20.0);

    static constexpr double THREAT_SCORE_INVERSE_BALL_DISTANCE_FACTOR = 0.4;
    DoubleParam threat_score_inverse_ball_distance_factor =
        createDoubleParam("threat_score_inverse_ball_distance_factor",
                          "The (inverse) factor which is included in the ball-distance calculation (smaller -> smaller "
                          "distances count less)",
                          "threat_score", THREAT_SCORE_INVERSE_BALL_DISTANCE_FACTOR, 0.05, 5);

    /* ------------------ STRATEGY TYPE ------------------*/
    static constexpr double STRATEGY_TYPE_MAX_TIME = 1.5;
    DoubleParam strategy_type_max_time =
        createDoubleParam("strategy_type_max_time",
                          "The total time of ball posession that should be saved for the strategy type calculation",
                          "strategy_type", STRATEGY_TYPE_MAX_TIME, 0.1, 10.0);

    static constexpr double STRATEGY_TYPE_THRESHHOLD = 1.0;
    DoubleParam strategy_type_threshhold = createDoubleParam(
        "strategy_type_threshhold", "The time that the enemy needs to have the ball for the strategy time to change",
        "strategy_type", STRATEGY_TYPE_THRESHHOLD, 0.1, 10.0);

    /* ------------------ MISC ------------------*/
    // According to the rules a robot may not move more than 1m with the ball.
    static constexpr double MOVEMENT_ALLOWED_DISTANCE = 0.95;
    DoubleParam max_ball_movement_distance =
        createDoubleParam("max_ball_movement_distance", "The max distance a robot should move with the ball",
                          "ball_posession", MOVEMENT_ALLOWED_DISTANCE);

    static constexpr double BALL_POSESSION_THRESHHOLD = 0.40;  // In meters to the dribbler
    DoubleParam ball_posession_threshhold =
        createDoubleParam("ball_posession_threshhold",
                          "The maximum threshhold in which a ball counts as being controlled by a robot (not that the "
                          "ball is nesacarrly in the Dribbler, but that the Robot controlls the Balls area)",
                          "ball_posession", BALL_POSESSION_THRESHHOLD, 0, 2.0);

    static constexpr double PASS_DEFENDED_ANGLE_DEG = 13.0;  // In degrees
    DoubleParam pass_defended_angle =
        createDoubleParam("pass_defended_angle", "The angle in which a pass will be counted as being defended",
                          "pass_defense", PASS_DEFENDED_ANGLE_DEG);

    static constexpr double BEST_GOAL_POINT_MIN_HOLE_SIZE = 0.05;  // In meters
    DoubleParam best_goal_point_min_hole_size =
        createDoubleParam("best_goal_point_min_hole_size",
                          "The minimum size a hole in the shadows must have to be counted as a viable shootPoint",
                          "best_goal_point", BEST_GOAL_POINT_MIN_HOLE_SIZE, 0, 1.0);

    static constexpr double FUTURE_BALL_POS_DRAG_COEFF = 1.0;
    DoubleParam future_ball_pos_drag_coeff =
        createDoubleParam("future_ball_pos_drag_coeff",
                          "The approximate drag coefficiant the ball has to the carpet (smaller -> Ball will move more "
                          "like the classic linear approximation)",
                          "future_ball_pos", FUTURE_BALL_POS_DRAG_COEFF, 0.0, 5.0);

    /* ------------------ INTERCEPTOR ------------------*/
    static constexpr double INTERCEPTOR_ALPHA_FACTOR = 1.0;
    DoubleParam interceptor_alpha_factor = createDoubleParam(
        "interceptor_alpha_factor", "The factor the angle for the ball-interceptor is multiplied with", "interceptor",
        INTERCEPTOR_ALPHA_FACTOR, 0, 3.0);

    static constexpr double INTERCEPTOR_MAX_ROBOT_VEL = 2.0;  // In m/s
    DoubleParam interceptor_max_robot_vel =
        createDoubleParam("interceptor_max_robot_vel", "The assumed velocity of the robot ignoring acceleration",
                          "interceptor", INTERCEPTOR_MAX_ROBOT_VEL, 0.1, 5.0);

    static constexpr double INTERCEPTOR_MIN_BALL_VEL_THRESHOLD = 0.22;  // In m/s
    DoubleParam interceptor_min_ball_vel_threshold =
        createDoubleParam("interceptor_min_ball_vel_threshold",
                          "The minimum velocity the ball needs to have in order for a best-interceptor to exist",
                          "interceptor", INTERCEPTOR_MIN_BALL_VEL_THRESHOLD, 0.05, 10.0);

    static constexpr double INTERCEPTOR_MAX_DISTANCE_IN_BALL_VEL = 2.0;  // In seconds
    DoubleParam interceptor_max_distance_in_ball_vel =
        createDoubleParam("interceptor_max_distance_in_ball_vel",
                          "The maximum distance a robot can be away from the ball (in seconds which are multiplied "
                          "with the current ball speed -> Results in Distance)",
                          "interceptor", INTERCEPTOR_MAX_DISTANCE_IN_BALL_VEL, 0.5, 10.0);
};

}  // namespace luhsoccer::config_provider
#include "robot_control/components/features/target_feature.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "config/robot_control_visualization_config.hpp"
#include "marker_adapter.hpp"

namespace luhsoccer::robot_control {

std::pair<Eigen::Vector2d, Eigen::Vector2d> TargetFeature::calcArtificialDesiredVelocity(
    const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractShape>>& restricted_areas,
    const transform::Position& robot_position, const transform::Position& observe_position) const {
    //
    auto vec_and_vel_to_shape = this->getTransformToClosestPointFromShape(comp_data, robot_position, observe_position);
    auto robot_pos = robot_position.getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    Eigen::Vector2d desired_velocity = Eigen::Vector2d::Zero();
    Eigen::Vector2d valid_goal = Eigen::Vector2d::Zero();
    auto robot_vel = robot_position.getVelocity(comp_data.wm, observe_position, observe_position, comp_data.time);
    if (vec_and_vel_to_shape.vec.has_value() && robot_pos.has_value() && robot_vel.has_value()) {
        // check if robot is in restricted area
        bool in_restricted_area =
            std::any_of(restricted_areas.begin(), restricted_areas.end(),
                        [&robot_pos, &comp_data, &observe_position](const std::shared_ptr<const AbstractShape>& area) {
                            return area->isPointInArea(robot_pos->translation(), comp_data, observe_position);
                        });
        if (in_restricted_area) {
            constexpr double ESCAPE_MARGIN = 0.05;
            valid_goal = this->getUnrestrictedGoal(robot_pos->translation(), comp_data, restricted_areas,
                                                   observe_position, ESCAPE_MARGIN);
        } else {
            constexpr double GOAL_MARGIN = 0.01;
            valid_goal = this->getUnrestrictedGoal(vec_and_vel_to_shape.vec.value() + robot_pos->translation(),
                                                   comp_data, restricted_areas, observe_position, GOAL_MARGIN);
        }
        desired_velocity = this->weight.val(comp_data) * robotControlConfig().target_k_p * this->k_p.val(comp_data) *
                           (valid_goal - robot_pos->translation()) /
                           std::max(robotControlConfig().target_min_velocity.val(), robot_vel->head<2>().norm());

        if (robotControlVisualizationConfig().display_shapes) {
            marker::Circle m({"", valid_goal.x(), valid_goal.y()});
            constexpr double GOAL_MARKER_RADIUS = 0.025;
            m.setRadius(GOAL_MARKER_RADIUS);
            m.setColor(RC_GREEN);
            comp_data.ma.displayMarker(m);
        }
    }

    if (!ignore_velocity.val(comp_data) && vec_and_vel_to_shape.velocity.has_value() && robot_vel.has_value()) {
        Eigen::Vector2d target_vel = vec_and_vel_to_shape.velocity.value() + robot_vel.value().head(2);
        desired_velocity += this->weight.val(comp_data) * target_vel;
    }

    return {desired_velocity, valid_goal - robot_pos->translation()};
}

bool TargetFeature::isReached(const ComponentData& comp_data) const {
    VectorWithVelocityStamped transform_to_shape =
        this->getTransformToClosestPointFromShape(comp_data, comp_data.robot.getFrame(), "");
    if (!transform_to_shape.vec.has_value()) return false;

    bool velocity_zero = true;
    if (this->velocity_zero_for_reach.val(comp_data) && transform_to_shape.velocity.has_value()) {
        velocity_zero = transform_to_shape.velocity->norm() < ZERO_VELOCITY_TOLERANCE;
    }
    bool reached = transform_to_shape.vec->norm() < this->translational_tolerance.val(comp_data) && velocity_zero;
    this->setCookie(comp_data.td, "influence", !reached);
    return reached;
}

bool BallTargetFeature::isReached(const ComponentData& comp_data) const {
    auto ball_info = comp_data.wm->getBallInfo(comp_data.time);
    return ball_info.has_value() && ball_info->robot.has_value() && ball_info->robot.value() == comp_data.robot;
};

using PointCandidateT = std::tuple<Eigen::Vector2d, double, std::shared_ptr<const AbstractShape>>;

struct CandidateCompareLess {
    bool operator()(const PointCandidateT& lhs, const PointCandidateT& rhs) const {
        return std::get<1>(lhs) < std::get<1>(rhs);
    }
};

Eigen::Vector2d TargetFeature::getUnrestrictedGoal(
    const Eigen::Vector2d& goal_position, const ComponentData& comp_data,
    const std::vector<std::shared_ptr<const AbstractShape>>& restricted_areas,
    const transform::Position& observe_position, double margin) const {
    // check if point is in shape
    bool in_restricted_area =
        std::any_of(restricted_areas.begin(), restricted_areas.end(),
                    [&goal_position, &comp_data, &observe_position](const std::shared_ptr<const AbstractShape>& area) {
                        return area->isPointInArea(goal_position, comp_data, observe_position);
                    });

    if (!in_restricted_area) return goal_position;

    // get brim points
    std::set<PointCandidateT, CandidateCompareLess> brim_points;
    for (const auto& area : restricted_areas) {
        std::vector<Eigen::Vector2d> ps =
            area->getPointsOnBrim(comp_data, DEFAULT_BRIM_SPACING, observe_position, margin);
        std::transform(
            ps.begin(), ps.end(), std::inserter(brim_points, brim_points.end()),
            [&goal_position, &comp_data, &observe_position, &area](const Eigen::Vector2d& v) -> PointCandidateT {
                if (robotControlVisualizationConfig().display_valid_goal_candidates) {
                    marker::Circle p(transform::Position(observe_position.getFrame(), v.x(), v.y()));
                    constexpr double BRIM_POINT_RADIUS = 0.025;
                    p.setRadius(BRIM_POINT_RADIUS);
                    p.setColor(RC_RED);
                    comp_data.ma.displayMarker(p);
                }
                return {v, (goal_position - v).norm(), area};
            });
    }

    // check if closest brim point is on other shape
    for (const auto& candidate_it : brim_points) {
        // check if candidate in shape
        bool in_area = false;
        for (const auto& area : restricted_areas) {
            if (std::get<2>(candidate_it) == area) continue;
            if (area->isPointInArea(std::get<0>(candidate_it), comp_data, observe_position)) {
                in_area = true;
                break;
            }
        }

        if (!in_area) {
            return std::get<0>(candidate_it);
        }
    }
    return goal_position;
}

}  // namespace luhsoccer::robot_control
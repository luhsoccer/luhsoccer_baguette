
#pragma once

#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {
constexpr double MID_ALPHA = 0.5;
inline ComponentPosition complexMidPosition(const ComponentPosition& p1, const ComponentPosition& p2,
                                            const DoubleComponentParam& alpha = MID_ALPHA) {
    return {CALLBACK, [=](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) {
                auto t_p1 = p1.positionObject(wm, td).getCurrentPosition(wm);
                auto t_p2 = p2.positionObject(wm, td).getCurrentPosition(wm);
                if (t_p1 && t_p2) {
                    Eigen::Vector2d v1 = {t_p1->translation().x(), t_p1->translation().y()};
                    Eigen::Vector2d v2 = {t_p2->translation().x(), t_p2->translation().y()};

                    Eigen::Translation2d t_mid(v1 + (v2 - v1) * alpha.val(wm, td));
                    return transform::Position("", t_mid * Eigen::Rotation2Dd(0));
                }
                return transform::Position("");
            }};
}

}  // namespace luhsoccer::local_planner
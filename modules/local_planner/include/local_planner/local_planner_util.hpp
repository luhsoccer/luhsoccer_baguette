#pragma once

#include <Eigen/Dense>
#include <optional>
#include <stdexcept>

#include "config/config_store.hpp"

namespace luhsoccer::local_planner {
template <class P, typename T>
class LocalPlannerParam {
   public:
    LocalPlannerParam(const T constant) : param(nullptr), constant(constant), cs(nullptr){};
    LocalPlannerParam(const P& param) : param(&param), cs(nullptr){};

    virtual ~LocalPlannerParam() = default;
    LocalPlannerParam(const LocalPlannerParam&) = default;
    LocalPlannerParam& operator=(const LocalPlannerParam&) = default;
    LocalPlannerParam(LocalPlannerParam&&) noexcept = default;
    LocalPlannerParam& operator=(LocalPlannerParam&&) noexcept = default;

    [[nodiscard]] T val() const {
        if (this->constant) {
            return constant.value();
        } else if (this->param) {
            return param->val();
        } else {
            throw std::runtime_error("No LocalPlannerParam option defined!");
        }
    }

    operator T() const { return this->val(); }

   protected:
    LocalPlannerParam() : param(nullptr), cs(nullptr){};

   private:
    const P* param;
    std::optional<T> constant{};
    const config_provider::ConfigStore* cs;
};

using BoolLocalPlannerParam = LocalPlannerParam<config_provider::BoolParamClass, bool>;
using DoubleLocalPlannerParam = LocalPlannerParam<config_provider::DoubleParamClass, double>;
using IntLocalPlannerParam = LocalPlannerParam<config_provider::IntParamClass, int>;
using StringLocalPlannerParam = LocalPlannerParam<config_provider::StringParamClass, std::string>;

inline double clipVelocity(double vel, double min, double max) { return std::min(max, std::max(min, vel)); }
inline double clipVelocity(double vel, double max) { return std::min(max, std::max(-max, vel)); }
inline Eigen::Vector2d clipVelocity2(const Eigen::Vector2d& vel, double max) {
    return std::min(1.0, max / vel.norm()) * vel;
}

inline double clipAcceleration(double acc, double vel, double acc_max, double brk_max) {
    if (acc * vel > 0.0)
        return std::min(acc_max, std::max(-acc_max, acc));
    else
        return std::min(brk_max, std::max(-brk_max, acc));
}
}  // namespace luhsoccer::local_planner
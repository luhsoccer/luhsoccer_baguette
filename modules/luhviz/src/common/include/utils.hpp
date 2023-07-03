#pragma once

#include <vector>
#include <string>
#include <sstream>
#include "imgui.h"
#include "common_types.hpp"

namespace luhsoccer::luhviz {

class Utils {
   public:
    /**
     * @brief use this to split a string by a given delimiter and retrieve the splitted list of strings
     *
     * @param in
     * @param sep
     * @return auto
     */
    static auto splitString(std::string_view in, char sep) {
        std::vector<std::string> r;
        r.reserve(std::count(in.begin(), in.end(), sep) + 1);  // optional
        for (auto p = in.begin();; ++p) {
            auto q = p;
            p = std::find(p, in.end(), sep);
            r.emplace_back(q, p);
            if (p == in.end()) return r;
        }
    }

    /**
     * @brief returns the size of the largest text of the items
     *
     * @param items
     * @return ImVec2
     */
    template <class T>
    static ImVec2 getMaxItemSize(const std::vector<T>& items) {
        constexpr float WIDTH_ADD = 50.0f;
        ImVec2 max_size{0, 0};
        for (const T& item : items) {
            std::string s = std::to_string(item);
            ImVec2 size = ImGui::CalcTextSize(s.c_str());
            if (size.x > max_size.x) {
                max_size = size;
            }
        }
        return {max_size.x + WIDTH_ADD, max_size.y};
    }

    /**
     * @brief see getMaxItemSize(const std::vector<T>& items)
     *
     * @param items
     * @return ImVec2
     */
    static ImVec2 getMaxItemSize(const std::vector<std::string>& items, float width_add = DEFAULT_WIDTH_ADD) {
        ImVec2 max_size{0, 0};
        for (const auto& item : items) {
            ImVec2 size = ImGui::CalcTextSize(item.c_str());
            if (size.x > max_size.x) {
                max_size = size;
            }
        }
        return {max_size.x + width_add, max_size.y};
    }

    /**
     * @brief converts radian angle to degree angle
     *
     * @param rad
     * @return double
     */
    static double rad2Degree(double rad) {
        constexpr double FACTOR = 180;
        return rad / L_PI * FACTOR;
    }

    /**
     * @brief converts degree angle to radian angle
     *
     * @param deg
     * @return double
     */
    static double deg2Radian(double deg) {
        constexpr double FACTOR = 180;
        return deg / FACTOR * L_PI;
    }

   private:
    constexpr static float DEFAULT_WIDTH_ADD = 50.0f;
};
}  // namespace luhsoccer::luhviz
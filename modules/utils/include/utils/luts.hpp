#pragma once

#include <vector>
#include <unordered_map>
#include <logger/logger.hpp>

namespace luhsoccer::util {
class Lut1D {
   public:
    Lut1D() = default;
    ~Lut1D() = default;

    Lut1D(Lut1D&&) = delete;
    Lut1D& operator=(const Lut1D&) = delete;
    Lut1D& operator=(Lut1D&&) = delete;
    Lut1D(const Lut1D&) = delete;

    void load(const std::string& path);
    [[nodiscard]] double interpolate(double x) const;

   private:
    std::vector<double> indices;
    std::vector<double> values;
    bool loaded{false};
    logger::Logger logger{"Lut1D"};
};

class Lut2D {
   public:
    Lut2D() = default;
    ~Lut2D() = default;

    Lut2D(Lut2D&&) = delete;
    Lut2D& operator=(const Lut2D&) = delete;
    Lut2D& operator=(Lut2D&&) = delete;
    Lut2D(const Lut2D&) = delete;

    void load(const std::string& path);
    [[nodiscard]] double interpolate(double x, double y) const;

   private:
    [[nodiscard]] double getValue(size_t x, size_t y) const;

    std::vector<double> x_indices;
    std::vector<double> y_indices;
    std::vector<double> values;
    bool loaded{false};
    logger::Logger logger{"Lut2D"};
};

}  // namespace luhsoccer::util
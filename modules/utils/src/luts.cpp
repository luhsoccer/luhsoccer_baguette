#include "utils/luts.hpp"
#include <algorithm>
#include <fstream>

namespace {

std::pair<size_t, size_t> findIndices(double t, const std::vector<double>& indices) {
    auto upper = std::lower_bound(indices.begin(), indices.end(), t);

    if (upper == indices.begin()) {
        return {0, 0};
    }

    if (upper == indices.end()) {
        return {indices.size() - 1, indices.size() - 1};
    }

    return {std::distance(indices.begin(), upper - 1), std::distance(indices.begin(), upper)};
}

double lutLinear(double t, double left_x, double right_x, double left_y, double right_y) {
    if (left_x == right_x) {
        return left_y;
    }

    double factor = (t - left_x) / (right_x - left_x);

    double interpolated = left_y + factor * (right_y - left_y);

    return interpolated;
}

}  // namespace

namespace luhsoccer::util {
double Lut1D::interpolate(double x) const {
    if (!loaded) {
        LOG_WARNING(logger, "Lut1D not loaded");
        return 0.0;
    }

    const auto [left_index, right_index] = findIndices(x, this->indices);

    double left_x = this->indices[left_index];
    double right_x = this->indices[right_index];
    double left_y = this->values[left_index];
    double right_y = this->values[right_index];

    return lutLinear(x, left_x, right_x, left_y, right_y);
}

void Lut1D::load(const std::string& path) {
    std::ifstream file(path, std::ios_base::in);

    if (!file.is_open()) {
        LOG_WARNING(logger, "Can't open lut file: {}", path);
        return;
    }

    std::string s;
    bool first_line = true;
    while (std::getline(file, s)) {
        if (first_line) {
            first_line = false;
            continue;
        }

        auto comma_index = s.find(',');
        if (comma_index == std::string::npos) {
            continue;
        }

        auto x_value = std::stod(s.substr(0, comma_index));
        auto y_value = std::stod(s.substr(comma_index + 1));

        this->indices.push_back(x_value);
        this->values.push_back(y_value);
    }

    this->loaded = true;
}

double Lut2D::interpolate(double x, double y) const {
    if (!loaded) {
        LOG_WARNING(logger, "Lut2D not loaded");
        return 0.0;
    }

    const auto [x_left_index, x_right_index] = findIndices(x, this->x_indices);
    const auto [y_left_index, y_right_index] = findIndices(y, this->y_indices);

    double x_left_0 = this->x_indices[x_left_index];
    double x_right_0 = this->x_indices[x_right_index];
    double y_left_0 = this->getValue(x_left_index, y_left_index);
    double y_right_0 = this->getValue(x_right_index, y_left_index);

    auto i_0 = lutLinear(x, x_left_0, x_right_0, y_left_0, y_right_0);

    double y_left_1 = this->getValue(x_left_index, y_right_index);
    double y_right_1 = this->getValue(x_right_index, y_right_index);

    auto i_1 = lutLinear(x, x_left_0, x_right_0, y_left_1, y_right_1);

    double x_left_1 = this->y_indices[y_left_index];
    double x_right_1 = this->y_indices[y_right_index];

    auto result = lutLinear(y, x_left_1, x_right_1, i_0, i_1);

    return result;
}

double Lut2D::getValue(size_t x, size_t y) const { return this->values[x * this->y_indices.size() + y]; }

void Lut2D::load(const std::string& path) {
    std::ifstream file(path, std::ios_base::in);

    if (!file.is_open()) {
        LOG_WARNING(logger, "Can't open lut file: {}", path);
        return;
    }

    std::string s;
    bool first_line = true;
    while (std::getline(file, s)) {
        if (first_line) {
            first_line = false;

            std::string header;
            std::stringstream ss(s);

            bool first_header = true;
            while (std::getline(ss, header, ',')) {
                if (first_header) {
                    first_header = false;
                    continue;
                }
                double y_value = 0.0;
                // https://stackoverflow.com/a/15498832
                std::sscanf(header.c_str(), "%*[^0-9]%lf", &y_value);
                this->y_indices.push_back(y_value);
            }

            continue;
        }

        std::string value;
        std::stringstream ss(s);

        bool first_value = true;
        while (std::getline(ss, value, ',')) {
            double x_value = std::stod(value);
            if (first_value) {
                this->x_indices.push_back(x_value);
                first_value = false;
                continue;
            }

            this->values.push_back(x_value);
        }
    }

    this->loaded = true;
}

}  // namespace luhsoccer::util
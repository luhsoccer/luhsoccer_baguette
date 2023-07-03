#include <algorithm>
#include <cmath>
#include <iostream>
#include <ostream>

// TODO: prevent changing struct members

struct ColorRgb {
    double red;
    double green;
    double blue;
    double alpha;

    /**
     * @brief Construct a new Color from rgb values, saved in Range [0,1]. Values greater 255 will be cliped. Negative
     * signs will be ignored.
     *
     * @param red red component
     * @param green green component
     * @param blue blue component range
     * @param alpha alpha componen range
     */
    ColorRgb(double red, double green, double blue, double alpha)
        : red(std::abs(red)), green(std::abs(green)), blue(std::abs(blue)), alpha(std::abs(alpha)) {
        if (this->red > 1.0) this->red = std::min(1.0, this->red / 255);
        if (this->green > 1.0) this->green = std::min(1.0, this->green / 255);
        if (this->blue > 1.0) this->blue = std::min(1.0, this->blue / 255);
        if (this->alpha > 1.0) this->alpha = std::min(1.0, this->alpha / 255);
    }
};

struct ColorHsv {
    double hue;
    double saturation;
    double value;
    double alpha;

    /**
     * @brief Construct a new Color from hsv values, Negative signs will be ignored.
     *
     * @param hue hue component [0-360]
     * @param saturation saturation component [0-1]
     * @param value value component range [0-1]
     * @param alpha alpha componen range [0-1]
     */
    ColorHsv(double hue, double saturation, double value, double alpha)
        : hue(std::abs(int(hue) % 360)),
          saturation(std::abs(saturation)),
          value(std::abs(value)),
          alpha(std::abs(alpha)) {
        if (this->saturation > 1.0) this->saturation = std::min(1.0, this->saturation / 100);
        if (this->value > 1.0) this->value = std::min(1.0, this->value / 100);
        if (this->alpha > 1.0) this->alpha = std::min(1.0, this->alpha / 255);
    }
};

class Color {
   private:
    ColorRgb rgb{0, 0, 0, 0};
    ColorHsv hsv{0, 0, 0, 0};

   public:
    /**
     * @brief Construct a new Color with rgb & hsv
     *
     * @param v1 red or hue component
     * @param v2 green or saturation component
     * @param v3 blue or value component range
     * @param alpha alpha componen range
     */

    Color(double v1, double v2, double v3, double alpha, bool is_rgb) {
        if (is_rgb) {
            this->rgb = {v1, v2, v3, alpha};
            this->hsv = rgb2Hsv(this->rgb);
        } else {
            this->hsv = {v1, v2, v3, alpha};
            this->rgb = hsv2Rgb(this->hsv);
        }
    }

    ColorRgb getColorRgb() { return this->rgb; }

    ColorHsv getColorHsv() { return this->hsv; }

    void setRgb(double r, double g, double b, double a) {
        this->rgb = {r, g, b, a};
        this->hsv = rgb2Hsv(this->rgb);
    }

    void setHsv(double h, double s, double v, double a) {
        this->hsv = {h, s, v, a};
        this->rgb = hsv2Rgb(this->hsv);
    }

    static Color RED() { return {1, 0, 0, 1, true}; }
    static Color GREEN() { return {0, 1, 0, 1, true}; }
    static Color BLUE() { return {0, 0, 1, 1, true}; }
    static Color YELLOW() { return {1, 1, 0, 1, true}; }
    static Color ORANGE() { return {1, 0.6, 0.1, 1, true}; }
    static Color LIGHT_GREEN() { return {0.5, 1, 0.1, 1, true}; }
    static Color LIGHT_BLUE() { return {0.3, 0.6, 1, 1, true}; }
    static Color PURPLE() { return {0.6, 0.2, 0.8, 1, true}; }
    static Color PINK() { return {1, 0.4, 1, 1, true}; }
    static Color GREY() { return {0.4, 0.4, 0.4, 1, true}; }
    static Color LIGHT_GREY() { return {0.7, 0.7, 0.7, 1, true}; }
    static Color WHITE() { return {1, 1, 1, 1, true}; }
    static Color BLACK() { return {0, 0, 0, 1, true}; }

    static Color random() {
        return Color{rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, 1, true};
    }

    static Color random(unsigned int seed) {
        std::srand(seed);
        return Color{rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, 1, true};
    }

    /**
     * @brief Converts rgb to hsv
     */
    static ColorHsv rgb2Hsv(ColorRgb rgb) {
        double r = rgb.red;
        double g = rgb.green;
        double b = rgb.blue;
        double a = rgb.alpha;

        double value = std::max(std::max(r, g), b);
        double cmin = std::min(std::min(r, g), b);
        double delta = value - cmin;
        double hue = 0.0;
        double saturation = 0;

        if (value == 0)
            saturation = 0;
        else
            saturation = delta / value;

        if (delta == 0) {
            hue = 0;
        } else if (r >= g && r >= b) {
            hue = 60 * (std::fmod(((g - b) / delta), 6));
        } else if (g >= r && g >= b) {
            hue = 60 * (((b - r) / delta) + 2);
        } else {
            hue = 60 * (((r - g) / delta) + 4);
        }

        return {hue, saturation, value, a};
    }

    /**
     * @brief Converts hsv to rgb
     */
    static ColorRgb hsv2Rgb(ColorHsv hsv) {
        double hue = hsv.hue;
        double saturation = hsv.saturation;
        double value = hsv.value;
        double a = hsv.alpha;

        double c = value * saturation;
        double x = c * (1 - std::abs(std::fmod((hue / 60), 2) - 1));
        double m = value - c;

        double r = 0, g = 0, b = 0;

        if (hue >= 0 && hue < 60) {
            r = c;
            g = x;
            b = 0;
        } else if (hue >= 60 && hue < 120) {
            r = x;
            g = c;
            b = 0;
        } else if (hue >= 120 && hue < 180) {
            r = 0;
            g = c;
            b = x;
        } else if (hue >= 180 && hue < 240) {
            r = 0;
            g = x;
            b = c;
        } else if (hue >= 240 && hue < 300) {
            r = x;
            g = 0;
            b = c;
        } else if (hue >= 300 && hue < 360) {
            r = c;
            g = 0;
            b = x;
        }

        return {r + m, g + m, b + m, a};
    }
};
struct RGB {
    double r;  // a fraction between 0 and 1
    double g;  // a fraction between 0 and 1
    double b;  // a fraction between 0 and 1
};

struct HSV {
    double h;  // angle in degrees
    double s;  // a fraction between 0 and 1
    double v;  // a fraction between 0 and 1
};
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
inline HSV rgb2hsv(RGB in) {
    HSV out{};

    double min = in.r < in.g ? in.r : in.g;
    min = min < in.b ? min : in.b;

    double max = in.r > in.g ? in.r : in.g;
    max = max > in.b ? max : in.b;

    out.v = max;  // v
    double delta = max - min;
    if (delta < 0.00001) {
        out.s = 0;
        out.h = 0;  // undefined, maybe nan?
        return out;
    }
    if (max > 0.0) {            // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);  // s
    } else {
        // if max is 0, then r = g = b = 0
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;  // its now undefined
        return out;
    }
    if (in.r >= max)                    // > is bogus, just keeps compilor happy
        out.h = (in.g - in.b) / delta;  // between yellow & magenta
    else if (in.g >= max)
        out.h = 2.0 + (in.b - in.r) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + (in.r - in.g) / delta;  // between magenta & cyan

    out.h *= 60.0;  // degrees

    if (out.h < 0.0) out.h += 360.0;

    return out;
}

inline RGB hsv2rgb(HSV in) {
    RGB out{};

    if (in.s <= 0.0) {  // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    double hh = in.h;
    if (hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    long i = (long)hh;
    double ff = hh - static_cast<double>(i);
    double p = in.v * (1.0 - in.s);
    double q = in.v * (1.0 - (in.s * ff));
    double t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch (i) {
        case 0:
            out.r = in.v;
            out.g = t;
            out.b = p;
            break;
        case 1:
            out.r = q;
            out.g = in.v;
            out.b = p;
            break;
        case 2:
            out.r = p;
            out.g = in.v;
            out.b = t;
            break;

        case 3:
            out.r = p;
            out.g = q;
            out.b = in.v;
            break;
        case 4:
            out.r = t;
            out.g = p;
            out.b = in.v;
            break;
        case 5:
        default:
            out.r = in.v;
            out.g = p;
            out.b = q;
            break;
    }
    return out;
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
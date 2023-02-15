#ifndef HELPER
#define HELPER

#include <cmath>
#include <algorithm>

#include "constants.h"
namespace HybridAStar {

namespace Helper {

// 将 t 转换到 (0,360]
static inline float normalizeHeading(float t) {
    if ((int)t <= 0 || (int)t >= 360) {
        if (t < -0.1) {
            t += 360.f;
        }
        else if ((int)t >= 360) {
            t -= 360.f;
        }
        else {
            t =  0;
        }
    }

    return t;
}

// 将 t 转换到 (0,2PI]
static inline float normalizeHeadingRad(float t) {
    if (t < 0) {
        t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
        return 2.f * M_PI + t;
    }

    return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

// 将 t 从 rad 转换到 deg
static inline float toDeg(float t) {
    return normalizeHeadingRad(t) * 180.f / M_PI ;
}

// 将 t 从 deg 转换到 Rad
static inline float toRad(float t) {
    return normalizeHeadingRad(t / 180.f * M_PI);
}

// 将 t 限制在 [lower,upper]
static inline float clamp(float n, float lower, float upper) {
    return std::max(lower, std::min(n, upper));
}

}
}

#endif // HELPER


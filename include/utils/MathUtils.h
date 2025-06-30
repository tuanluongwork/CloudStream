#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <vector>

namespace CloudStream {
namespace MathUtils {

// Constants
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float HALF_PI = 0.5f * PI;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

// Angle conversions
inline float degreesToRadians(float degrees) {
    return degrees * DEG_TO_RAD;
}

inline float radiansToDegrees(float radians) {
    return radians * RAD_TO_DEG;
}

// Clamping
template<typename T>
inline T clamp(T value, T min, T max) {
    return std::max(min, std::min(max, value));
}

// Linear interpolation
template<typename T>
inline T lerp(const T& a, const T& b, float t) {
    return a + (b - a) * t;
}

// Smooth step
inline float smoothstep(float edge0, float edge1, float x) {
    x = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

// Vector operations
inline float length2(const glm::vec3& v) {
    return glm::dot(v, v);
}

inline glm::vec3 safeNormalize(const glm::vec3& v, const glm::vec3& fallback = glm::vec3(0, 1, 0)) {
    float len2 = length2(v);
    if (len2 > 0.0f) {
        return v / std::sqrt(len2);
    }
    return fallback;
}

// Quaternion operations
inline glm::quat rotationBetweenVectors(const glm::vec3& from, const glm::vec3& to) {
    glm::vec3 a = glm::normalize(from);
    glm::vec3 b = glm::normalize(to);
    
    float dot = glm::dot(a, b);
    if (dot > 0.999999f) {
        return glm::quat(1, 0, 0, 0);
    }
    if (dot < -0.999999f) {
        glm::vec3 axis = glm::cross(glm::vec3(1, 0, 0), a);
        if (length2(axis) < 0.00001f) {
            axis = glm::cross(glm::vec3(0, 1, 0), a);
        }
        return glm::angleAxis(PI, glm::normalize(axis));
    }
    
    glm::vec3 axis = glm::cross(a, b);
    float s = std::sqrt((1 + dot) * 2);
    float invs = 1 / s;
    
    return glm::quat(s * 0.5f, axis.x * invs, axis.y * invs, axis.z * invs);
}

// Statistics
template<typename Container>
inline float mean(const Container& values) {
    if (values.empty()) return 0.0f;
    
    float sum = 0.0f;
    for (const auto& v : values) {
        sum += v;
    }
    return sum / values.size();
}

template<typename Container>
inline float variance(const Container& values) {
    if (values.size() < 2) return 0.0f;
    
    float m = mean(values);
    float sum = 0.0f;
    for (const auto& v : values) {
        float diff = v - m;
        sum += diff * diff;
    }
    return sum / (values.size() - 1);
}

template<typename Container>
inline float standardDeviation(const Container& values) {
    return std::sqrt(variance(values));
}

// Random number generation
inline float random01() {
    return static_cast<float>(rand()) / RAND_MAX;
}

inline float randomRange(float min, float max) {
    return min + random01() * (max - min);
}

inline glm::vec3 randomPointInSphere(float radius) {
    float theta = randomRange(0, TWO_PI);
    float phi = std::acos(randomRange(-1, 1));
    float r = radius * std::cbrt(random01());
    
    float sin_phi = std::sin(phi);
    return glm::vec3(
        r * sin_phi * std::cos(theta),
        r * sin_phi * std::sin(theta),
        r * std::cos(phi)
    );
}

} // namespace MathUtils
} // namespace CloudStream 
#pragma once
#include <array>
#include <cstdint>

constexpr double WHEEL_RADIUS = 0.05;
constexpr double BASE_RADIUS  = 0.125;
constexpr double MAX_RAW      = 3000;
constexpr int    WHEEL_IDS[3] = {7, 8, 9};
constexpr double WHEEL_ANGLES_DEG[3] = {240.0, 0.0, 120.0};

std::array<int32_t, 3> body_to_wheel(double x, double y, double theta_deg);
std::array<double, 3> wheel_to_body(int32_t left_raw, int32_t back_raw, int32_t right_raw);

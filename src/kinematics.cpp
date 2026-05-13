#include "kinematics.hpp"
#include <cmath>
#include <algorithm>

// ============================================================
// INVERSE KINEMATICS
//
// We want to move the robot at a desired (x, y, theta) velocity
// in the robot's body frame. But each wheel can only spin around
// its own axis. Inverse kinematics converts "what we want the body
// to do" into "how fast each wheel must spin".
//
// The 3 omniwheels are mounted at 120 apart:
//   Wheel 0 (Left,  ID 7):  240  — points back-left
//   Wheel 1 (Back,  ID 8):    0  — points forward
//   Wheel 2 (Right, ID 9):  120  — points back-right
//
// Each wheel can only produce force along its roller direction,
// which is perpendicular to its mounting angle.
//
// The kinematic matrix M maps body velocities [x, y, theta] to
// wheel linear speeds [v0, v1, v2]:
//
//   [v0]   [cos(a0-90)  sin(a0-90)  R] [x]
//   [v1] = [cos(a1-90)  sin(a1-90)  R] [y]
//   [v2]   [cos(a2-90)  sin(a2-90)  R] [theta]
//
// where ai = wheel mounting angle, R = base radius (distance from
// center to wheel), and the -90 accounts for wheel roller direction
// being perpendicular to the mounting spoke.
//
// Once we have linear speed at each wheel, convert to angular speed
// (w = v / r), then to deg/s, then to Feetech raw units.
// ============================================================

std::array<int32_t, 3> body_to_wheel(double x, double y, double theta_deg) {
    // --- Step 1: Convert rotation velocity to rad/s ---
    // The kinematic matrix expects theta in rad/s because the
    // rotational component R*theta uses radians implicitly.
    double theta_rad = theta_deg * (M_PI / 180.0);

    // --- Step 2: Build the velocity vector in body frame ---
    // [x, y, theta] where x = forward (+), y = left (+)
    double body_vel[3] = {x, y, theta_rad};

    // --- Step 3: Build the 3x3 kinematic matrix M ---
    // For each wheel, a row of M maps body velocities to that
    // wheel's linear speed along its roller direction.
    //
    // Row i: [cos(ai - 90), sin(ai - 90), R]
    //
    // The -90 offset converts from wheel spoke angle to roller
    // direction (rollers are perpendicular to the spoke).
    //
    // The third column is the wheel base radius R — when the
    // robot rotates at theta rad/s, each wheel must contribute
    // R * theta linear speed to produce that rotation.
    double angles_rad[3] = {
        (WHEEL_ANGLES_DEG[0] - 90.0) * (M_PI / 180.0),  // 150
        (WHEEL_ANGLES_DEG[1] - 90.0) * (M_PI / 180.0),  // -90
        (WHEEL_ANGLES_DEG[2] - 90.0) * (M_PI / 180.0),  // 30
    };

    double M[3][3];
    for (int i = 0; i < 3; ++i) {
        M[i][0] = std::cos(angles_rad[i]);
        M[i][1] = std::sin(angles_rad[i]);
        M[i][2] = BASE_RADIUS;  // meters
    }

    // --- Step 4: Multiply M * body_vel to get wheel linear speeds ---
    // wheel_speed[i] = M[i][0]*x + M[i][1]*y + M[i][2]*theta
    double wheel_linear[3];
    for (int i = 0; i < 3; ++i) {
        wheel_linear[i] = M[i][0] * body_vel[0]
                        + M[i][1] * body_vel[1]
                        + M[i][2] * body_vel[2];
    }

    // --- Step 5: Convert linear speed to angular speed ---
    // w = v / r  (rad/s), where r = wheel radius
    double wheel_angular_radps[3];
    for (int i = 0; i < 3; ++i) {
        wheel_angular_radps[i] = wheel_linear[i] / WHEEL_RADIUS;
    }

    // --- Step 6: Convert rad/s -> deg/s -> raw units ---
    // The STS3215 uses a position/speed resolution of 4096 steps
    // per full revolution (360 degrees). So speed in raw units is:
    //   raw = deg/s * (4096 / 360)
    // where (4096/360) = about 11.38 steps per degree.
    double steps_per_deg = 4096.0 / 360.0;
    double raw_double[3];
    for (int i = 0; i < 3; ++i) {
        double wheel_degps = wheel_angular_radps[i] * (180.0 / M_PI);
        raw_double[i] = wheel_degps * steps_per_deg;
    }

    // --- Step 7: Scale down if any wheel exceeds MAX_RAW ---
    // The STS3215 raw velocity range is +/- 32,767, but we cap at
    // MAX_RAW = 3000 for safety (~reasonable max wheel speed).
    // If any wheel exceeds this, scale ALL wheels proportionally
    // so the direction vector is preserved.
    double max_raw = 0.0;
    for (int i = 0; i < 3; ++i) {
        max_raw = std::max(max_raw, std::abs(raw_double[i]));
    }
    if (max_raw > MAX_RAW) {
        double scale = MAX_RAW / max_raw;
        for (int i = 0; i < 3; ++i) {
            raw_double[i] *= scale;
        }
    }

    // --- Step 8: Round to integers and return ---
    return {
        static_cast<int32_t>(std::round(raw_double[0])),  // Left  (ID 7)
        static_cast<int32_t>(std::round(raw_double[1])),  // Back  (ID 8)
        static_cast<int32_t>(std::round(raw_double[2])),  // Right (ID 9)
    };
}


// ============================================================
// FORWARD KINEMATICS
//
// Given measured wheel speeds (from the motor's velocity feedback),
// figure out how fast the robot body is actually moving.
// This is the inverse of body_to_wheel - we invert the matrix.
//
//   [x]           [v_left ]
//   [y] = M_inv * [v_back ]
//   [theta]       [v_right]
//
// where M_inv is the 3x3 inverse of the kinematic matrix above.
// ============================================================

std::array<double, 3> wheel_to_body(int32_t left_raw, int32_t back_raw, int32_t right_raw) {
    double steps_per_deg = 4096.0 / 360.0;

    // --- Step 1: Convert raw -> deg/s -> rad/s -> linear speed ---
    // Reverse the math from body_to_wheel
    int32_t raws[3] = {left_raw, back_raw, right_raw};
    double wheel_linear[3];
    for (int i = 0; i < 3; ++i) {
        double degps = raws[i] / steps_per_deg;          // deg/s
        double radps = degps * (M_PI / 180.0);           // rad/s
        wheel_linear[i] = radps * WHEEL_RADIUS;          // m/s
    }

    // --- Step 2: Build the inverse kinematic matrix ---
    // Same M as above, but we need M_inv = M^-1.
    // Since M IS square (3x3), we compute the standard 3x3 matrix inverse.
    double angles_rad[3] = {
        (WHEEL_ANGLES_DEG[0] - 90.0) * (M_PI / 180.0),
        (WHEEL_ANGLES_DEG[1] - 90.0) * (M_PI / 180.0),
        (WHEEL_ANGLES_DEG[2] - 90.0) * (M_PI / 180.0),
    };

    double M[3][3];
    for (int i = 0; i < 3; ++i) {
        M[i][0] = std::cos(angles_rad[i]);
        M[i][1] = std::sin(angles_rad[i]);
        M[i][2] = BASE_RADIUS;
    }

    // --- Step 3: Compute 3x3 matrix inverse ---
    // For a 3x3 matrix, inverse = adjugate / determinant.
    // We hardcode the formula since our matrix is small and fixed-size.

    // Calculate determinant
    double det = M[0][0] * (M[1][1]*M[2][2] - M[1][2]*M[2][1])
               - M[0][1] * (M[1][0]*M[2][2] - M[1][2]*M[2][0])
               + M[0][2] * (M[1][0]*M[2][1] - M[1][1]*M[2][0]);

    if (std::abs(det) < 1e-12) {
        // Singular matrix - robot would need to be in a degenerate
        // configuration (impossible for omniwheels at 120)
        return {0.0, 0.0, 0.0};
    }

    double inv_det = 1.0 / det;

    // Compute inverse matrix elements
    // M_inv[i][j] = (-1)^(i+j) * det(minor(M, j, i)) / det(M)
    double M_inv[3][3];
    M_inv[0][0] =  (M[1][1]*M[2][2] - M[1][2]*M[2][1]) * inv_det;
    M_inv[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]) * inv_det;
    M_inv[0][2] =  (M[0][1]*M[1][2] - M[0][2]*M[1][1]) * inv_det;
    M_inv[1][0] = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]) * inv_det;
    M_inv[1][1] =  (M[0][0]*M[2][2] - M[0][2]*M[2][0]) * inv_det;
    M_inv[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]) * inv_det;
    M_inv[2][0] =  (M[1][0]*M[2][1] - M[1][1]*M[2][0]) * inv_det;
    M_inv[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]) * inv_det;
    M_inv[2][2] =  (M[0][0]*M[1][1] - M[0][1]*M[1][0]) * inv_det;

    // --- Step 4: Multiply M_inv * wheel_linear to get body velocities ---
    double body_x = M_inv[0][0]*wheel_linear[0] + M_inv[0][1]*wheel_linear[1] + M_inv[0][2]*wheel_linear[2];
    double body_y = M_inv[1][0]*wheel_linear[0] + M_inv[1][1]*wheel_linear[1] + M_inv[1][2]*wheel_linear[2];
    double body_theta_rad = M_inv[2][0]*wheel_linear[0] + M_inv[2][1]*wheel_linear[1] + M_inv[2][2]*wheel_linear[2];

    // --- Step 5: Convert theta from rad/s back to deg/s ---
    double body_theta_deg = body_theta_rad * (180.0 / M_PI);

    return {body_x, body_y, body_theta_deg};
}

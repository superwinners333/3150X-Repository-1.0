#ifndef VIRTUAL_TARGET_PURSUIT_H
#define VIRTUAL_TARGET_PURSUIT_H

#include "movement.hpp"
#include "Odometry.hpp"
#include "helper_functions.hpp"
// #include "PID.h"
#include <deque>
#include <cmath>

// ============================================================================
//  VIRTUAL TARGET PURSUIT — Drift-Compensated Omnidirectional Movement
// ============================================================================
//
//  Problem: On an omni-wheel drivetrain, the robot slides laterally (perpendicular
//           to the drive direction) during curves and high-speed maneuvers. This
//           causes the robot to veer off the desired path.
//
//  Solution: Instead of driving directly to the real target, we generate a
//            "virtual target" that is dynamically shifted to compensate for
//            predicted drift. The robot follows the virtual target, which
//            effectively steers it back onto the correct trajectory.
//
//  Core Loop (every 10ms):
//    1. MEASURE:  Read odometry → current (x, y, θ)
//    2. ESTIMATE: Compute velocity in robot frame → decompose into longitudinal
//                 (forward/back) and LATERAL (drift) components
//    3. PREDICT:  Project where drift will carry us: predicted_pos = pos + v_lat * T
//    4. COMPENSATE: Shift the virtual target OPPOSITE to the predicted drift
//    5. PURSUE:   PID steers the robot toward the virtual target
//    6. REPEAT
//
// ============================================================================

// ─── Configuration Struct ────────────────────────────────────────────────────
struct VTPConfig {
    // Drift estimation
    double drift_gain;            // How aggressively to counter drift [0.5–2.0]
    double prediction_horizon;    // How far ahead to predict drift (seconds) [0.05–0.3]
    int    velocity_filter_size;  // Moving-average window for velocity smoothing [3–8]
    
    // Lateral friction model
    double lateral_friction_coeff; // Coefficient of lateral friction (mu_lat) [0.3–0.8]
    double cornering_stiffness;    // Tire cornering stiffness (N/rad) — how much
                                   // lateral force per degree of slip angle [0.5–2.0]
    
    // Pursuit geometry
    double lookahead_distance;    // How far ahead on path to aim (inches) [4–15]
    double min_lookahead;         // Minimum lookahead when close to target [2–5]
    double curvature_lookahead_scale; // Scale lookahead by 1/(1 + curv * this) [0–5]
    
    // Speed control
    double decel_distance;        // Distance at which to start slowing (inches)
    double min_speed;             // Minimum motor output (prevents stall)
    double max_slew_rate;         // Max voltage change per loop (slew limiting)
    
    // Exit conditions
    double position_tolerance;    // How close is "arrived" (inches) [0.5–2.0]
    double heading_tolerance;     // How close heading must be (degrees) [2–5]
    double settle_time_ms;        // Time to hold within tolerance before exit [50–200]
    double anti_orbit_threshold;  // Distance increase that triggers anti-orbit exit [1–3]
    double decel_rate;            // Deceleration rate for predictive braking (in/s²) [150–400]
};

// Default configuration — a good starting point for a 6-motor omni drive
inline VTPConfig getDefaultVTPConfig() {
    VTPConfig cfg;
    cfg.drift_gain             = 0.08;
    cfg.prediction_horizon     = 0.03;
    cfg.velocity_filter_size   = 8;
    cfg.lateral_friction_coeff = 0.55;
    cfg.cornering_stiffness    = 1.0;
    cfg.lookahead_distance     = 8.0;
    cfg.min_lookahead          = 3.0;
    cfg.curvature_lookahead_scale = 2.0;
    cfg.decel_distance         = 24.0;
    cfg.min_speed              = 4.0;
    cfg.max_slew_rate          = 8.0;
    cfg.position_tolerance     = 2.5;
    cfg.heading_tolerance      = 12.0;
    cfg.settle_time_ms         = 20.0;
    cfg.anti_orbit_threshold   = 0.4;
    cfg.decel_rate             = 150.0;
    return cfg;
}

// ─── Velocity Filter (Moving Average) ────────────────────────────────────────
class VTPVelocityFilter {
    std::deque<double> buffer;
    double sum;
    int max_size;
public:
    VTPVelocityFilter(int size = 5) : sum(0.0), max_size(size) {}
    
    void reset() { buffer.clear(); sum = 0.0; }
    
    void add(double val) {
        buffer.push_back(val);
        sum += val;
        while ((int)buffer.size() > max_size) {
            sum -= buffer.front();
            buffer.pop_front();
        }
    }
    
    double get() const {
        if (buffer.empty()) return 0.0;
        return sum / (double)buffer.size();
    }
};

// ─── Drift Estimator ─────────────────────────────────────────────────────────
// Decomposes the robot's global velocity into longitudinal and lateral 
// components in the robot's local frame. Lateral velocity = drift.
struct DriftEstimate {
    double v_longitudinal;  // Forward/back velocity (robot frame, in/s)
    double v_lateral;       // Side-slip velocity (robot frame, in/s) — THIS IS THE DRIFT
    double v_global_x;      // Smoothed global X velocity (in/s)
    double v_global_y;      // Smoothed global Y velocity (in/s)
    double speed;           // Total speed magnitude (in/s)
    double slip_angle_deg;  // Angle between heading and velocity vector
};

// ─── Orbit Detector ──────────────────────────────────────────────────────────
// Tracks angular momentum and distance divergence to detect when the robot
// is orbiting a target instead of converging. On detection, forces a slow
// direct approach that bypasses VTP compensation — making orbiting IMPOSSIBLE.
struct OrbitDetector {
    double min_dist;       // Minimum distance to current target
    int    div_frames;     // Frames where distance exceeded min + threshold
    int    no_progress;    // Frames with no improvement in min_dist
    bool   direct_mode;    // True = orbit detected, using slow direct approach
    int    grace_frames;   // Frames since last reset — orbit immune during grace period
    double am_hist[50];    // Angular momentum sign history (ring buffer)
    int    am_idx;
    int    am_count;
    
    OrbitDetector() { reset(); }
    
    void reset() {
        min_dist = 999.0; div_frames = 0; no_progress = 0;
        direct_mode = false; grace_frames = 0; am_idx = 0; am_count = 0;
        for (int i = 0; i < 50; i++) am_hist[i] = 0;
    }
    
    void pushAM(double sign) {
        am_hist[am_idx] = sign;
        am_idx = (am_idx + 1) % 50;
        if (am_count < 50) am_count++;
    }
    
    bool consistentAM() {
        if (am_count < 25) return false;
        int n = (am_count < 30) ? am_count : 30;
        int start = (am_idx - n + 50) % 50;
        double dom = am_hist[(am_idx - 1 + 50) % 50];
        int same = 0;
        for (int i = 0; i < n; i++)
            if (am_hist[(start + i) % 50] == dom) same++;
        return same > 26;
    }
};

// ─── Waypoint (for multi-point paths) ────────────────────────────────────────
struct VTPWaypoint {
    double x, y;
    double target_heading;  // Desired heading at this waypoint (degrees, NaN = don't care)
    double max_speed;       // Speed limit for this segment (%)
    bool   brake;           // True = full stop at waypoint, false = chain through
    
    VTPWaypoint() : x(0), y(0), target_heading(NAN), max_speed(80), brake(false) {}
    VTPWaypoint(double _x, double _y, double _heading = NAN, double _speed = 80, bool _brake = false)
        : x(_x), y(_y), target_heading(_heading), max_speed(_speed), brake(_brake) {}
};

// ─── Live Tracking Output ────────────────────────────────────────────────────
// Outputs position data in a format the simulator's Track tab can read.
// Prints "VTP_TRACK:x,y,heading,target_x,target_y" to stdout (VEXcode terminal)
// and displays position/heading/distance on the V5 controller screen.
//
// Usage in your autonomous: VTPTracker::output(target_x, target_y);
//   — OR — tracking is automatic when using driveToPointVTP and variants.
class VTPTracker {
public:
    static void output(double target_x, double target_y);
    static void displayOnController(double target_x, double target_y, double dist);
};

// ─── Function Declarations ───────────────────────────────────────────────────

// Single-point virtual-target drive (most common use case)
extern void driveToPointVTP(
    PIDDataSet   steering_pid,     // PID gains for heading correction
    double       target_x,         // Target X position (inches, field frame)
    double       target_y,         // Target Y position (inches, field frame)
    double       max_speed,        // Maximum speed (negative = reverse)
    double       final_decel_speed,// Speed at the end of deceleration
    double       timeout_ms,       // Maximum time allowed (seconds)
    bool         brake,            // Brake or coast at end
    VTPConfig    cfg = getDefaultVTPConfig()
);

// Single-point with final heading constraint
extern void driveToPointVTPAngle(
    PIDDataSet   steering_pid,
    double       target_x,
    double       target_y,
    double       max_speed,
    double       final_decel_speed,
    double       final_heading_deg, // Desired heading when arriving
    double       timeout_ms,
    bool         brake,
    VTPConfig    cfg = getDefaultVTPConfig()
);

// Multi-waypoint path following with virtual target
extern void followPathVTP(
    PIDDataSet          steering_pid,
    VTPWaypoint*        waypoints,      // Array of waypoints
    int                 waypoint_count, // Number of waypoints
    double              timeout_ms,
    bool                brake,
    VTPConfig           cfg = getDefaultVTPConfig()
);

// Arc/curve movement with drift compensation
extern void curveVTP(
    PIDDataSet   steering_pid,
    double       target_x,
    double       target_y,
    double       end_heading_deg,
    double       lead_distance,     // How far ahead the carrot point leads
    double       max_speed,
    double       final_decel_speed,
    double       timeout_ms,
    bool         brake,
    VTPConfig    cfg = getDefaultVTPConfig()
);

#endif // VIRTUAL_TARGET_PURSUIT_H
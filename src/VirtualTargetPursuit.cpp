// ============================================================================
//  VirtualTargetPursuit.cpp
//  Drift-Compensated Omnidirectional Movement for VEX V5
// ============================================================================
//
//  HOW IT WORKS (detailed):
//
//  Omni-wheel robots have near-zero lateral friction compared to traction wheels.
//  During any maneuver with angular velocity (turning, curved paths, or even
//  heading corrections), centripetal force pushes the chassis sideways. Since
//  omni rollers freely spin perpendicular to the wheel axis, the robot slides.
//
//  Traditional PID heading correction reacts AFTER drift has already occurred,
//  causing a lagging S-curve oscillation. The virtual target approach is
//  PREDICTIVE — it looks at the current drift velocity and anticipates where
//  the robot will be, then shifts the target to pre-compensate.
//
//  Drift compensation pipeline (per loop iteration):
//
//    ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐
//    │  Odometry     │───▶│  Velocity     │───▶│  Robot-Frame     │
//    │  (x, y, θ)   │    │  Estimation   │    │  Decomposition   │
//    └──────────────┘    └──────────────┘    └──────────────────┘
//                                                      │
//                                                      ▼
//    ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐
//    │  Motor       │◀───│  PID Pursuit  │◀───│  Virtual Target  │
//    │  Output      │    │  Controller   │    │  Generation      │
//    └──────────────┘    └──────────────┘    └──────────────────┘
//
//  The "Virtual Target" is the real target PLUS an offset that cancels the
//  predicted lateral displacement:
//
//    virtual_target = real_target - drift_prediction
//
//  Where drift_prediction = v_lateral * prediction_horizon * drift_gain
//  rotated back into the global frame.
//
// ============================================================================

#include "vex.h"
#include "VirtualTargetPursuit.hpp"
#include "math.h"
#include "screen_gui.hpp"
#include "odom.hpp"
#include "movement.hpp"
#include <iostream>
#include <cmath>
#include <cstdio>

using namespace vex;

// ─── Internal Helpers ────────────────────────────────────────────────────────

static inline double vtpClamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline double vtpWrapAngle(double angle) {
    while (angle > 180.0)  angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

static inline double vtpDegToRad(double deg) { return deg * M_PI / 180.0; }
static inline double vtpRadToDeg(double rad) { return rad * 180.0 / M_PI; }

// ─── Drift Estimator Implementation ─────────────────────────────────────────
//
// Given consecutive odometry readings, compute the robot's velocity vector
// in the global frame, then rotate it into the robot's local frame to
// decompose into longitudinal (useful) and lateral (drift) components.
//
//  Robot frame:
//    +X_local = right
//    +Y_local = forward (direction robot faces)
//
//  Rotation from global to robot frame:
//    v_long =  vx_global * sin(θ) + vy_global * cos(θ)
//    v_lat  =  vx_global * cos(θ) - vy_global * sin(θ)
//

static DriftEstimate computeDrift(
    double smoothed_vx, double smoothed_vy,
    double heading_deg,
    double h_tracker_lateral_vel)   // Direct lateral vel from horizontal tracker (in/s)
{
    DriftEstimate est;
    est.v_global_x = smoothed_vx;
    est.v_global_y = smoothed_vy;
    est.speed = hypot(smoothed_vx, smoothed_vy);
    
    double heading_rad = vtpDegToRad(heading_deg);
    
    // Rotate global velocity into robot-local frame
    // Robot faces along +Y in global when heading = 0
    // v_longitudinal = component along robot's forward axis
    // v_lateral      = component perpendicular to robot's forward axis (DRIFT)
    est.v_longitudinal =  smoothed_vx * sin(heading_rad) + smoothed_vy * cos(heading_rad);
    
    // USE HORIZONTAL TRACKER for lateral velocity — this is a DIRECT measurement
    // of how fast the robot slides sideways, far more accurate than differentiating
    // odometry position (which introduces noise and phase lag).
    // The tracker wheel spins when the robot moves perpendicular to its heading.
    est.v_lateral = h_tracker_lateral_vel;
    
    // Slip angle: angle between heading direction and actual velocity direction
    if (est.speed > 0.5) {
        double velocity_angle_deg = vtpRadToDeg(atan2(smoothed_vx, smoothed_vy));
        est.slip_angle_deg = vtpWrapAngle(velocity_angle_deg - heading_deg);
    } else {
        est.slip_angle_deg = 0.0;
    }
    
    return est;
}

// ─── Virtual Target Generator ────────────────────────────────────────────────
//
// Given the drift estimate, project where the drift will carry the robot,
// then shift the target in the OPPOSITE direction to compensate.
//
// The compensation is applied in the robot's local frame (where we know
// which direction is "sideways"), then rotated back to global.
//
// Prediction:
//   drift_displacement_local = (0, v_lateral * T * gain)
//   drift_displacement_global = rotate(drift_displacement_local, heading)
//   virtual_target = real_target - drift_displacement_global
//
// Additional refinement:
//   - Scale compensation by speed (no compensation when stationary)
//   - Reduce compensation when very close to target (prevents overshoot)
//   - Apply a physics-based lateral friction model for better prediction

static void computeVirtualTarget(
    double real_target_x, double real_target_y,
    double robot_x,       double robot_y,
    double heading_deg,
    const DriftEstimate& drift,
    double omega_deg_per_sec,      // Current angular velocity (deg/s)
    const VTPConfig& cfg,
    double& virtual_x,   double& virtual_y)
{
    double heading_rad = vtpDegToRad(heading_deg);
    double dist_to_target = hypot(real_target_x - robot_x, real_target_y - robot_y);
    
    // ── NO centripetal subtraction ──
    // On omni wheels, the rollers allow free lateral movement — the wheels
    // provide ZERO lateral friction. When the robot turns, the velocity
    // vector does NOT follow the heading change. ALL lateral velocity
    // measured by the horizontal tracker IS genuine unwanted drift.
    // (Centripetal subtraction only makes sense for traction-wheel robots
    //  where tire friction redirects the velocity vector during a turn.)
    double v_drift = drift.v_lateral;
    
    double speed = drift.speed;
    
    // Cap drift velocity to prevent runaway compensation
    v_drift = vtpClamp(v_drift, -30.0, 30.0);
    
    double T = cfg.prediction_horizon;
    
    // Predict drift displacement using simple linear model
    double lateral_displacement = v_drift * T * cfg.drift_gain;
    
    // Rotate lateral compensation to global frame (only lateral)
    double comp_x = -lateral_displacement * cos(heading_rad);
    double comp_y =  lateral_displacement * sin(heading_rad);
    
    // Distance fade: smooth S-curve (smoothstep) over 5"
    // Starts fading at 5" but very gradually, preventing the sudden
    // compensation drop-off that causes heading snap at the target.
    //   5" = 100%, 4" = 90%, 3" = 65%, 2" = 35%, 1" = 10%, 0" = 0%
    double t_dist = vtpClamp(dist_to_target / 5.0, 0.0, 1.0);
    double dist_fade = t_dist * t_dist * (3.0 - 2.0 * t_dist);  // smoothstep
    comp_x *= dist_fade;
    comp_y *= dist_fade;
    
    // Speed fade: no compensation when nearly stopped (threshold 3 in/s)
    double speed_fade = vtpClamp(speed / 3.0, 0.0, 1.0);
    comp_x *= speed_fade;
    comp_y *= speed_fade;
    
    // Cap compensation magnitude to 50% of distance to target
    double comp_mag = hypot(comp_x, comp_y);
    double max_comp = dist_to_target * 0.50;
    if (comp_mag > max_comp && comp_mag > 0.01) {
        double scale = max_comp / comp_mag;
        comp_x *= scale;
        comp_y *= scale;
    }
    
    virtual_x = real_target_x + comp_x;
    virtual_y = real_target_y + comp_y;
}

// ─── VTPTracker Implementation ────────────────────────────────────────────────
// Outputs tracking data to VEXcode terminal (readable by simulator's Track tab
// via Web Serial API) and displays on the V5 controller screen.

void VTPTracker::output(double target_x, double target_y) {
    ChassisDataSet vals = ChassisUpdate();
    // Format: VTP_TRACK:x,y,heading,target_x,target_y
    // The simulator's Track tab parses this via USB serial connection
    printf("VTP_TRACK:%.2f,%.2f,%.2f,%.2f,%.2f\n",
           CPos.x, CPos.y, vals.HDG, target_x, target_y);
}

void VTPTracker::displayOnController(double target_x, double target_y, double dist) {
    // V5 controller has 3 lines x ~19 chars
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("X:%.1f Y:%.1f    ", CPos.x, CPos.y);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("H:%.1f D:%.1f    ", ChassisUpdate().HDG, dist);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("T:(%.0f,%.0f)      ", target_x, target_y);
}



// ═════════════════════════════════════════════════════════════════════════════
//  driveToPointVTP — Single point, drift-compensated
// ═════════════════════════════════════════════════════════════════════════════
void driveToPointVTP(
    PIDDataSet   steering_pid,
    double       target_x,
    double       target_y,
    double       max_speed,
    double       final_decel_speed,
    double       timeout_ms,
    bool         brake,
    VTPConfig    cfg)
{
    Brain.Timer.reset();
    
    // Direction
    bool moving_forward = (max_speed >= 0);
    int add = moving_forward > 0 ? 0 : 180;

    double abs_max_speed = fabs(max_speed);
    final_decel_speed = fmin(fabs(final_decel_speed), abs_max_speed);
    
    // Velocity filters
    VTPVelocityFilter vx_filter(cfg.velocity_filter_size);
    VTPVelocityFilter vy_filter(cfg.velocity_filter_size);
    
    // Previous state for velocity computation
    double prev_x    = CPos.x;
    double prev_y    = CPos.y;
    double prev_time = Brain.Timer.value();
    
    // PID state
    double PVal = 0, IVal = 0, DVal = 0;
    ChassisDataSet initVals = ChassisUpdate();
    double init_dx = target_x - CPos.x;
    double init_dy = target_y - CPos.y;
    double init_angle = vtpRadToDeg(atan2(init_dx, init_dy));
    if (!moving_forward) init_angle = vtpWrapAngle(init_angle + 180.0);
    double PrevE = vtpWrapAngle(init_angle - initVals.HDG);
    
    // Exit condition state
    double settle_timer  = 0;
    
    // Orbit detector (replaces old anti-orbit exit)
    OrbitDetector od;
    
    // Slew rate state
    double prev_left_out  = 0;
    double prev_right_out = 0;
    
    // Angular velocity tracking (for centripetal subtraction)
    double prev_heading = ChassisUpdate().HDG;
    
    // Horizontal tracker — direct lateral velocity measurement
    double prev_h_tracker_deg = odomx.position(degrees);
    VTPVelocityFilter h_lat_filter(cfg.velocity_filter_size);
     double exittolerance = 1;
    // Let 2 loops pass to build velocity estimate
    int warmup_loops = 0;
    
    bool perpendicular_line = false, prev_perpendicular_line = true;
    while (Brain.Timer.value() < timeout_ms) {
        ChassisDataSet SensorVals = ChassisUpdate();
        double curr_x     = CPos.x;
        double curr_y     = CPos.y;
        double curr_theta = SensorVals.HDG;
        
        // ── 1. VELOCITY ESTIMATION ──
        double curr_time = Brain.Timer.value();
        double dt = curr_time - prev_time;
        if (dt < 0.005) dt = 0.005; // Safety floor
        
        double raw_vx = (curr_x - prev_x) / dt;
        double raw_vy = (curr_y - prev_y) / dt;
        vx_filter.add(raw_vx);
        vy_filter.add(raw_vy);
        
        // Angular velocity (heading rate, deg/s)
        double omega = vtpWrapAngle(curr_theta - prev_heading) / dt;
        prev_heading = curr_theta;
        
        // Horizontal tracker — direct lateral displacement measurement
        double curr_h_tracker_deg = odomx.position(degrees);
        double delta_h_deg = curr_h_tracker_deg - prev_h_tracker_deg;
        double delta_h_inches = delta_h_deg * horizontal_tracker_diameter * M_PI / 360.0;
        // Subtract the component caused by rotation around the tracker offset
        double delta_heading_rad = vtpDegToRad(omega * dt);
        double rotation_component = horizontal_tracker_dist_from_center * delta_heading_rad;
        double pure_lateral_inches = delta_h_inches - rotation_component;
        double h_lateral_vel = pure_lateral_inches / dt;
        h_lat_filter.add(h_lateral_vel);
        prev_h_tracker_deg = curr_h_tracker_deg;
        
        prev_x    = curr_x;
        prev_y    = curr_y;
        prev_time = curr_time;
        warmup_loops++;
        
        // ── 2. DRIFT ESTIMATION (using horizontal tracker) ──
        DriftEstimate drift = computeDrift(
            vx_filter.get(), vy_filter.get(), curr_theta, h_lat_filter.get());
        
        // ── 3. VIRTUAL TARGET GENERATION ──
        double virtual_x, virtual_y;
        if (warmup_loops < 3) {
            // Not enough data yet — use real target
            virtual_x = target_x;
            virtual_y = target_y;
        } else {
            computeVirtualTarget(
                target_x, target_y,
                curr_x, curr_y,
                curr_theta,
                drift, omega, cfg,
                virtual_x, virtual_y);
        }
        
        // ── 4. DISTANCE CALCULATIONS ──
        double dx_real    = target_x  - curr_x;
        double dy_real    = target_y  - curr_y;
        double dist_real  = hypot(dx_real, dy_real);
        
        double dx_virt    = virtual_x - curr_x;
        double dy_virt    = virtual_y - curr_y;
        double dist_virt  = hypot(dx_virt, dy_virt);
        
        // ── 5. ORBIT DETECTION ──
        // Track minimum distance, angular momentum, and progress to detect orbiting.
        // Three detection conditions make orbiting IMPOSSIBLE:
        //   a) Distance diverged significantly after getting close
        //   b) Consistent angular momentum (circling) with some divergence
        //   c) Extended stagnation (no progress for a long time)
        double spd = drift.speed;
        od.grace_frames++;
        if (dist_real < od.min_dist - 0.05) {
            od.min_dist = dist_real;
            od.div_frames = 0;
            od.no_progress = 0;
        } else {
            od.no_progress++;
            if (dist_real > od.min_dist + cfg.anti_orbit_threshold) od.div_frames++;
        }
        if (spd > 1.0) {
            double am = (curr_x - target_x) * vy_filter.get() - (curr_y - target_y) * vx_filter.get();
            od.pushAM(am > 0 ? 1.0 : -1.0);
        }
        if (!od.direct_mode && od.grace_frames > 80) {
            if (od.div_frames > 35 && od.min_dist < 10.0) od.direct_mode = true;
            if (od.consistentAM() && od.div_frames > 30 && od.min_dist < 10.0) od.direct_mode = true;
            if (od.no_progress > 400 && dist_real < 25.0 && spd > 1.0) od.direct_mode = true;
            if (od.direct_mode)
                std::cout << "VTP: ORBIT detected d=" << dist_real << " min=" << od.min_dist << std::endl;
        }
        // Direct approach mode: bypass VTP, aim at real target, cap speed
        if (od.direct_mode) {
            virtual_x = target_x;
            virtual_y = target_y;
            dx_virt = dx_real; dy_virt = dy_real; dist_virt = dist_real;
            abs_max_speed = fmin(abs_max_speed, 45.0);
            IVal = 0;  // Kill integral windup
            if (dist_real < 1.5) {
                od.reset(); od.min_dist = dist_real;
                std::cout << "VTP: Orbit recovery" << std::endl;
            }
        }
        
        // ── 6. STEERING (aim at VIRTUAL target) ──
        double target_angle = vtpRadToDeg(atan2(dx_virt, dy_virt));
        if (!moving_forward) target_angle = vtpWrapAngle(target_angle + 180.0);
        
        double angle_error = vtpWrapAngle(target_angle - curr_theta);
        
        // PID
        PVal = steering_pid.kp * angle_error;
        IVal += steering_pid.ki * angle_error * dt;
        IVal = vtpClamp(IVal, -10.0, 10.0); // Anti-windup
        // D-term: uses fixed 0.02 divisor to match legacy PID behavior.
        // Legacy PID runs at 20ms and does kd*(error-prev)/0.02.
        // Using variable dt here would amplify kd by 100x, causing oscillation.
        DVal = steering_pid.kd * vtpWrapAngle(angle_error - PrevE) / 0.02;
        DVal = vtpClamp(DVal, -50.0, 50.0);  // Cap D-term to prevent oscillation
        PrevE = angle_error;
        
        double Correction = PVal + IVal + DVal;
        
        // ── STEERING AUTHORITY LIMIT near target ──
        // Clamp max correction based on distance to target.
        // This prevents aggressive PID turns near the target, which on omni
        // wheels create massive lateral thrust → overshoot → orbit.
        // At 4": max correction = 12 (gentle). At 20"+: unclamped.
        double max_correction = vtpClamp(dist_real * 3.0, 3.0, fabs(Correction));
        Correction = vtpClamp(Correction, -max_correction, max_correction);
        
        // Additional fade within 3" — coast in nearly straight
        if (dist_real < 3.0) {
            double steer_fade = vtpClamp(dist_real / 3.0, 0.05, 1.0);
            Correction *= steer_fade;
            IVal *= steer_fade;
        }
        
        // ── ANGULAR VELOCITY LIMITER ──
        // When already spinning fast near the target, scale down correction
        // (if it's driving the spin) and add active counter-spin.
        // Prevents angular momentum runaway → overshoot → orbit.
        // Threshold 90°/s — normal arcs peak ~70-80, dangerous spins hit 150+.
        if (dist_real < cfg.decel_distance * 2.0 && fabs(omega) > 90.0) {
            double excess = omega - copysign(90.0, omega);
            // Only scale down if correction is driving the spin (same sign)
            if (Correction * omega > 0) {
                double omega_scale = vtpClamp(90.0 / fabs(omega), 0.3, 1.0);
                Correction *= omega_scale;
            }
            // Active counter-spin proportional to excess angular velocity
            Correction -= 0.15 * excess;
        }
        
        // ── 7. SPEED CONTROL ──
        // Use real distance for deceleration (not virtual — we care about actual arrival)
        double speed_ratio = vtpClamp(dist_real / cfg.decel_distance, 0.0, 1.0);
        double base_speed = final_decel_speed + (abs_max_speed - final_decel_speed) * speed_ratio;
        
        // Min speed floor
        if (dist_real > 3.0 && base_speed < cfg.min_speed) {
            base_speed = cfg.min_speed;
        }
        
        // ── DRIFT-ADJUSTED DECELERATION ──
        // On omni wheels, lateral drift steals braking authority from the motors.
        // When the robot is sliding sideways, it physically cannot decelerate
        // as fast in the forward direction. Scale effective decel_rate down.
        double drift_fraction = fabs(drift.v_lateral) / fmax(drift.speed, 1.0);
        double effective_decel = cfg.decel_rate * (1.0 - 0.5 * drift_fraction);
        
        // ── PREDICTIVE BRAKING — prevent overshoot ──
        // Uses kinematic envelope: v_max = sqrt(2 * a * d)
        // Max theoretical speed: 3.25" wheels @ 450 RPM = 76.6 in/s
        if (dist_real > 0.3) {
            double approach_vel = (dx_real * vx_filter.get() + dy_real * vy_filter.get()) / dist_real;
            
            // Kinematic envelope with drift-adjusted decel
            double max_approach_vel = sqrt(2.0 * effective_decel * dist_real) * 0.90;
            double max_speed_pct = (max_approach_vel / 76.6) * abs_max_speed;
            base_speed = fmin(base_speed, max_speed_pct);
            
            // Emergency brake — ONLY when actually approaching too fast
            if (approach_vel > max_approach_vel * 2.0 && dist_real < 6.0) {
                base_speed = fmin(base_speed, 5.0);
            }
        }
        
        // ── TOTAL-SPEED BRAKE (braking waypoints only) ──
        // Prevents tangential overshoots where approach_vel ≈ 0 but robot
        // flies past the target at high total speed. Activates at full
        // decel_distance with moderate margin.
        if (brake && dist_real < cfg.decel_distance) {
            double max_total_vel = sqrt(2.0 * effective_decel * dist_real) * 0.75;
            double max_total_pct = (max_total_vel / 76.6) * abs_max_speed;
            base_speed = fmin(base_speed, fmax(max_total_pct, 3.0));
        }
        
        // ── TRACKING OUTPUT (every 50ms) ──
        if (warmup_loops % 5 == 0) {
            VTPTracker::output(target_x, target_y);
            VTPTracker::displayOnController(target_x, target_y, dist_real);
        }
        // ── DEBUG OUTPUT (every 100ms) ──
        if (warmup_loops % 10 == 0) {
            double comp_mag_dbg = hypot(virtual_x - target_x, virtual_y - target_y);
            printf("VTP_DBG:vLat=%.1f comp=%.2f spd=%.0f dist=%.1f base=%.0f aErr=%.1f omega=%.0f\n",
                   drift.v_lateral, comp_mag_dbg, drift.speed, dist_real, base_speed, angle_error, omega);
        }
        
        // Cosine scaling — reduce forward speed when angle error is large
        // Ramps in with distance: no effect far away (robot arcs smoothly),
        // full effect near target (prevents charging in at a bad angle).
        // At 2×decel_distance+: no scaling. At decel_distance: 50% effect. At 0: full.
        double cos_scale = fabs(cos(vtpDegToRad(angle_error)));
        double cos_intensity = vtpClamp(1.0 - dist_real / (cfg.decel_distance * 2.0), 0.0, 1.0);
        base_speed *= (1.0 - cos_intensity * (1.0 - (0.5 + 0.5 * cos_scale)));
        
        // ── 8. MOTOR MIXING ──
        double left_corr  = moving_forward ?  Correction : -Correction;
        double right_corr = moving_forward ? -Correction :  Correction;
        
        double left_speed  = base_speed + left_corr;
        double right_speed = base_speed + right_corr;
        
        if (!moving_forward) {
            left_speed  *= -1.0;
            right_speed *= -1.0;
        }
        
        // Normalize to max speed
        double max_req = fmax(fabs(left_speed), fabs(right_speed));
        if (max_req > abs_max_speed) {
            left_speed  = (left_speed  / max_req) * abs_max_speed;
            right_speed = (right_speed / max_req) * abs_max_speed;
        }
        
        // Slew rate limiting (smooth acceleration)
        // First 5 loops: no slew limit (warm start for smooth chaining)
        double max_delta = warmup_loops < 5 ? 100.0 : cfg.max_slew_rate;
        left_speed  = vtpClamp(left_speed,  prev_left_out  - max_delta, prev_left_out  + max_delta);
        right_speed = vtpClamp(right_speed, prev_right_out - max_delta, prev_right_out + max_delta);
        prev_left_out  = left_speed;
        prev_right_out = right_speed;
        
        // ── 9. OUTPUT ──
        Move(left_speed, right_speed);
         perpendicular_line = ((CPos.y - target_y) * -cos(degToRad(normalizeTarget(SensorVals.HDG + add))) <= (CPos.x - target_x) * sin(degToRad(normalizeTarget(SensorVals.HDG + add))) + exittolerance);
        if(perpendicular_line && !prev_perpendicular_line) {
        break;
        }
        prev_perpendicular_line = perpendicular_line;
        // ── 10. EXIT CONDITIONS ──
        if (dist_real < cfg.position_tolerance) {
            settle_timer += 10;
            if (settle_timer >= cfg.settle_time_ms || !brake) {
              std::cout <<"Settled"<< std::endl;
              break;
            }
        } else {
            settle_timer = 0;
        }
        
        // Chain exit (non-braking)
        if (!brake && dist_real < 3.0) break;
        
        wait(10, msec);
    }
    
    // Log final position
    std::cout << "VTP: Final X=" << CPos.x << " Y=" << CPos.y << "Heading=" << ChassisUpdate().HDG << std::endl;
    if (Brain.Timer.value() >= timeout_ms) std::cout <<"timed out"<< std::endl;
    if (brake) { BStop(); wait(100, msec); }
    else       { CStop(); }
}

// ═════════════════════════════════════════════════════════════════════════════
//  driveToPointVTPAngle — With final heading constraint
// ═════════════════════════════════════════════════════════════════════════════
void driveToPointVTPAngle(
    PIDDataSet   steering_pid,
    double       target_x,
    double       target_y,
    double       max_speed,
    double       final_decel_speed,
    double       final_heading_deg,
    double       timeout_ms,
    bool         brake,
    VTPConfig    cfg)
{
    Brain.Timer.reset();
    
    bool moving_forward = (max_speed >= 0);
    double abs_max_speed = fabs(max_speed);
    final_decel_speed = fmin(fabs(final_decel_speed), abs_max_speed);
    
    // Velocity filters
    VTPVelocityFilter vx_filter(cfg.velocity_filter_size);
    VTPVelocityFilter vy_filter(cfg.velocity_filter_size);
    
    double prev_x = CPos.x, prev_y = CPos.y;
    double prev_time = Brain.Timer.value();
    
    // PID state
    double PVal = 0, IVal = 0, DVal = 0;
    ChassisDataSet initVals = ChassisUpdate();
    double init_angle = vtpRadToDeg(atan2(target_x - CPos.x, target_y - CPos.y));
    if (!moving_forward) init_angle = vtpWrapAngle(init_angle + 180.0);
    double PrevE = vtpWrapAngle(init_angle - initVals.HDG);
    
    double settle_timer  = 0;
    OrbitDetector od;
    double prev_left_out = 0, prev_right_out = 0;
    double prev_heading_angle = ChassisUpdate().HDG;
    int warmup_loops = 0;
    bool heading_phase = false;
    double heading_phase_dist = fmin(cfg.decel_distance * 0.6, 8.0);
    
    // Horizontal tracker — direct lateral velocity measurement
    double prev_h_tracker_deg_a = odomx.position(degrees);
    VTPVelocityFilter h_lat_filter_a(cfg.velocity_filter_size);
    
    while (Brain.Timer.value() < timeout_ms) {
        ChassisDataSet SensorVals = ChassisUpdate();
        double curr_x = CPos.x, curr_y = CPos.y;
        double curr_theta = SensorVals.HDG;
        
        // Velocity estimation
        double curr_time = Brain.Timer.value();
        double dt = curr_time - prev_time;
        if (dt < 0.005) dt = 0.005;
        
        vx_filter.add((curr_x - prev_x) / dt);
        vy_filter.add((curr_y - prev_y) / dt);
        double omega = vtpWrapAngle(curr_theta - prev_heading_angle) / dt;
        prev_heading_angle = curr_theta;
        
        // Horizontal tracker lateral velocity
        double curr_h_deg_a = odomx.position(degrees);
        double dh_deg_a = curr_h_deg_a - prev_h_tracker_deg_a;
        double dh_in_a = dh_deg_a * horizontal_tracker_diameter * M_PI / 360.0;
        double rot_comp_a = horizontal_tracker_dist_from_center * vtpDegToRad(omega * dt);
        double h_lat_vel_a = (dh_in_a - rot_comp_a) / dt;
        h_lat_filter_a.add(h_lat_vel_a);
        prev_h_tracker_deg_a = curr_h_deg_a;
        
        prev_x = curr_x; prev_y = curr_y; prev_time = curr_time;
        warmup_loops++;
        
        // Drift estimation (using horizontal tracker)
        DriftEstimate drift = computeDrift(vx_filter.get(), vy_filter.get(), curr_theta, h_lat_filter_a.get());
        
        // Virtual target
        double virtual_x, virtual_y;
        if (warmup_loops < 3) {
            virtual_x = target_x; virtual_y = target_y;
        } else {
            computeVirtualTarget(target_x, target_y, curr_x, curr_y,
                                 curr_theta, drift, omega, cfg, virtual_x, virtual_y);
        }
        
        // Distances
        double dist_real = hypot(target_x - curr_x, target_y - curr_y);
        double dx_virt = virtual_x - curr_x;
        double dy_virt = virtual_y - curr_y;
        
        // Orbit detection
        double spd = drift.speed;
        od.grace_frames++;
        if (dist_real < od.min_dist - 0.05) {
            od.min_dist = dist_real; od.div_frames = 0; od.no_progress = 0;
        } else {
            od.no_progress++;
            if (dist_real > od.min_dist + cfg.anti_orbit_threshold) od.div_frames++;
        }
        if (spd > 1.0) {
            double am = (curr_x - target_x) * vy_filter.get() - (curr_y - target_y) * vx_filter.get();
            od.pushAM(am > 0 ? 1.0 : -1.0);
        }
        if (!od.direct_mode && od.grace_frames > 80) {
            if (od.div_frames > 35 && od.min_dist < 10.0) od.direct_mode = true;
            if (od.consistentAM() && od.div_frames > 30 && od.min_dist < 10.0) od.direct_mode = true;
            if (od.no_progress > 400 && dist_real < 25.0 && spd > 1.0) od.direct_mode = true;
            if (od.direct_mode)
                std::cout << "VTPAngle: ORBIT detected d=" << dist_real << std::endl;
        }
        if (od.direct_mode) {
            virtual_x = target_x; virtual_y = target_y;
            dx_virt = target_x - curr_x; dy_virt = target_y - curr_y;
            abs_max_speed = fmin(abs_max_speed, 45.0);
            IVal = 0;
            if (dist_real < 1.5) { od.reset(); od.min_dist = dist_real; }
        }
        
        // Phase transition: switch to heading lock when close
        if (!heading_phase && dist_real < heading_phase_dist) {
            heading_phase = true;
            PrevE = vtpWrapAngle(final_heading_deg - curr_theta);
            IVal = 0;
        }
        
        // Steering target
        double target_angle;
        if (heading_phase) {
            target_angle = final_heading_deg;
        } else {
            target_angle = vtpRadToDeg(atan2(dx_virt, dy_virt));
            if (!moving_forward) target_angle = vtpWrapAngle(target_angle + 180.0);
        }
        
        double angle_error = vtpWrapAngle(target_angle - curr_theta);
        
        // PID
        PVal = steering_pid.kp * angle_error;
        IVal += steering_pid.ki * angle_error * dt;
        IVal = vtpClamp(IVal, -10.0, 10.0);
        double d_error = vtpWrapAngle(angle_error - PrevE);
        DVal = steering_pid.kd * d_error / 0.02;
        DVal = vtpClamp(DVal, -50.0, 50.0);
        PrevE = angle_error;
        double Correction = PVal + IVal + DVal;
        
        // ── STEERING AUTHORITY LIMIT ──
        if (!heading_phase) {
            double max_corr_a = vtpClamp(dist_real * 3.0, 3.0, fabs(Correction));
            Correction = vtpClamp(Correction, -max_corr_a, max_corr_a);
        }
        if (dist_real < 3.0 && !heading_phase) {
            double steer_fade = vtpClamp(dist_real / 3.0, 0.05, 1.0);
            Correction *= steer_fade;
            IVal *= steer_fade;
        }
        
        // ── ANGULAR VELOCITY LIMITER ──
        if (dist_real < cfg.decel_distance * 2.0 && fabs(omega) > 90.0 && !heading_phase) {
            double excess = omega - copysign(90.0, omega);
            if (Correction * omega > 0) {
                double omega_scale = vtpClamp(90.0 / fabs(omega), 0.3, 1.0);
                Correction *= omega_scale;
            }
            Correction -= 0.15 * excess;
        }
        
        // Speed
        double speed_ratio = vtpClamp(dist_real / cfg.decel_distance, 0.0, 1.0);
        double base_speed = final_decel_speed + (abs_max_speed - final_decel_speed) * speed_ratio;
        if (dist_real > 3.0 && base_speed < cfg.min_speed) base_speed = cfg.min_speed;
        
        // ── DRIFT-ADJUSTED DECELERATION ──
        double drift_frac_a = fabs(drift.v_lateral) / fmax(drift.speed, 1.0);
        double effective_decel_a = cfg.decel_rate * (1.0 - 0.5 * drift_frac_a);
        
        // ── PREDICTIVE BRAKING ──
        if (dist_real > 0.3) {
            double approach_vel_a = ((target_x - curr_x) * vx_filter.get()
                                  + (target_y - curr_y) * vy_filter.get()) / dist_real;
            double max_av = sqrt(2.0 * effective_decel_a * dist_real) * 0.90;
            base_speed = fmin(base_speed, (max_av / 76.6) * abs_max_speed);
            if (approach_vel_a > max_av * 2.0 && dist_real < 6.0) base_speed = fmin(base_speed, 5.0);
        }
        
        // Total-speed brake (braking waypoints) — earlier + moderate margin
        if (brake && dist_real < cfg.decel_distance) {
            double max_tv = sqrt(2.0 * effective_decel_a * dist_real) * 0.75;
            base_speed = fmin(base_speed, fmax((max_tv / 76.6) * abs_max_speed, 3.0));
        }
        
        // Tracking output
        if (warmup_loops % 5 == 0) {
            VTPTracker::output(target_x, target_y);
            VTPTracker::displayOnController(target_x, target_y, dist_real);
        }
        
        // Motor mixing
        double left_corr  = moving_forward ?  Correction : -Correction;
        double right_corr = moving_forward ? -Correction :  Correction;
        double left_speed  = base_speed + left_corr;
        double right_speed = base_speed + right_corr;
        if (!moving_forward) { left_speed *= -1.0; right_speed *= -1.0; }
        
        double max_req = fmax(fabs(left_speed), fabs(right_speed));
        if (max_req > abs_max_speed) {
            left_speed  = (left_speed  / max_req) * abs_max_speed;
            right_speed = (right_speed / max_req) * abs_max_speed;
        }
        
        // Slew (warm start for first 5 loops)
        double max_delta = warmup_loops < 5 ? 100.0 : cfg.max_slew_rate;
        left_speed  = vtpClamp(left_speed,  prev_left_out  - max_delta, prev_left_out  + max_delta);
        right_speed = vtpClamp(right_speed, prev_right_out - max_delta, prev_right_out + max_delta);
        prev_left_out = left_speed; prev_right_out = right_speed;
        
        Move(left_speed, right_speed);
        
        // Exit: position AND heading
        double heading_error = fabs(vtpWrapAngle(final_heading_deg - curr_theta));
        if (dist_real < cfg.position_tolerance && heading_error < cfg.heading_tolerance) {
            settle_timer += 10;
            if (settle_timer >= cfg.settle_time_ms) break;
        } else {
            settle_timer = 0;
        }
        
        if (!brake && dist_real < 3.0) break;
        
        wait(10, msec);
    }
    
    std::cout << "VTPAngle: Final X=" << CPos.x << " Y=" << CPos.y 
              << " H=" << ChassisUpdate().HDG << std::endl;
    if (brake) { BStop(); wait(130, msec); } else { CStop(); }
}

// ═════════════════════════════════════════════════════════════════════════════
//  followPathVTP — Multi-waypoint path following
// ═════════════════════════════════════════════════════════════════════════════
void followPathVTP(
    PIDDataSet          steering_pid,
    VTPWaypoint*        waypoints,
    int                 waypoint_count,
    double              timeout_ms,
    bool                brake,
    VTPConfig           cfg)
{
    if (waypoint_count <= 0) return;
    
    // Drive to each waypoint in sequence
    // All but the last use chain-exit (no brake)
    for (int i = 0; i < waypoint_count; i++) {
        bool is_last = (i == waypoint_count - 1);
        bool wp_brake = is_last ? brake : false;
        
        VTPWaypoint& wp = waypoints[i];
        
        if (!std::isnan(wp.target_heading)) {
            // Waypoint has a heading constraint
            driveToPointVTPAngle(
                steering_pid,
                wp.x, wp.y,
                wp.max_speed,
                is_last ? 10.0 : 20.0,  // Higher chain speed for intermediate points
                wp.target_heading,
                timeout_ms,
                wp_brake,
                cfg
            );
        } else {
            // No heading constraint
            driveToPointVTP(
                steering_pid,
                wp.x, wp.y,
                wp.max_speed,
                is_last ? 10.0 : 20.0,
                timeout_ms,
                wp_brake,
                cfg
            );
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  curveVTP — Arc/curve with drift compensation
// ═════════════════════════════════════════════════════════════════════════════
//
//  Uses a "carrot point" (lead point) along the direction the robot should
//  face at arrival. As the robot approaches, the carrot converges to the
//  target. Drift compensation shifts the carrot to counter lateral slip
//  during the curve.
//
void curveVTP(
    PIDDataSet   steering_pid,
    double       target_x,
    double       target_y,
    double       end_heading_deg,
    double       lead_distance,
    double       max_speed,
    double       final_decel_speed,
    double       timeout_ms,
    bool         brake,
    VTPConfig    cfg)
{
    Brain.Timer.reset();
    
    bool moving_forward = (max_speed >= 0);
    int direction = moving_forward ? 1 : -1;
    double abs_max_speed = fabs(max_speed);
    final_decel_speed = fmin(fabs(final_decel_speed), abs_max_speed);
    
    // Velocity filters
    VTPVelocityFilter vx_filter(cfg.velocity_filter_size);
    VTPVelocityFilter vy_filter(cfg.velocity_filter_size);
    
    double prev_x = CPos.x, prev_y = CPos.y;
    double prev_time = Brain.Timer.value();
    
    // PID state
    double PVal = 0, IVal = 0, DVal = 0, PrevE = 0;
    double prev_left_out = 0, prev_right_out = 0;
    double settle_timer = 0;
    OrbitDetector od;
    int warmup_loops = 0;
    bool is_close = false;
    
    // Angular velocity tracking (for centripetal subtraction)
    double prev_heading_curve = ChassisUpdate().HDG;
    
    // Horizontal tracker — direct lateral velocity measurement
    double prev_h_tracker_deg_c = odomx.position(degrees);
    VTPVelocityFilter h_lat_filter_c(cfg.velocity_filter_size);
    
    while (Brain.Timer.value() < timeout_ms) {
        ChassisDataSet SensorVals = ChassisUpdate();
        double curr_x = CPos.x, curr_y = CPos.y;
        double curr_theta = SensorVals.HDG;
        
        // ── Velocity estimation ──
        double curr_time = Brain.Timer.value();
        double dt = curr_time - prev_time;
        if (dt < 0.005) dt = 0.005;
        
        vx_filter.add((curr_x - prev_x) / dt);
        vy_filter.add((curr_y - prev_y) / dt);
        double omega = vtpWrapAngle(curr_theta - prev_heading_curve) / dt;
        prev_heading_curve = curr_theta;
        
        // Horizontal tracker lateral velocity
        double curr_h_deg_c = odomx.position(degrees);
        double dh_deg_c = curr_h_deg_c - prev_h_tracker_deg_c;
        double dh_in_c = dh_deg_c * horizontal_tracker_diameter * M_PI / 360.0;
        double rot_comp_c = horizontal_tracker_dist_from_center * vtpDegToRad(omega * dt);
        double h_lat_vel_c = (dh_in_c - rot_comp_c) / dt;
        h_lat_filter_c.add(h_lat_vel_c);
        prev_h_tracker_deg_c = curr_h_deg_c;
        
        prev_x = curr_x; prev_y = curr_y; prev_time = curr_time;
        warmup_loops++;
        
        // ── Drift estimation (using horizontal tracker) ──
        DriftEstimate drift = computeDrift(vx_filter.get(), vy_filter.get(), curr_theta, h_lat_filter_c.get());
        
        // ── Distance to real target ──
        double dist_to_target = hypot(target_x - curr_x, target_y - curr_y);
        
        if (dist_to_target < 5.0) is_close = true;
        if (!brake && dist_to_target < 3.0) break;
        
        // Orbit detection
        double spd = drift.speed;
        od.grace_frames++;
        if (dist_to_target < od.min_dist - 0.05) {
            od.min_dist = dist_to_target; od.div_frames = 0; od.no_progress = 0;
        } else {
            od.no_progress++;
            if (dist_to_target > od.min_dist + cfg.anti_orbit_threshold) od.div_frames++;
        }
        if (spd > 1.0) {
            double am = (curr_x - target_x) * vy_filter.get() - (curr_y - target_y) * vx_filter.get();
            od.pushAM(am > 0 ? 1.0 : -1.0);
        }
        if (!od.direct_mode && od.grace_frames > 80) {
            if (od.div_frames > 35 && od.min_dist < 10.0) od.direct_mode = true;
            if (od.consistentAM() && od.div_frames > 30 && od.min_dist < 10.0) od.direct_mode = true;
            if (od.no_progress > 400 && dist_to_target < 25.0 && spd > 1.0) od.direct_mode = true;
            if (od.direct_mode)
                std::cout << "CurveVTP: ORBIT detected d=" << dist_to_target << std::endl;
        }
        
        // ── Carrot point computation ──
        // When in orbit direct mode, skip carrot and go straight to target
        double virtual_carrot_x, virtual_carrot_y;
        if (od.direct_mode) {
            virtual_carrot_x = target_x;
            virtual_carrot_y = target_y;
            abs_max_speed = fmin(abs_max_speed, 45.0);
            IVal = 0;
            if (dist_to_target < 1.5) {
                od.reset(); od.min_dist = dist_to_target;
                std::cout << "CurveVTP: Orbit recovery" << std::endl;
            }
        } else {
            // Carrot sits along the end_heading direction, behind the target
            // As dist decreases, lead shrinks → carrot converges to target
            double effective_lead = lead_distance * vtpClamp(dist_to_target / 20.0, 0.1, 1.0);
            effective_lead = fmax(effective_lead, 2.0);
            
            double end_rad = vtpDegToRad(end_heading_deg);
            double carrot_x = target_x - effective_lead * sin(end_rad);
            double carrot_y = target_y - effective_lead * cos(end_rad);
            
            // ── Apply drift compensation to carrot ──
            if (warmup_loops < 3) {
                virtual_carrot_x = carrot_x;
                virtual_carrot_y = carrot_y;
            } else {
                computeVirtualTarget(
                    carrot_x, carrot_y,
                    curr_x, curr_y,
                    curr_theta, drift, omega, cfg,
                    virtual_carrot_x, virtual_carrot_y);
            }
        }
        
        // ── Steering ──
        double angle_to_carrot = vtpRadToDeg(atan2(
            virtual_carrot_x - curr_x,
            virtual_carrot_y - curr_y));
        
        double drive_heading = angle_to_carrot;
        if (direction == -1) drive_heading = vtpWrapAngle(angle_to_carrot + 180.0);
        
        double angle_error = vtpWrapAngle(drive_heading - curr_theta);
        
        // PID
        PVal = steering_pid.kp * angle_error;
        IVal += steering_pid.ki * angle_error * dt;
        IVal = vtpClamp(IVal, -10.0, 10.0);
        double d_error = vtpWrapAngle(angle_error - PrevE);
        DVal = steering_pid.kd * d_error / 0.02;
        DVal = vtpClamp(DVal, -50.0, 50.0);
        PrevE = angle_error;
        
        double angular_out = PVal + IVal + DVal;
        
        // ── ANGULAR VELOCITY LIMITER ──
        if (dist_to_target < cfg.decel_distance * 2.0 && fabs(omega) > 90.0) {
            double excess = omega - copysign(90.0, omega);
            if (angular_out * omega > 0) {
                double omega_scale = vtpClamp(90.0 / fabs(omega), 0.3, 1.0);
                angular_out *= omega_scale;
            }
            angular_out -= 0.15 * excess;
        }
        
        // ── Speed control ──
        double speed_ratio = vtpClamp(dist_to_target / cfg.decel_distance, 0.0, 1.0);
        double lateral_out = final_decel_speed + (abs_max_speed - final_decel_speed) * speed_ratio;
        
        // Predictive braking for curve
        // Lateral drift handled by VTP + horizontal tracking wheel
        if (dist_to_target > 0.3) {
            double max_av = sqrt(2.0 * cfg.decel_rate * dist_to_target) * 0.95;
            double max_pct = (max_av / 76.6) * abs_max_speed;
            lateral_out = fmin(lateral_out, max_pct);
        }
        
        // Tracking output
        if (warmup_loops % 5 == 0) {
            VTPTracker::output(target_x, target_y);
            VTPTracker::displayOnController(target_x, target_y, dist_to_target);
        }
        
        if (!is_close) lateral_out += cfg.min_speed * 0.3; // Feedforward
        
        // Cosine scaling for large angle errors
        double cos_scale = fabs(cos(vtpDegToRad(angle_error)));
        lateral_out *= (0.5 + 0.5 * cos_scale);
        
        // Apply direction
        if (direction == -1) lateral_out = -fabs(lateral_out);
        else lateral_out = fabs(lateral_out);
        
        // Prevent overturn
        double overturn = fabs(lateral_out) + fabs(angular_out) - abs_max_speed;
        if (overturn > 0) {
            lateral_out -= (lateral_out > 0 ? overturn : -overturn);
        }
        
        // Motor output
        double left_speed  = lateral_out + angular_out;
        double right_speed = lateral_out - angular_out;
        
        // Slew rate limiting (warm start for first 5 loops)
        double max_delta = warmup_loops < 5 ? 100.0 : cfg.max_slew_rate;
        left_speed  = vtpClamp(left_speed,  prev_left_out  - max_delta, prev_left_out  + max_delta);
        right_speed = vtpClamp(right_speed, prev_right_out - max_delta, prev_right_out + max_delta);
        prev_left_out = left_speed; prev_right_out = right_speed;
        
        Move(left_speed, right_speed);
        
        // ── Exit ──
        if (brake && is_close) {
            double heading_err = fabs(vtpWrapAngle(end_heading_deg - curr_theta));
            if (dist_to_target < cfg.position_tolerance && heading_err < cfg.heading_tolerance) {
                settle_timer += 10;
                if (settle_timer > cfg.settle_time_ms) break;
            } else {
                settle_timer = 0;
            }
        }
        
        wait(10, msec);
    }
    
    std::cout << "CurveVTP: Final X=" << CPos.x << " Y=" << CPos.y 
              << " H=" << ChassisUpdate().HDG << std::endl;
    if (brake) { BStop(); wait(100, msec); } else { CStop(); }
}
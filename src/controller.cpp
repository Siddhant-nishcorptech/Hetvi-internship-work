/**
 * @file    controller_node.cpp
 * @brief   ROS 2 node: Central motion controller for the stair-climbing
 *          wheelchair robot – integrates body levelling, inverse kinematics,
 *          mode blending, and stair-climbing state machine.
 *
 *  ┌──────────────┐   /imu/data        ┌──────────────────────────────────┐
 *  │  IMU         │──────────────────►│                                  │
 *  └──────────────┘                    │         Controller Node           │
 *                                      │                                  │
 *  ┌──────────────┐   /joint_states    │  1. Pitch/Roll PID  (levelling)  │
 *  │ ros2_control │──────────────────►│  2. 2-DOF IK        (leg height) │
 *  └──────────────┘                    │  3. Mode blend α    (wheel↔track)│
 *                                      │  4. Climb FSM       (stair climb) │
 *  ┌──────────────┐   /cmd_vel         │                                  │
 *  │  Teleop / Nav│──────────────────►│                                  │
 *  └──────────────┘                    └───────────┬───────────────────────┘
 *                                                  │
 *  ┌──────────────┐   /step_state                  │ publishes to
 *  │StairDetector │──────────────────►            │
 *  │              │   /stair_info                  ▼
 *  └──────────────┘              /wheel_controller/cmd_vel  (TwistStamped)
 *                                /joint_controller/commands (effort torques)
 *                                /track_right/speed         (Float64)
 *                                /track_left/speed          (Float64)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * LOCOMOTION MODES  (controlled by `transition_alpha_` ∈ [0, 1])
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   α = 1.0  →  WHEELED mode
 *       Diff-drive velocity commands sent to /wheel_controller/cmd_vel.
 *       Leg joints held at IK-derived angles for chosen body height H.
 *       Active roll+pitch levelling with pitch_pid_ and roll_pid_.
 *
 *   α = 0.0  →  TRACK mode
 *       Track surface speeds sent to /track_right/speed and /track_left/speed.
 *       Leg joints held at fixed compact angles (H_track = 0.30 m, x = −0.08 m).
 *       Stiffer track_pitch_pid_ and track_roll_pid_ used for levelling.
 *
 *   0 < α < 1  →  TRANSITION (blended commands – both actuator groups active)
 *       alpha changes at 0.10 per second  (hard-coded step in control_loop).
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * STAIR-CLIMBING STATE MACHINE  (climb_override_ = true)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   IDLE ──► EXTEND_BACK (150 ticks ≈ 1.5 s)
 *       Back-leg knee joints driven to −0.4 rad, pushing the front of the
 *       robot up and forward onto the first step.
 *
 *   EXTEND_BACK ──► TRACK_CLIMB (250 ticks ≈ 2.5 s)
 *       Tracks driven backward at -0.6 m/s.
 *
 *   TRACK_CLIMB ──► PULL_UP
 *       Knee joints smoothly interpolate back to pre-climb cached angles
 *       (5 % lerp per tick) until within 0.05 rad of target → IDLE.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * INVERSE KINEMATICS  (2-DOF per leg, planar)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   Given desired foot position (x, z) relative to the hip joint:
 *     r  = √(x² + z²)
 *     q2 = −(π − arccos((L1²+L2²−r²)/(2·L1·L2)))  ← knee angle
 *     q1 = atan2(x, −z) − arccos((L1²+r²−L2²)/(2·L1·r))  ← hip angle
 *
 *   Link lengths:   L1 = 0.2442 m (upper),  L2 = 0.175 m (lower)
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <algorithm>

struct PID {
    double kp{0}, ki{0}, kd{0};
    double integral{0}, prev_error{0};
    double d_filtered{0};     //Low-pass filtered derivative term

    void set_gains(double p, double i, double d) { kp = p; ki = i; kd = d; }

    void reset() { integral = 0; prev_error = 0; d_filtered = 0; }

    /**
     * @brief  Compute PID output for one timestep.
     * @param setpoint  Desired value (typically 0.0 for levelling)
     * @param input     Measured value (IMU roll or pitch in radians)
     * @param dt        Time since last call [s]
     * @return          Control output (correction in radians or m/s depending on use)
     */
    double compute(double setpoint, double input, double dt) {
        double error = setpoint - input;

        // Anti-windup: decay integral near setpoint
        if      (std::fabs(error) < 0.005) integral *= 0.60;
        else if (std::fabs(error) < 0.4)   integral += error * dt;
        integral = std::clamp(integral, -0.1, 0.1);

        // Derivative with low-pass filter (α = 0.3)
        double raw_deriv = (error - prev_error) / dt;
        d_filtered = (0.7 * d_filtered) + (0.3 * raw_deriv);
        prev_error = error;

        return (kp * error) + (ki * integral) + (kd * d_filtered);
    }
};

struct JointPD {
    double kp{0}, kd{0};
    double torque_limit{100.0}; 

    /**
     * @param q_des  Desired joint angle [rad]
     * @param q      Current joint angle [rad]  (from /joint_states)
     * @param qd     Current joint velocity [rad/s] (from /joint_states)
     * @return       Commanded torque [Nm], clamped to ±torque_limit
     */
    double compute(double q_des, double q, double qd) {
        double tau = kp * (q_des - q) - kd * qd;
        return std::clamp(tau, -torque_limit, torque_limit);
    }
};

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("controller_node")
    {
        this->declare_parameter("auto_level",    true);   
        this->declare_parameter("manual_control", false); 

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&Controller::read_joint_angles, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            std::bind(&Controller::imuCallback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&Controller::cmdVelCallback, this, std::placeholders::_1));

        step_state_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/step_state", 10,
            std::bind(&Controller::stepStateCallback, this, std::placeholders::_1));

        stair_info_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/stair_info", 10,
            std::bind(&Controller::stairInfoCallback, this, std::placeholders::_1));

        wheel_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/wheel_controller/cmd_vel", 10);

        joint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_controller/commands", 10);

        track_right_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/track_right/speed", 10);

        track_left_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/track_left/speed", 10);

        current_joint_state.resize(4, 0.0);  // [R_hip, R_knee, L_hip, L_knee]

        L1          = 0.2442;   // Upper leg link length [m]
        L2          = 0.175;    // Lower leg link length [m]
        x0          = 0.0;      // Nominal horizontal foot offset [m]
        track_width = 0.2208;   // Distance between left and right track centrelines [m]

        H     = 0.32;
        H_min = 0.14;
        H_max = 0.40;

        pitch_pid_.set_gains(1.2, 0.1, 0.05);
        roll_pid_.set_gains(0.8, 0.05, 0.01);

        track_pitch_pid_.set_gains(2.5, 0.1, 0.08);
        track_roll_pid_.set_gains(0.1, 0.0, 0.05);


        hip_pd_.kp   = 60.0;  hip_pd_.kd   = 3.0;
        knee_pd_.kp  = 50.0;  knee_pd_.kd  = 2.0;

        auto_level_enabled_ = this->get_parameter("auto_level").as_bool();
        manual_control_     = this->get_parameter("manual_control").as_bool();

        param_cb_ = this->add_on_set_parameters_callback(
            std::bind(&Controller::paramCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Controller::control_loop, this));

        last_time_ = this->now();
    }

private:

    rcl_interfaces::msg::SetParametersResult
    paramCallback(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &p : params) {
            if (p.get_name() == "manual_control") {
                manual_control_ = p.as_bool();
                RCLCPP_WARN(this->get_logger(), "manual_control changed to %s",
                            manual_control_ ? "TRUE" : "FALSE");
            }
            if (p.get_name() == "auto_level") {
                auto_level_enabled_ = p.as_bool();
                roll_pid_.reset();
                pitch_pid_.reset();
                RCLCPP_WARN(this->get_logger(), "auto_level changed to %s",
                            auto_level_enabled_ ? "TRUE" : "FALSE");
            }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void read_joint_angles(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "revolute_1") {
                current_joint_state[0] = msg->position[i];
                joint_velocity_[0]     = msg->velocity[i];
            }
            if (msg->name[i] == "revolute_2") {
                current_joint_state[1] = msg->position[i];
                joint_velocity_[1]     = msg->velocity[i];
            }
            if (msg->name[i] == "revolute_5") {
                current_joint_state[2] = -msg->position[i];  
                joint_velocity_[2]     = -msg->velocity[i];
            }
            if (msg->name[i] == "revolute_6") {
                current_joint_state[3] = -msg->position[i];  
                joint_velocity_[3]     = -msg->velocity[i];
            }
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);

        // Low-pass filter: α = 0.1 (slow update – smooth steady state)
        imu_roll_  = (alpha * r) + (1.0 - alpha) * imu_roll_;
        imu_pitch_ = (alpha * p) + (1.0 - alpha) * imu_pitch_;

        imu_received_ = true;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (msg->linear.y > 0.0) {
            track_mode_ = !track_mode_;
            RCLCPP_INFO(this->get_logger(), "Track mode toggled: %s",
                        track_mode_ ? "ON" : "OFF");
            return;
        }

        if (msg->linear.y < 0.0) {
            if (climb_override_) { return; }  
            climb_override_ = false; //true for automatically switching to backleg logic after reaching platform
            climb_state_    = ClimbState::EXTEND_BACK;
            climb_timer_    = 0;
            RCLCPP_INFO(this->get_logger(), "Backleg logic started!");
            return;
        }

        vx_ = msg->linear.x;
        wz_ = msg->angular.z;

        if (manual_control_) {
            manual_roll_  += msg->angular.x * 0.03;
            manual_pitch_ += msg->angular.y * 0.01;
        }

        H += msg->linear.z * 0.01;
        H  = std::clamp(H, H_min, H_max);
    }

    void stepStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int state = msg->data;
        current_step_state_ = state;

        if (state == 1) {  // FIRST_STEP – begin approach
            is_transitioning_ = true;
            if (!track_mode_) {
                track_mode_ = true;
            }
        } else if (state == 3) {  // REACHED_PLATFORM – begin pull-up
            RCLCPP_INFO(this->get_logger(), "Reached Platform! Ready for backleg logic.");
            if (!climb_override_) {
                climb_override_ = false; //true for automatically switching to backleg logic after reaching platform
                climb_state_    = ClimbState::EXTEND_BACK;
                climb_timer_    = 0;
            }
        }

    void stairInfoCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 8) {
            detected_stairs_count_ = static_cast<int>(msg->data[0]);
            if (msg->data[0] >= 1.0) {
                has_stair_info_      = true;
                last_stair_info_time_ = this->now();
            }
        }
    }

    //  2-DOF Inverse Kinematics
    /**
     * Closed-form IK for a planar 2-link leg.
     *
     * Given desired foot position in the hip frame:
     *   x = forward offset [m] (positive forward)
     *   z = vertical offset [m] (negative = downward)
     *
     * Outputs:
     *   q1 = hip joint angle  [rad]
     *   q2 = knee joint angle [rad]
     *
     * Derivation uses the cosine rule:
     *   cos(θ_knee) = (L1²+L2²−r²)/(2·L1·L2)
     *   q2 = −(π − arccos(cos_θ))   ← negative = bent knee pointing backward
     *   q1 = atan2(x,−z) − arccos((L1²+r²−L2²)/(2·L1·r))
     *
     * cos_theta is clamped to [−1, 1] to guard against numerical errors
     * when the target is at the kinematic boundary.
     */
    void compute_ik(double x, double z, double &q1, double &q2)
    {
        double r = std::sqrt(x * x + z * z);

        double cos_theta = (L1 * L1 + L2 * L2 - r * r) / (2.0 * L1 * L2);
        cos_theta = std::clamp(cos_theta, -1.0, 1.0);

        q2 = -(M_PI - std::acos(cos_theta));  
        double alpha = std::atan2(x, -z);
        double beta  = std::acos((L1 * L1 + r * r - L2 * L2) / (2.0 * L1 * r));
        q1 = alpha - beta;
    }

    void update_climb_sequence()
    {
        climb_timer_++;
        switch (climb_state_) {

            case ClimbState::EXTEND_BACK:
                q2_climb_r_ = -0.4;
                q2_climb_l_ = -0.4;
                if (climb_timer_ > 150) {
                    climb_state_ = ClimbState::TRACK_CLIMB;
                    climb_timer_ = 0;
                }
                break;

            case ClimbState::TRACK_CLIMB:
                climb_vx_ = -0.6;
                if (climb_timer_ > 250) {
                    climb_state_ = ClimbState::PULL_UP;
                    climb_timer_ = 0;
                }
                break;

            case ClimbState::PULL_UP:
                q2_climb_r_ += (q2_w_r_cached_ - q2_climb_r_) * 0.05;
                q2_climb_l_ += (q2_w_l_cached_ - q2_climb_l_) * 0.05;
                if (std::fabs(q2_climb_r_ - q2_w_r_cached_) < 0.05) {
                    transition_alpha_ = 1.0;
                    climb_override_   = false;
                    track_mode_       = false;
                    climb_vx_         = 0.0;
                    climb_timer_      = 0;
                    climb_state_      = ClimbState::IDLE;
                }
                break;

            default:
                break;
        }
    }

    void control_loop()
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        if (dt <= 0.005 || dt > 0.05) dt = 0.01;

        if (climb_override_ &&
            (climb_state_ == ClimbState::EXTEND_BACK ||
             climb_state_ == ClimbState::TRACK_CLIMB)) {
            track_mode_ = true;
        }

        double target_alpha = track_mode_ ? 0.0 : 1.0;
        if (transition_alpha_ != target_alpha) {
            double step = 0.10 * dt;
            if (transition_alpha_ < target_alpha)
                transition_alpha_ = std::min(target_alpha, transition_alpha_ + step);
            else
                transition_alpha_ = std::max(target_alpha, transition_alpha_ - step);
        }

        double roll_input  = (std::fabs(imu_roll_)  < 0.01) ? 0.0 : imu_roll_;
        double pitch_input = (std::fabs(imu_pitch_) < 0.01) ? 0.0 : imu_pitch_;

        double roll_corr = 0.0, pitch_corr = 0.0;
        if (auto_level_enabled_ && imu_received_) {
            pitch_corr = pitch_pid_.compute(0.0, pitch_input, dt);
            roll_corr  = roll_pid_.compute (0.0, roll_input,  dt);
        } else if (manual_control_) {
            roll_corr  = manual_roll_;
            pitch_corr = manual_pitch_;
        }

        double x_target_pitch = H * std::sin(pitch_corr);

        double lean_factor = 0.30;
        double x_lean = -lean_factor * std::sin(imu_roll_);
        double x_final_w = x0 + x_target_pitch + x_lean;

        double geometric_offset = (track_width / 2.0) * std::tan(imu_roll_);
        double delta_z = geometric_offset + roll_corr;

        double z_l_w = std::clamp(-H + delta_z, -H_max, -H_min);
        double z_r_w = std::clamp(-H - delta_z, -H_max, -H_min);

        double q1_w_r, q2_w_r, q1_w_l, q2_w_l;
        compute_ik(x_final_w, z_r_w, q1_w_r, q2_w_r);
        compute_ik(x_final_w, z_l_w, q1_w_l, q2_w_l);

        double t_pitch_corr = track_pitch_pid_.compute(0.0, pitch_input, dt);
        double t_roll_corr  = track_roll_pid_.compute (0.0, roll_input,  dt);
        const double H_track       = 0.30;
        const double x_track_base  = -0.08;

        double x_final       = x_track_base + (H_track * std::sin(t_pitch_corr));
        double z_final_track = -H_track * std::cos(t_pitch_corr);
        double z_base_track  = (track_width / 2.0) * std::tan(imu_roll_) + t_roll_corr;

        double z_l_track = std::clamp(z_final_track + z_base_track, -H_max, -H_min);
        double z_r_track = std::clamp(z_final_track - z_base_track, -H_max, -H_min);

        double q1_r_track, q2_r_track, q1_l_track, q2_l_track;
        compute_ik(x_final, z_r_track, q1_r_track, q2_r_track);
        q2_r_track = std::fabs(q2_r_track);  // Track mode: knee always positive
        compute_ik(x_final, z_l_track, q1_l_track, q2_l_track);
        q2_l_track = std::fabs(q2_l_track);

        // α=1.0 → pure wheeled targets; α=0.0 → pure track targets
        double q1_r = (transition_alpha_ * q1_w_r) + ((1.0 - transition_alpha_) * q1_r_track);
        double q2_r = (transition_alpha_ * q2_w_r) + ((1.0 - transition_alpha_) * q2_r_track);
        double q1_l = (transition_alpha_ * q1_w_l) + ((1.0 - transition_alpha_) * q1_l_track);
        double q2_l = (transition_alpha_ * q2_w_l) + ((1.0 - transition_alpha_) * q2_l_track);


        q2_w_r_cached_ = q2_w_r;
        q2_w_l_cached_ = q2_w_l;

        if (has_stair_info_) {
            stair_data_fresh_ = (current_time - last_stair_info_time_).seconds() < 0.3;
            if (!stair_data_fresh_) {
                detected_stairs_count_ = 0;
            }
        } else {
            stair_data_fresh_ = false;
        }

        if (climb_override_) {
            update_climb_sequence();

            if (climb_state_ == ClimbState::EXTEND_BACK ||
                climb_state_ == ClimbState::TRACK_CLIMB  ||
                climb_state_ == ClimbState::PULL_UP) {
                q2_r = q2_climb_r_;
                q2_l = q2_climb_l_;
            }

            std_msgs::msg::Float64 r_spd, l_spd;
            double base_speed    = climb_vx_;
            double differential  = 0.0;


            r_spd.data = base_speed - differential;
            l_spd.data = base_speed + differential;
            track_right_pub_->publish(r_spd);
            track_left_pub_->publish(l_spd);

        } else {
            geometry_msgs::msg::TwistStamped cmd_wheel;
            cmd_wheel.header.stamp = current_time;

            if (is_transitioning_) {
                cmd_wheel.twist.linear.x  = 0.0;
                cmd_wheel.twist.angular.z = 0.0;
            } else {
                cmd_wheel.twist.linear.x  = vx_ * transition_alpha_;
                cmd_wheel.twist.angular.z = wz_ * transition_alpha_;
            }
            wheel_vel_pub_->publish(cmd_wheel);

            double track_fade  = (1.0 - transition_alpha_);
            double applied_wz  = wz_;


            std_msgs::msg::Float64 r_spd, l_spd;
            r_spd.data = (vx_ + applied_wz * (track_width / 2.0)) * track_fade;
            l_spd.data = (vx_ - applied_wz * (track_width / 2.0)) * track_fade;
            track_right_pub_->publish(r_spd);
            track_left_pub_->publish(l_spd);
        }

        double tau_r_hip  = hip_pd_.compute (q1_r, current_joint_state[0], joint_velocity_[0]);
        double tau_r_knee = knee_pd_.compute(q2_r, current_joint_state[1], joint_velocity_[1]);
        double tau_l_hip  = hip_pd_.compute (q1_l, current_joint_state[2], joint_velocity_[2]);
        double tau_l_knee = knee_pd_.compute(q2_l, current_joint_state[3], joint_velocity_[3]);

        if (count_++ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "Mode: %s | Stairs Detected: %d",
                track_mode_ ? "TRACK" : "WHEEL",
                detected_stairs_count_);
        }

        std_msgs::msg::Float64MultiArray torque_cmd;
        torque_cmd.data = {tau_r_hip, tau_r_knee, -tau_l_hip, -tau_l_knee};
        joint_pub_->publish(torque_cmd);
    }

    enum class ClimbState {
        IDLE,        
        EXTEND_BACK, 
        TRACK_CLIMB,  
        PULL_UP     
    };

    const char* climbStateStr() const {
        switch (climb_state_) {
            case ClimbState::IDLE:        return "IDLE";
            case ClimbState::EXTEND_BACK: return "EXTEND_BACK";
            case ClimbState::TRACK_CLIMB: return "TRACK_CLIMB";
            case ClimbState::PULL_UP:     return "PULL_UP";
            default:                      return "UNKNOWN";
        }
    }

    ClimbState climb_state_{ClimbState::IDLE};
    int     climb_timer_{0};
    bool    climb_override_{false};
    double  q1_climb_r_{0}, q2_climb_r_{0};
    double  q1_climb_l_{0}, q2_climb_l_{0};
    double  climb_vx_{0};                  
    double  q2_w_r_cached_{0};             
    double  q2_w_l_cached_{0};             

    double transition_alpha_{1.0};
    double target_alpha_{1.0};    
    
    int    count_{0};
    double vx_{0}, wz_{0};

    int    detected_stairs_count_{0};
    bool   has_stair_info_{false};
    bool   stair_data_fresh_{false};
    rclcpp::Time last_stair_info_time_;

    double imu_roll_{0}, imu_pitch_{0}, imu_yaw_{0};
    const double alpha = 0.1;         
    bool imu_received_{false};

    double manual_roll_{0}, manual_pitch_{0};

    double L1;         
    double L2;         
    double x0;         
    double track_width;

    double H;     
    double H_min;  
    double H_max; 

    bool auto_level_enabled_;
    bool manual_control_;
    bool track_mode_{false};
    bool is_transitioning_{false};
    int  current_step_state_{0};

    PID     roll_pid_;         
    PID     pitch_pid_;       
    PID     track_pitch_pid_;  
    PID     track_roll_pid_;   
    JointPD hip_pd_;          
    JointPD knee_pd_;          

    rclcpp::Time last_time_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_;

    std::vector<double> current_joint_state;  
    std::vector<double> joint_velocity_{0,0,0,0};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr         joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr                imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr            cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                 step_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr     stair_info_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr        wheel_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr        joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                  track_right_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                  track_left_pub_;
    rclcpp::TimerBase::SharedPtr                                          timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
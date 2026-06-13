/**
 * @file gamepad_control_node.cpp
 * @brief Real-time controller for the Addverb Cobot using Cosmic Byte Ares Gamepad
 * @author Keshav Bansal
 * @date 2026-06-13
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "addverb_cobot_msgs/msg/joint_jogging_velocity.hpp"
#include "addverb_cobot_msgs/srv/gripper.hpp"

using namespace std::chrono_literals;

enum class ControlMode {
    CARTESIAN = 0,
    JOINT = 1
};

class GamepadControlNode : public rclcpp::Node {
public:
    GamepadControlNode() : Node("gamepad_control_node") {
        // Declare and retrieve parameters
        declare_parameters_();
        get_parameters_();

        // Initialize publishers
        cartesian_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cartesian_jogging_controller/cartesian_jogging/command", 10);
        
        joint_pub_ = this->create_publisher<addverb_cobot_msgs::msg::JointJoggingVelocity>(
            "/joint_jogging_controller/joint_jogging/command", 10);

        // Initialize service clients
        gripper_client_ = this->create_client<addverb_cobot_msgs::srv::Gripper>("/gripper_controller/command");
        recovery_client_ = this->create_client<std_srvs::srv::Trigger>("/cobot_services/error_recovery_srv");
        shutdown_client_ = this->create_client<std_srvs::srv::Trigger>("/cobot_services/shutdown_srv");

        // Subscribe to joystick topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&GamepadControlNode::joy_callback_, this, std::placeholders::_1));

        // Create main control loop timer (running at 50Hz)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&GamepadControlNode::control_loop_, this));

        // State variables
        current_mode_ = ControlMode::CARTESIAN;
        fast_speed_active_ = false;
        joy_received_ = false;

        RCLCPP_INFO(this->get_logger(), "Addverb Cobot Gamepad Remote Controller Node initialized.");
        RCLCPP_INFO(this->get_logger(), "Default Mode: CARTESIAN JOGGING (Hold LB to activate motions).");
    }

private:
    void declare_parameters_() {
        // Button indices (XInput mappings default for Cosmic Byte Ares)
        this->declare_parameter("button_gripper_close", 0);   // A button
        this->declare_parameter("button_gripper_open", 1);    // B button
        this->declare_parameter("button_mode_toggle", 2);     // X button
        this->declare_parameter("button_error_recovery", 3);  // Y button
        this->declare_parameter("button_deadman", 4);         // LB button
        this->declare_parameter("button_speed_toggle", 5);    // RB button
        this->declare_parameter("button_shutdown", 6);        // Back button

        // Cartesian Mode Axis indices
        this->declare_parameter("axis_linear_x", 1);          // Left Stick Y (Up/Down)
        this->declare_parameter("axis_linear_y", 0);          // Left Stick X (Left/Right)
        this->declare_parameter("axis_linear_z", 4);          // Right Stick Y (Up/Down)
        this->declare_parameter("axis_angular_z", 3);         // Right Stick X (Left/Right)
        this->declare_parameter("axis_angular_x", 6);         // D-pad X (Left/Right)
        this->declare_parameter("axis_angular_y", 7);         // D-pad Y (Up/Down)
        this->declare_parameter("axis_trigger_left", 2);      // LT Trigger Axis
        this->declare_parameter("axis_trigger_right", 5);     // RT Trigger Axis
        this->declare_parameter("use_triggers_for_z", true);  // Use LT/RT triggers for Z axis

        // Joint Mode Axis indices (individual joint velocities)
        this->declare_parameter("axis_joint_1", 1);           // Left Stick Y
        this->declare_parameter("axis_joint_2", 0);           // Left Stick X
        this->declare_parameter("axis_joint_3", 4);           // Right Stick Y
        this->declare_parameter("axis_joint_4", 3);           // Right Stick X
        this->declare_parameter("axis_joint_5", 7);           // D-pad Y
        this->declare_parameter("axis_joint_6", 6);           // D-pad X

        // Speed settings
        this->declare_parameter("linear_speed_scale_slow", 0.1);
        this->declare_parameter("linear_speed_scale_fast", 0.4);
        this->declare_parameter("angular_speed_scale_slow", 0.2);
        this->declare_parameter("angular_speed_scale_fast", 0.8);
        this->declare_parameter("joint_speed_scale_slow", 0.2);
        this->declare_parameter("joint_speed_scale_fast", 0.8);
        
        // General settings
        this->declare_parameter("deadband", 0.05);
        this->declare_parameter("gripper_force", 50.0);
    }

    void get_parameters_() {
        btn_gripper_close_ = this->get_parameter("button_gripper_close").as_int();
        btn_gripper_open_ = this->get_parameter("button_gripper_open").as_int();
        btn_mode_toggle_ = this->get_parameter("button_mode_toggle").as_int();
        btn_error_recovery_ = this->get_parameter("button_error_recovery").as_int();
        btn_deadman_ = this->get_parameter("button_deadman").as_int();
        btn_speed_toggle_ = this->get_parameter("button_speed_toggle").as_int();
        btn_shutdown_ = this->get_parameter("button_shutdown").as_int();

        ax_linear_x_ = this->get_parameter("axis_linear_x").as_int();
        ax_linear_y_ = this->get_parameter("axis_linear_y").as_int();
        ax_linear_z_ = this->get_parameter("axis_linear_z").as_int();
        ax_angular_z_ = this->get_parameter("axis_angular_z").as_int();
        ax_angular_x_ = this->get_parameter("axis_angular_x").as_int();
        ax_angular_y_ = this->get_parameter("axis_angular_y").as_int();
        ax_trigger_left_ = this->get_parameter("axis_trigger_left").as_int();
        ax_trigger_right_ = this->get_parameter("axis_trigger_right").as_int();
        use_triggers_for_z_ = this->get_parameter("use_triggers_for_z").as_bool();

        ax_joint_1_ = this->get_parameter("axis_joint_1").as_int();
        ax_joint_2_ = this->get_parameter("axis_joint_2").as_int();
        ax_joint_3_ = this->get_parameter("axis_joint_3").as_int();
        ax_joint_4_ = this->get_parameter("axis_joint_4").as_int();
        ax_joint_5_ = this->get_parameter("axis_joint_5").as_int();
        ax_joint_6_ = this->get_parameter("axis_joint_6").as_int();

        linear_scale_slow_ = this->get_parameter("linear_speed_scale_slow").as_double();
        linear_scale_fast_ = this->get_parameter("linear_speed_scale_fast").as_double();
        angular_scale_slow_ = this->get_parameter("angular_speed_scale_slow").as_double();
        angular_scale_fast_ = this->get_parameter("angular_speed_scale_fast").as_double();
        joint_scale_slow_ = this->get_parameter("joint_speed_scale_slow").as_double();
        joint_scale_fast_ = this->get_parameter("joint_speed_scale_fast").as_double();

        deadband_ = this->get_parameter("deadband").as_double();
        gripper_force_ = this->get_parameter("gripper_force").as_double();
    }

    void joy_callback_(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_joy_ = *joy_msg;
        joy_received_ = true;

        // Update calibration status for triggers (avoids startup drift)
        if (latest_joy_.axes.size() > static_cast<size_t>(ax_trigger_left_)) {
            if (latest_joy_.axes[ax_trigger_left_] != 0.0) {
                lt_calibrated_ = true;
            }
        }
        if (latest_joy_.axes.size() > static_cast<size_t>(ax_trigger_right_)) {
            if (latest_joy_.axes[ax_trigger_right_] != 0.0) {
                rt_calibrated_ = true;
            }
        }

        // Process button transitions (rising edge) to prevent spamming service calls
        if (prev_joy_.buttons.size() == latest_joy_.buttons.size()) {
            // Gripper Open (B Button)
            if (latest_joy_.buttons[btn_gripper_open_] && !prev_joy_.buttons[btn_gripper_open_]) {
                trigger_gripper_(1.0);
            }
            // Gripper Close (A Button)
            else if (latest_joy_.buttons[btn_gripper_close_] && !prev_joy_.buttons[btn_gripper_close_]) {
                trigger_gripper_(0.0);
            }

            // Mode Toggle (X Button)
            if (latest_joy_.buttons[btn_mode_toggle_] && !prev_joy_.buttons[btn_mode_toggle_]) {
                if (current_mode_ == ControlMode::CARTESIAN) {
                    current_mode_ = ControlMode::JOINT;
                    RCLCPP_INFO(this->get_logger(), "Switched Mode to: JOINT JOGGING");
                } else {
                    current_mode_ = ControlMode::CARTESIAN;
                    RCLCPP_INFO(this->get_logger(), "Switched Mode to: CARTESIAN JOGGING");
                }
            }

            // Speed Toggle (RB Button)
            if (latest_joy_.buttons[btn_speed_toggle_] && !prev_joy_.buttons[btn_speed_toggle_]) {
                fast_speed_active_ = !fast_speed_active_;
                RCLCPP_INFO(this->get_logger(), "Speed Mode changed to: %s", fast_speed_active_ ? "FAST" : "SLOW");
            }

            // Error Recovery (Y Button)
            if (latest_joy_.buttons[btn_error_recovery_] && !prev_joy_.buttons[btn_error_recovery_]) {
                trigger_error_recovery_();
            }

            // Shutdown (Back Button)
            if (latest_joy_.buttons[btn_shutdown_] && !prev_joy_.buttons[btn_shutdown_]) {
                trigger_shutdown_();
            }
        } else {
            // Handle first callback initialization
            RCLCPP_INFO(this->get_logger(), "Joystick detected with %zu axes and %zu buttons.", 
                        latest_joy_.axes.size(), latest_joy_.buttons.size());
        }

        prev_joy_ = latest_joy_;
    }

    double apply_deadband_(double value) {
        if (std::abs(value) < deadband_) {
            return 0.0;
        }
        // Scale remaining range linearly
        return (value > 0.0) ? (value - deadband_) / (1.0 - deadband_)
                             : (value + deadband_) / (1.0 - deadband_);
    }

    double get_axis_value_(const sensor_msgs::msg::Joy& joy, size_t index) {
        if (index < joy.axes.size()) {
            return apply_deadband_(joy.axes[index]);
        }
        return 0.0;
    }

    bool is_button_pressed_(const sensor_msgs::msg::Joy& joy, size_t index) {
        if (index < joy.buttons.size()) {
            return joy.buttons[index] != 0;
        }
        return false;
    }

    void control_loop_() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!joy_received_) {
            return;
        }

        // Safety: Check deadman switch (LB)
        if (!is_button_pressed_(latest_joy_, btn_deadman_)) {
            // Publish zero command to ensure robot stops immediately when LB is released
            publish_zero_commands_();
            return;
        }

        // Apply scale depending on speed toggles
        double lin_scale = fast_speed_active_ ? linear_scale_fast_ : linear_scale_slow_;
        double ang_scale = fast_speed_active_ ? angular_scale_fast_ : angular_scale_slow_;
        double jnt_scale = fast_speed_active_ ? joint_scale_fast_ : joint_scale_slow_;

        if (current_mode_ == ControlMode::CARTESIAN) {
            auto twist = geometry_msgs::msg::Twist();
            
            // Map sticks to linear Cartesian motions (X, Y, Z)
            twist.linear.x = get_axis_value_(latest_joy_, ax_linear_x_) * lin_scale;
            twist.linear.y = get_axis_value_(latest_joy_, ax_linear_y_) * lin_scale;

            if (use_triggers_for_z_) {
                // Squeezing RT moves Z up, squeezing LT moves Z down
                double lt_val = 0.0;
                double rt_val = 0.0;

                if (lt_calibrated_ && static_cast<size_t>(ax_trigger_left_) < latest_joy_.axes.size()) {
                    lt_val = (1.0 - latest_joy_.axes[ax_trigger_left_]) / 2.0;
                }
                if (rt_calibrated_ && static_cast<size_t>(ax_trigger_right_) < latest_joy_.axes.size()) {
                    rt_val = (1.0 - latest_joy_.axes[ax_trigger_right_]) / 2.0;
                }

                twist.linear.z = (rt_val - lt_val) * lin_scale;
            } else {
                twist.linear.z = get_axis_value_(latest_joy_, ax_linear_z_) * lin_scale;
            }

            // Map sticks/dpad to angular Cartesian motions (Roll, Pitch, Yaw)
            twist.angular.x = get_axis_value_(latest_joy_, ax_angular_x_) * ang_scale;
            twist.angular.y = get_axis_value_(latest_joy_, ax_angular_y_) * ang_scale;
            twist.angular.z = get_axis_value_(latest_joy_, ax_angular_z_) * ang_scale;

            cartesian_pub_->publish(twist);

        } else if (current_mode_ == ControlMode::JOINT) {
            auto jvel_msg = addverb_cobot_msgs::msg::JointJoggingVelocity();
            jvel_msg.jvel_scaling_factor.resize(6, 0.0);

            // Map axes to 6 joints
            jvel_msg.jvel_scaling_factor[0] = get_axis_value_(latest_joy_, ax_joint_1_) * jnt_scale;
            jvel_msg.jvel_scaling_factor[1] = get_axis_value_(latest_joy_, ax_joint_2_) * jnt_scale;
            jvel_msg.jvel_scaling_factor[2] = get_axis_value_(latest_joy_, ax_joint_3_) * jnt_scale;
            jvel_msg.jvel_scaling_factor[3] = get_axis_value_(latest_joy_, ax_joint_4_) * jnt_scale;
            jvel_msg.jvel_scaling_factor[4] = get_axis_value_(latest_joy_, ax_joint_5_) * jnt_scale;
            jvel_msg.jvel_scaling_factor[5] = get_axis_value_(latest_joy_, ax_joint_6_) * jnt_scale;

            joint_pub_->publish(jvel_msg);
        }
    }

    void publish_zero_commands_() {
        if (current_mode_ == ControlMode::CARTESIAN) {
            auto twist = geometry_msgs::msg::Twist();
            cartesian_pub_->publish(twist);
        } else if (current_mode_ == ControlMode::JOINT) {
            auto jvel_msg = addverb_cobot_msgs::msg::JointJoggingVelocity();
            jvel_msg.jvel_scaling_factor.resize(6, 0.0);
            joint_pub_->publish(jvel_msg);
        }
    }

    void trigger_gripper_(double position) {
        if (!gripper_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Gripper service not ready!");
            return;
        }

        auto request = std::make_shared<addverb_cobot_msgs::srv::Gripper::Request>();
        request->position = position;
        request->grasp_force = (position == 0.0) ? gripper_force_ : 0.0;

        std::string action = (position == 1.0) ? "open" : "close";
        RCLCPP_INFO(this->get_logger(), "Calling gripper service to %s...", action.c_str());

        gripper_client_->async_send_request(request, 
            [this, action](rclcpp::Client<addverb_cobot_msgs::srv::Gripper>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Gripper %s successful: %s", action.c_str(), response->message.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Gripper %s failed: %s", action.c_str(), response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception while calling gripper service: %s", e.what());
                }
            });
    }

    void trigger_error_recovery_() {
        if (!recovery_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Error recovery service not ready!");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        RCLCPP_INFO(this->get_logger(), "Triggering Error Recovery service...");

        recovery_client_->async_send_request(request, 
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Error Recovery completed successfully: %s", response->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Error Recovery failed: %s", response->message.c_str());
                }
            });
    }

    void trigger_shutdown_() {
        if (!shutdown_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Shutdown service not ready!");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        RCLCPP_INFO(this->get_logger(), "Calling Shutdown service for safe state shutdown...");

        shutdown_client_->async_send_request(request, 
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Shutdown command processed successfully: %s", response->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Shutdown command failed: %s", response->message.c_str());
                }
            });
    }

    // ROS 2 Communication Objects
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cartesian_pub_;
    rclcpp::Publisher<addverb_cobot_msgs::msg::JointJoggingVelocity>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Client<addverb_cobot_msgs::srv::Gripper>::SharedPtr gripper_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recovery_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Thread safety
    std::mutex mutex_;

    // Joy Data
    sensor_msgs::msg::Joy latest_joy_;
    sensor_msgs::msg::Joy prev_joy_;
    bool joy_received_;

    // Mappings and configuration parameters
    int btn_gripper_close_;
    int btn_gripper_open_;
    int btn_mode_toggle_;
    int btn_error_recovery_;
    int btn_deadman_;
    int btn_speed_toggle_;
    int btn_shutdown_;

    int ax_linear_x_;
    int ax_linear_y_;
    int ax_linear_z_;
    int ax_angular_z_;
    int ax_angular_x_;
    int ax_angular_y_;

    int ax_trigger_left_;
    int ax_trigger_right_;
    bool use_triggers_for_z_;

    // Calibration flags for triggers to prevent startup drift
    bool lt_calibrated_ = false;
    bool rt_calibrated_ = false;

    int ax_joint_1_;
    int ax_joint_2_;
    int ax_joint_3_;
    int ax_joint_4_;
    int ax_joint_5_;
    int ax_joint_6_;

    double linear_scale_slow_;
    double linear_scale_fast_;
    double angular_scale_slow_;
    double angular_scale_fast_;
    double joint_scale_slow_;
    double joint_scale_fast_;

    double deadband_;
    double gripper_force_;

    // States
    ControlMode current_mode_;
    bool fast_speed_active_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GamepadControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

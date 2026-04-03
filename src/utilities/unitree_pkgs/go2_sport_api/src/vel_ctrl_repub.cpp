#include <algorithm>
#include <cmath>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "common/ros2_sport_client.h"

typedef union
{
  struct
  {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

/**
 * @class Go2CmdVelRepub
 * @brief ROS2 node that republishes cmd_vel commands to Go2 sport API
 * 
 * This node subscribes to /cmd_vel and forwards velocity commands to the Go2 robot.
 * Press L1+R1 to toggle between AUTO mode (forward /cmd_vel) and MANUAL mode
 * (this node stays silent so the wireless controller can fully take over).
 */
class Go2CmdVelRepub : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    Go2CmdVelRepub()
        : Node("vel_cmd_repub")
        , sport_client_(this)
        , target_vx_(0.0)
        , target_vy_(0.0)
        , target_vyaw_(0.0)
        , filtered_vx_(0.0)
        , filtered_vy_(0.0)
        , filtered_vyaw_(0.0)
        , enable_vel_cmd_(true)  // Default: enabled
        , last_L1_R1_state_(false)
        , cmd_timeout_(rclcpp::Duration::from_seconds(0.25))
        , control_period_(std::chrono::milliseconds(20))
    {
        // Create subscriber for wireless controller
        wireless_controller_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10,
            std::bind(&Go2CmdVelRepub::wireless_controller_callback, this, std::placeholders::_1));

        // Create subscriber for cmd_vel
        vel_cmd_suber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&Go2CmdVelRepub::vel_cmd_callback, this, std::placeholders::_1));

        unitree_api::msg::Request req;

        sport_client_.ClassicWalk(req, true);
        last_cmd_time_ = this->now();
        control_timer_ = this->create_wall_timer(
            control_period_, std::bind(&Go2CmdVelRepub::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Node initialized. AUTO mode: %s", 
                enable_vel_cmd_ ? "ON" : "OFF");
        RCLCPP_INFO(this->get_logger(), "Press L1+R1 to toggle AUTO/MANUAL mode");
        RCLCPP_INFO(this->get_logger(), "Velocity forwarding runs at 50 Hz with %.0f ms timeout",
                    cmd_timeout_.seconds() * 1000.0);
    }


private:
    /**
     * @brief Callback function for cmd_vel topic
     * @param msg Twist message containing velocity commands
     */
    void vel_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!enable_vel_cmd_) {
            return;
        }

        target_vx_ = msg->linear.x;
        target_vy_ = msg->linear.y;
        target_vyaw_ = msg->angular.z;
        last_cmd_time_ = this->now();
    }

    /**
     * @brief Callback function for wireless controller data from ROS2 topic
    * Handles L1+R1 toggle logic for AUTO/MANUAL mode
     */
    void wireless_controller_callback(const unitree_go::msg::WirelessController::SharedPtr msg)
    {
        try {
            xKeySwitchUnion keys;
            keys.value = msg->keys;
            
            // Parse keys field to check L1 and R1 buttons
            // According to unitree documentation:
            // R1 is bit 0 (0x0001)
            // L1 is bit 1 (0x0002)
            bool L1_pressed = (keys.components.L1) != 0;
            bool R1_pressed = (keys.components.R1) != 0;
            bool L1_R1_both = L1_pressed && R1_pressed;
            
            // Toggle logic: detect transition from not-pressed to pressed
            if (L1_R1_both && !last_L1_R1_state_) {
                // Toggle between auto cmd_vel mode and manual controller mode
                enable_vel_cmd_ = !enable_vel_cmd_;
                if (enable_vel_cmd_) {
                    RCLCPP_INFO(this->get_logger(), "AUTO mode ON - Resuming /cmd_vel forwarding");
                    last_cmd_time_ = this->now();
                } else {
                    RCLCPP_INFO(this->get_logger(), "MANUAL mode ON - Stop forwarding /cmd_vel to sport API");
                    target_vx_ = 0.0;
                    target_vy_ = 0.0;
                    target_vyaw_ = 0.0;
                    filtered_vx_ = 0.0;
                    filtered_vy_ = 0.0;
                    filtered_vyaw_ = 0.0;

                    // Send a single stop command when handing over to manual control.
                    unitree_api::msg::Request req;
                    sport_client_.StopMove(req);
                }
            }
            
            last_L1_R1_state_ = L1_R1_both;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error processing wireless controller callback: %s", e.what());
        }
    }

    /**
     * @brief Send command using ROS2 SportClient
     * Note: SportClient methods internally publish to /api/sport/request
     */
    void publish_command()
    {
        try {
            unitree_api::msg::Request req;

            if (enable_vel_cmd_) {
                if (is_near_zero(filtered_vx_) && is_near_zero(filtered_vy_) && is_near_zero(filtered_vyaw_)) {
                    sport_client_.StopMove(req);
                } else {
                    sport_client_.Move(req, filtered_vx_, filtered_vy_, filtered_vyaw_);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending sport command: %s", e.what());
        }
    }

    void control_loop()
    {
        if (!enable_vel_cmd_) {
            return;
        }

        const rclcpp::Time now = this->now();
        const bool cmd_timed_out = (now - last_cmd_time_) > cmd_timeout_;

        const float desired_vx = cmd_timed_out ? 0.0f : target_vx_;
        const float desired_vy = cmd_timed_out ? 0.0f : target_vy_;
        const float desired_vyaw = cmd_timed_out ? 0.0f : target_vyaw_;

        filtered_vx_ = step_towards(filtered_vx_, desired_vx, kMaxLinearAccel * kControlDtSec);
        filtered_vy_ = step_towards(filtered_vy_, desired_vy, kMaxLinearAccel * kControlDtSec);
        filtered_vyaw_ = step_towards(filtered_vyaw_, desired_vyaw, kMaxYawAccel * kControlDtSec);

        publish_command();
    }

    static float step_towards(float current, float target, float max_step)
    {
        const float delta = target - current;
        return current + std::clamp(delta, -max_step, max_step);
    }

    static bool is_near_zero(float value)
    {
        return std::fabs(value) < 1e-4F;
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_suber_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_controller_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // SportClient helper class
    SportClient sport_client_;

    // Velocity command variables
    float target_vx_;
    float target_vy_;
    float target_vyaw_;
    float filtered_vx_;
    float filtered_vy_;
    float filtered_vyaw_;

    // AUTO/MANUAL mode control
    bool enable_vel_cmd_;
    bool last_L1_R1_state_;
    rclcpp::Time last_cmd_time_;
    const rclcpp::Duration cmd_timeout_;
    const std::chrono::milliseconds control_period_;

    static constexpr float kControlDtSec = 0.02F;
    static constexpr float kMaxLinearAccel = 1.0F;
    static constexpr float kMaxYawAccel = 2.5F;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2CmdVelRepub>();
    
    // Use spin to wait for messages and process callbacks immediately
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}

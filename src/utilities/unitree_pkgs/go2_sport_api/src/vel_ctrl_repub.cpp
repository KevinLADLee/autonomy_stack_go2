#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>

// Include ROS2 message types
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

// Include ros2_sport_client
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

/*
运动模式
0. idle, default stand
1. balanceStand
2. pose
3. locomotion
4. reserve
5. lieDown
6. jointLock
7. damping
8. recoveryStand
9. reserve
10. sit
11. frontFlip
12. frontJump
13. frontPounc
*/
enum class SportModeState : uint8_t
{
    IDLE = 0,
    BALANCE_STAND = 1,
    POSE = 2,
    LOCOMOTION = 3,
    RESERVE_1 = 4,
    LIE_DOWN = 5,
    JOINT_LOCK = 6,
    DAMPING = 7,
    RECOVERY_STAND = 8,
    RESERVE_2 = 9,
    SIT = 10,
    FRONT_FLIP = 11,
    FRONT_JUMP = 12,
    FRONT_POUNCE = 13,
};

/**
 * @class Go2CmdVelRepub
 * @brief ROS2 node that republishes cmd_vel commands to Go2 sport API using ROS2 topics
 * 
 * This node subscribes to /cmd_vel and forwards velocity commands to the Go2 robot.
 * It includes a protection mode that can be toggled by pressing L1+R1 on the wireless controller.
 * In protection mode, the robot enters Balance Stand to prevent falling.
 * 
 * This version uses ROS2 topics for sport API, wireless controller, and sport mode state.
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
        , vx_(0.0)
        , vy_(0.0)
        , vyaw_(0.0)
        , enable_vel_cmd_(true)  // Default: enabled
        , last_L1_R1_state_(false)
    {
        // Create publisher for sport API requests
        sport_request_pub_ = this->create_publisher<unitree_api::msg::Request>(
            "/api/sport/request", 10);
        // RCLCPP_INFO(this->get_logger(), "Sport request publisher initialized");
        
        // Create ROS2 subscriber for wireless controller
        wireless_controller_sub_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10,
            std::bind(&Go2CmdVelRepub::wireless_controller_callback, this, std::placeholders::_1));
        // RCLCPP_INFO(this->get_logger(), "WirelessController subscriber initialized");

        // Create ROS2 subscriber for sport mode state
        sport_mode_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "lf/sportmodestate", 10,
            std::bind(&Go2CmdVelRepub::sport_mode_state_callback, this, std::placeholders::_1));
        // RCLCPP_INFO(this->get_logger(), "SportModeState subscriber initialized");
        
        // Create subscriber for cmd_vel
        vel_cmd_suber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10, 
            std::bind(&Go2CmdVelRepub::vel_cmd_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Node initialized. Protection mode: %s", 
                    enable_vel_cmd_ ? "OFF" : "ON");
        RCLCPP_INFO(this->get_logger(), "Press L1+R1 to toggle protection mode");
    }

    /**
     * @brief Publish command in control loop
     * This function should be called periodically (e.g., at 100Hz)
     */
    void publish_command_loop()
    {
        publish_command();
    }

private:
    /**
     * @brief Callback function for cmd_vel topic
     * @param msg TwistStamped message containing velocity commands
     */
    void vel_cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        vx_ = msg->twist.linear.x;
        vy_ = msg->twist.linear.y;
        vyaw_ = msg->twist.angular.z;
    }

    void sport_mode_state_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        sport_mode_state_ = static_cast<SportModeState>(msg->mode);
        // RCLCPP_INFO(this->get_logger(), "Sport mode state: %d", sport_mode_state_);
    }

    /**
     * @brief Callback function for wireless controller data from ROS2 topic
     * Handles L1+R1 toggle logic for protection mode
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
                // Toggle protection mode
                enable_vel_cmd_ = !enable_vel_cmd_;
                if (enable_vel_cmd_) {
                    RCLCPP_INFO(this->get_logger(), "Protection mode OFF - Resuming velocity control");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Protection mode ON - Entering Balance Stand");
                }
                // Immediately send command to reduce latency
                publish_command();
            }
            
            last_L1_R1_state_ = L1_R1_both;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error processing wireless controller callback: %s", e.what());
        }
    }

    /**
     * @brief Send command using ROS2 SportClient and publish to /api/sport/request
     */
    void publish_command()
    {
        try {
            unitree_api::msg::Request req;
            
            if (enable_vel_cmd_) {
                // Normal mode: forward cmd_vel commands
                if (vx_ == 0.0 && vy_ == 0.0 && vyaw_ == 0.0) {
                    sport_client_.StopMove(req);
                } else {
                    sport_client_.Move(req, vx_, vy_, vyaw_);
                }
            } else {
                // Protection mode: send BalanceStand command
                if (sport_mode_state_ != SportModeState::IDLE) {
                    sport_client_.StopMove(req);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    sport_client_.BalanceStand(req);
                } else {
                    // Already in BalanceStand, no need to send command
                    return;
                }
            }
            
            // Publish the request message
            sport_request_pub_->publish(req);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending sport command: %s", e.what());
        }
    }

    // Subscriber for cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_suber_;
    
    // Publisher for sport API requests
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_request_pub_;
    
    // ROS2 SportClient helper class
    SportClient sport_client_;
    
    // ROS2 subscribers
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_controller_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_mode_state_sub_;
    
    // Velocity command variables
    float vx_;
    float vy_;
    float vyaw_;
    
    // Protection mode control
    bool enable_vel_cmd_;
    bool last_L1_R1_state_;


    // Sport mode state
    SportModeState sport_mode_state_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2CmdVelRepub>();
    
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->publish_command_loop();
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}

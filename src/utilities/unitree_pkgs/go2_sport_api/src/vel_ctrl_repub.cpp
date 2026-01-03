#include <rclcpp/rclcpp.hpp>

#include "common/ros2_sport_client.h"
#include "unitree_api/msg/request.hpp"
#include "unitree_go/msg/wireless_controller.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>

/**
 * @class Go2CmdVelRepub
 * @brief ROS2 node that republishes cmd_vel commands to Go2 sport API
 * 
 * This node subscribes to /cmd_vel and forwards velocity commands to the Go2 robot.
 * It includes a protection mode that can be toggled by pressing L1+R1 on the wireless controller.
 * In protection mode, the robot enters Balance Stand to prevent falling.
 */
class Go2CmdVelRepub : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    Go2CmdVelRepub()
        : Node("vel_cmd_repub")
        , vx_(0.0)
        , vy_(0.0)
        , vyaw_(0.0)
        , enable_vel_cmd_(true)  // Default: enabled
        , last_L1_R1_state_(false)
    {
        // Create subscribers
        vel_cmd_suber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10, 
            std::bind(&Go2CmdVelRepub::vel_cmd_callback, this, std::placeholders::_1));
        
        wireless_controller_suber_ = this->create_subscription<unitree_go::msg::WirelessController>(
            "/wirelesscontroller", 10, 
            std::bind(&Go2CmdVelRepub::wireless_controller_callback, this, std::placeholders::_1));
        
        // Create publisher
        req_puber_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        
        RCLCPP_INFO(this->get_logger(), "Node initialized. Protection mode: %s", 
                    enable_vel_cmd_ ? "OFF" : "ON");
        RCLCPP_INFO(this->get_logger(), "Press L1+R1 to toggle protection mode");
    }

    /**
     * @brief Destructor
     */
    ~Go2CmdVelRepub() = default;

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

    /**
     * @brief Callback function for wireless controller topic
     * Handles L1+R1 toggle logic for protection mode
     * @param msg WirelessController message containing button states
     */
    void wireless_controller_callback(const unitree_go::msg::WirelessController::SharedPtr msg)
    {
        // Parse keys field to check L1 and R1 buttons
        // According to advanced_gamepad.hpp:
        // R1 is bit 0 (0x0001)
        // L1 is bit 1 (0x0002)
        uint16_t keys = msg->keys;
        bool L1_pressed = (keys & 0x0002) != 0;
        bool R1_pressed = (keys & 0x0001) != 0;
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
            // Immediately publish command to reduce latency
            publish_command();
        }
        
        last_L1_R1_state_ = L1_R1_both;
    }

    /**
     * @brief Publish command based on current mode
     */
    void publish_command()
    {
        unitree_api::msg::Request req;
        
        if (enable_vel_cmd_) {
            // Normal mode: forward cmd_vel commands
            if (vx_ == 0.0 && vy_ == 0.0 && vyaw_ == 0.0) {
                sport_req_.StopMove(req);
            } else {
                sport_req_.Move(req, vx_, vy_, vyaw_);
            }
        } else {
            // Protection mode: send BalanceStand command
            sport_req_.BalanceStand(req);
        }
        
        req_puber_->publish(req);
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_suber_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_controller_suber_;
    
    // Publisher
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
    
    // Sport client
    SportClient sport_req_;
    
    // Velocity command variables
    float vx_;
    float vy_;
    float vyaw_;
    
    // Protection mode control
    bool enable_vel_cmd_;
    bool last_L1_R1_state_;
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

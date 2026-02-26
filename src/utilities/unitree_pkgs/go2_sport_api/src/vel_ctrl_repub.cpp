#include <memory>
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
 * Default: manual mode (sport API does not forward /cmd_vel). Robot is controlled only
 * by the wireless controller. Press L1+R1 to enable auto navigation (sport API forwards
 * /cmd_vel). Press L1+R1 again to return to manual mode. This ensures safe startup
 * without autonomous motion.
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
        , enable_vel_cmd_(false)  // Default: manual mode (no auto nav until L1+R1)
        , last_L1_R1_state_(false)
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
        // Start in manual: one StopMove so no residual velocity command from autonomy
        sport_client_.StopMove(req);

        RCLCPP_INFO(this->get_logger(), "Default: manual mode (sport API not forwarding /cmd_vel)");
        RCLCPP_INFO(this->get_logger(), "Press L1+R1 to enable auto navigation; L1+R1 again to return to manual");
    }


private:
    /**
     * @brief Callback function for cmd_vel topic
     * @param msg Twist message containing velocity commands
     */
    void vel_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        vyaw_ = msg->angular.z;
        // Immediately forward the command upon receipt
        publish_command();
    }

    /**
     * @brief Callback function for wireless controller data from ROS2 topic
     * L1+R1 toggles between manual (sport API off) and auto navigation (sport API forwards /cmd_vel).
     */
    void wireless_controller_callback(const unitree_go::msg::WirelessController::SharedPtr msg)
    {
        try {
            xKeySwitchUnion keys;
            keys.value = msg->keys;

            // R1 is bit 0 (0x0001), L1 is bit 1 (0x0002)
            bool L1_pressed = (keys.components.L1) != 0;
            bool R1_pressed = (keys.components.R1) != 0;
            bool L1_R1_both = L1_pressed && R1_pressed;

            // Toggle on L1+R1 press: manual <-> auto navigation
            if (L1_R1_both && !last_L1_R1_state_) {
                enable_vel_cmd_ = !enable_vel_cmd_;
                if (enable_vel_cmd_) {
                    RCLCPP_INFO(this->get_logger(), "Auto navigation ON - Sport API forwarding /cmd_vel");
                    publish_command();
                } else {
                    RCLCPP_INFO(this->get_logger(), "Manual mode ON - Sport API stopped (wireless controller only)");
                    vx_ = 0.0;
                    vy_ = 0.0;
                    vyaw_ = 0.0;
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
        if (!enable_vel_cmd_) {
            // Manual mode: do not send anything (sport API off, robot under wireless control)
            return;
        }
        try {
            unitree_api::msg::Request req;
            if (vx_ == 0.0 && vy_ == 0.0 && vyaw_ == 0.0) {
                sport_client_.StopMove(req);
            } else {
                sport_client_.Move(req, vx_, vy_, vyaw_);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error sending sport command: %s", e.what());
        }
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_suber_;
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr wireless_controller_sub_;

    // SportClient helper class
    SportClient sport_client_;

    // Velocity command variables
    float vx_;
    float vy_;
    float vyaw_;

    // When true: sport API forwards /cmd_vel (auto nav). When false: manual mode (wireless controller only).
    bool enable_vel_cmd_;
    bool last_L1_R1_state_;
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

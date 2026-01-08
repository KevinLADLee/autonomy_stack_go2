#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "unitree_go/msg/low_state.hpp"

/**
 * @brief Node that converts Go2 LowState to standard ROS2 joint_states
 * 
 * This node subscribes to /lowstate (Unitree LowState) and converts
 * the motor states to standard ROS2 sensor_msgs/JointState format.
 * 
 * Joint mapping (from motor_crc.h):
 * - FR_0=0, FR_1=1, FR_2=2  -> FR_hip_joint, FR_thigh_joint, FR_calf_joint
 * - FL_0=3, FL_1=4, FL_2=5  -> FL_hip_joint, FL_thigh_joint, FL_calf_joint
 * - RR_0=6, RR_1=7, RR_2=8  -> RR_hip_joint, RR_thigh_joint, RR_calf_joint
 * - RL_0=9, RL_1=10, RL_2=11 -> RL_hip_joint, RL_thigh_joint, RL_calf_joint
 */
class Go2JointStatePublisher : public rclcpp::Node
{
public:
    Go2JointStatePublisher()
        : Node("go2_joint_state_publisher")
    {
        // Create publisher for joint states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        
        // Create subscriber for LowState
        lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 10,
            std::bind(&Go2JointStatePublisher::lowstate_callback, this, std::placeholders::_1));
        
        // Define joint names in the correct order
        joint_names_ = {
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
        };
        
        RCLCPP_INFO(this->get_logger(), "Go2 Joint State Publisher initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /lowstate, publishing to /joint_states");
    }

private:
    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
    {
        // Check if we have enough motor states (Go2 has 20 motors, we need 12 for joints)
        if (msg->motor_state.size() < 12)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "LowState has insufficient motor states: %zu (expected at least 12)",
                msg->motor_state.size());
            return;
        }
        
        auto joint_state = sensor_msgs::msg::JointState();
        // Use current time to match with other TF publishers
        joint_state.header.stamp = this->now();
        joint_state.header.frame_id = "base";
        
        // Motor indices (from motor_crc.h)
        // FR: 0,1,2 | FL: 3,4,5 | RR: 6,7,8 | RL: 9,10,11
        // These correspond to: hip, thigh, calf for each leg
        std::vector<int> motor_indices = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        
        joint_state.name = joint_names_;
        joint_state.position.resize(joint_names_.size());
        joint_state.velocity.resize(joint_names_.size());
        joint_state.effort.resize(joint_names_.size());
        
        // Convert motor states to joint states
        // MotorState fields:
        // - q: joint position (radians, range: -7 to +7)
        // - dq: joint velocity (rad/s)
        // - tau_est: joint torque/effort (N.m)
        for (size_t i = 0; i < motor_indices.size(); ++i)
        {
            int motor_idx = motor_indices[i];
            if (motor_idx >= 0 && motor_idx < static_cast<int>(msg->motor_state.size()))
            {
                joint_state.position[i] = msg->motor_state[motor_idx].q;        // Position in radians
                joint_state.velocity[i] = msg->motor_state[motor_idx].dq;      // Velocity in rad/s
                joint_state.effort[i] = msg->motor_state[motor_idx].tau_est;  // Torque in N.m
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Invalid motor index: %d (max: %zu)", motor_idx, msg->motor_state.size());
            }
        }
        
        joint_state_pub_->publish(joint_state);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
    std::vector<std::string> joint_names_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2JointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


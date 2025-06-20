#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <vector>
#include <cmath>

class SafeStopper : public rclcpp::Node
{
public:
    SafeStopper() : Node("safe_stopper_node")
    {
        // Parameters
        obstacle_threshold_ = 0.5; // Stop if obstacle < 0.5m
        rotate_speed_ = 1.0;       // Rotate at 1.0 rad/s when blocked

        // Publishers and Subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/safe_cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SafeStopper::scanCallback, this, std::placeholders::_1));
        keyboard_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&SafeStopper::keyboardCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Safe Stopper Node Started");
    }

private:
    void keyboardCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_vel_ = *msg; // Store the latest keyboard command
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        bool obstacle_nearby = false;
        float min_distance = std::numeric_limits<float>::infinity();
        geometry_msgs::msg::Twist output_cmd;

        // Get laser properties
        size_t total_ranges = msg->ranges.size();
        double angle_min = msg->angle_min;
        double angle_increment = msg->angle_increment;

        // Define sector width (e.g., ±15 degrees)
        double sector_width_rad = M_PI / 12; // 15 degrees

        // Determine which sector to check based on motion
        double target_angle = 0.0;  // default: front
        if (last_cmd_vel_.linear.x < 0.0)
        {
            target_angle = M_PI;  // rear
            obstacle_threshold_ = 1.5;
        }
        else if (last_cmd_vel_.linear.x == 0.0)
        {
            // If not moving forward or backward, just pass command
            cmd_vel_pub_->publish(last_cmd_vel_);
            return;
        }

        for (size_t i = 0; i < total_ranges; ++i)
        {
            double angle = angle_min + i * angle_increment;
            double range = msg->ranges[i];

            // Check only points in the desired sector
            if (std::abs(angle - target_angle) <= sector_width_rad)
            {
                if (!std::isinf(range) && range < obstacle_threshold_)
                {
                    obstacle_nearby = true;
                    min_distance = range;
                    break;
                }
            }
        }

        if (obstacle_nearby)
        {
            if (last_cmd_vel_.linear.x < 0)
                output_cmd.linear.x = 0.2;
            else
                output_cmd.linear.x = -0.2;
                
            RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f m! Rotating...", min_distance);
        }
        else
        {
            output_cmd = last_cmd_vel_;
        }

        cmd_vel_pub_->publish(output_cmd);
    }

    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_sub_;

    geometry_msgs::msg::Twist last_cmd_vel_;
    float obstacle_threshold_;
    float rotate_speed_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeStopper>());
    rclcpp::shutdown();
    return 0;
}

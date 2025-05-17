#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SquaredPublisherNode : public rclcpp::Node
{
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

public:
    SquaredPublisherNode() : Node("squared_numbers")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>("number", 10, std::bind(&SquaredPublisherNode::square_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("output", 10);
    }


private:
    void square_callback(const std_msgs::msg::Int32 & msg)
    {
        auto square = std_msgs::msg::Int32();
        square.data = msg.data * msg.data;
        RCLCPP_INFO(this->get_logger(), "Square of %d : %d ", msg.data, square.data);
        publisher_->publish(square);
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquaredPublisherNode>());
    rclcpp::shutdown();
    return 0;
}

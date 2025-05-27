#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SquaredPublisherNode : public rclcpp::Node
{
    //define the method variable in the class
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

public:
    //name this node
    SquaredPublisherNode() : Node("squared_numbers")
    {
        // define compatibile Qos for as the `number_publisher` node.
        rclcpp::QoS qos_profile(10);
        qos_profile.reliable();
        qos_profile.durability_volatile();

	    // initate subscriber which take a Int32 msg type and callback square_callback()
        subscription_ = this->create_subscription<std_msgs::msg::Int32>("number", qos_profile, std::bind(&SquaredPublisherNode::square_callback, this, std::placeholders::_1));
        //initiate a publisher which publish Int32 msg
	publisher_ = this->create_publisher<std_msgs::msg::Int32>("output", qos_profile);
    }


private:
    // take the data from topic 'number' and put into msg
    void square_callback(const std_msgs::msg::Int32 & msg)
    {
	// define a square variable with Int32 properties
	// and find the square of the value
        auto square = std_msgs::msg::Int32();
        square.data = msg.data * msg.data;
	//print the value into teh terminal
        RCLCPP_INFO(this->get_logger(), "Square of %d : %d ", msg.data, square.data);
        //publish the squared value ( can be taken by anyother node )
	publisher_->publish(square);
    }
};

//MAIN FUNCTION
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquaredPublisherNode>());
    rclcpp::shutdown();
    return 0;
}

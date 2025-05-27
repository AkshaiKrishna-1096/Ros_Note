#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class NumberPublisherNode : public rclcpp::Node
{
    // This are Member Variable in ROS style 
    // these belong tp each object of the class and is outside of the any method
    // the program won't work without this
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    size_t count_ = 0;

public:
    // this line will name the node
    NumberPublisherNode() : Node("number_publisher")
    {   
        // setting up QoS profile of the node
        rclcpp::QoS qos_profile(10);        // Depth : keep last 10 msg.
        qos_profile.reliable();             // Ensure all the msg are sent.
        qos_profile.durability_volatile();  // Do not persist msg.

        /* this create a publisher that pubish to the topic 'number' and data type Int32. also allocated queue size of 10 for buffer*/
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("number", qos_profile);
        /* set a timer that call timer_callback() every 500ms. this will also bind the member function to this object instance*/
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&NumberPublisherNode::timer_callback, this));
    }

  /* This part is made private to avoid accidental calling.
    this is not meant to be called directly by anything outside the node*/  
private:
    void timer_callback()
    {
        // create a msg of type Int32
        auto msg = std_msgs::msg::Int32();
        // assign the current value and then increment the count_
        msg.data = count_++;
        /* printf in ROS-style.
            this will display the output in terminal and console.
            we have differnet type for 'warn', 'error' and so on.*/
        RCLCPP_INFO(this->get_logger(), "Publishing: %d", msg.data);
        // finally publish the content in msg
        publisher_->publish(msg);
    }


};

int main(int argc, char* argv[])
{
    //initialise the node
    rclcpp::init(argc, argv);

    // keep it running until ctrl+c is pressed
    rclcpp::spin(std::make_shared<NumberPublisherNode>());
    
    //shutdown the node
    rclcpp::shutdown();
    return 0;
}

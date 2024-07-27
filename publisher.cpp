#include <chrono> // For time-related functions
#include <functional> // For std::bind
#include <memory> // For smart pointers
#include <string> // For std::string

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ client library
#include "std_msgs/msg/string.hpp" // Standard message type for strings

using namespace std::chrono_literals; // Allows using time literals like 500ms

// Define a PublisherNode class that inherits from rclcpp::Node
class PublisherNode : public rclcpp::Node
{
public:
  // Constructor: Initializes the node with the name "cpp_publisher" and count_ to 0
  PublisherNode() : Node("cpp_publisher"), count_(0)
  {
    // Create a publisher for the topic "chatter" with a queue size of 10
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    // Create a timer that calls timer_callback every 500 milliseconds
    timer_ = this->create_wall_timer(
        500ms, std::bind(&PublisherNode::timer_callback, this));
  }

private:
  // Timer callback function: Called every 500 milliseconds
  void timer_callback()
  {
    // Create a new String message
    auto message = std_msgs::msg::String();
    // Set the message data to "Hello, world! " followed by the current count
    message.data = "Hello, world! " + std::to_string(count_++);
    // Log the message being published
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // Publish the message to the "chatter" topic
    publisher_->publish(message);
  }

  // Member variables
  rclcpp::TimerBase::SharedPtr timer_; // Shared pointer to the timer
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // Shared pointer to the publisher
  size_t count_; // Counter for the number of messages published
};

// Main function
int main(int argc, char *argv[])
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);
  // Create and spin the PublisherNode, allowing callbacks to be processed
  rclcpp::spin(std::make_shared<PublisherNode>());
  // Shut down the ROS 2 system
  rclcpp::shutdown();
  return 0;
}

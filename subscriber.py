#!/usr/bin/env python3
# Shebang line to allow running the script as an executable

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for all ROS 2 nodes
from std_msgs.msg import String  # Standard message type for strings

# Define a SubscriberNode class that inherits from rclpy::Node
class SubscriberNode(Node):
    def __init__(self):
        # Initialize the node with the name "py_subscriber"
        super().__init__('py_subscriber')
        # Create a subscription to the topic "chatter" with a queue size of 10
        # and a callback function listener_callback
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    # Callback function: Called whenever a message is received on the "chatter" topic
    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info('I heard: "%s"' % msg.data)

# Main function
def main(args=None):
    # Initialize the ROS 2 system
    rclpy.init(args=args)
    # Create an instance of SubscriberNode
    subscriber = SubscriberNode()
    # Spin the node to keep it running and process incoming messages
    rclpy.spin(subscriber)
    # Clean up and destroy the node
    subscriber.destroy_node()
    # Shut down the ROS 2 system
    rclpy.shutdown()

# Entry point: Run the main function if the script is executed directly
if __name__ == '__main__':
    main()

import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def joy_2_cmd(msg, publisher):
    # This function will be called every time a message is received on the /joy topic.
    # In this example, we're publishing the received message to the /skidbot/cmd_vel topic.

    # Create a Twist message from the Joy message.
    twist_msg = Twist()
    twist_msg.linear.x = 2.0 * msg.axes[1]  # Map the y-axis of the joystick to the linear velocity.
    twist_msg.angular.y = 2.0 * msg.axes[4]  # Map the x-axis of the joystick to the angular velocity.
    twist_msg.angular.z = 2.0 * msg.axes[0]  # Map the x-axis of the joystick to the angular velocity.

    # Publish the Twist message to the /skidbot/cmd_vel topic.
    publisher.publish(twist_msg)

def main():
    rclpy.init()

    # Create a node that subscribes to the /joy topic and publishes to the /skidbot/cmd_vel topic.
    node = rclpy.create_node('joy_controller')
    subscription = node.create_subscription(Joy, '/joy', lambda msg: joy_2_cmd(msg, publisher), 10)
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    # Spin the node to receive messages and call the joy_callback function for each message.
    rclpy.spin(node)

    # Clean up before exiting.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleBouncingNode(Node):

    def __init__(self):
        super().__init__('turtle_bouncer')
        self.pose_listener = self.create_subscription(Pose, '/turtle1/pose', self.pin_turtle, 1)
        self.cmd_talker = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)
        self.cmd_pub_timer = self.create_timer(0.1, self.cmd_pub)
        # Variables
        self.is_out = False
        self.motion_mode = "forward"

    def pin_turtle(self, pose_msg):
        # self.get_logger().info('I heard: "%s"' % rumors.data)
        turtle_x = pose_msg.x
        turtle_y = pose_msg.y
        if turtle_x > 9 or turtle_y > 9:
            self.is_out = True

        else:
            self.is_out = False

        self.get_logger().info(f"Turtle's position: x={turtle_x}, y={turtle_y}\n Is turtle out? {self.is_out}")

    def cmd_pub(self):
        twist_msg = Twist()
        if self.is_out:
            twist_msg.linear.x = 0.
            twist_msg.angular.z = 0.7
        else:
            twist_msg.linear.x = 0.7
            twist_msg.angular.z = 0.
        self.cmd_talker.publish(twist_msg)
        self.get_logger().debug(f"Velocity command: {twist_msg}")

    



def main(args=None):
    rclpy.init(args=args)

    turtle_bouncer = TurtleBouncingNode()

    rclpy.spin(turtle_bouncer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_bouncer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

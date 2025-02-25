import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleBouncer(Node):
    def __init__(self):
        super().__init__("turtle_bouncer")
        self.cmd_vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        self.cmd_pub_timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10 Hz
        self.pose_subscriber = self.create_subscription(
            Pose, "turtle1/pose", self.pose_listener, 1
        )
        # Variables
        self.is_out = False
        self.struggle_counter = 0

    def pose_listener(self, pose_msg):
        x = pose_msg.x
        y = pose_msg.y
        if x < 1 or x > 9:
            self.is_out = True
        elif y < 1 or y > 9:
            self.is_out = True
        self.get_logger().info(f"Turtle x: {x}, y: {y}, is turtle out? {self.is_out}")

    def publish_cmd_vel(self):
        cmd_vel_msg = Twist()
        if self.is_out:
            if self.struggle_counter < 10:
                cmd_vel_msg.linear.x = -0.5
                cmd_vel_msg.angular.z = 0.0
                self.struggle_counter += 1
            elif 10 <= self.struggle_counter < 20:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 2.7
                self.struggle_counter += 1
            else:
                self.is_out = False
                self.struggle_counter = 0
        else:
            cmd_vel_msg.linear.x = 1.5
            cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().debug(f"Talking: {cmd_vel_msg}")


def main(args=None):
    rclpy.init(args=args)
    turtle_bouncer = TurtleBouncer()
    rclpy.spin(turtle_bouncer)
    turtle_bouncer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

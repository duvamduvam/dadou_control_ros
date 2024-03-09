import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk

from dadou_utils.utils_static import AUDIO, FACE, ROBOT_LIGHTS, RELAYS, LEFT_EYE, NECK, RIGHT_EYE, LEFT_ARM, RIGHT_ARM
from controller.gui.small_gui import SmallGui

PUBLISHER_LIST = [AUDIO, FACE, ROBOT_LIGHTS, RELAYS, NECK,LEFT_EYE, RIGHT_EYE, LEFT_ARM, RIGHT_ARM]


class Ros2TkinterApp(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.publisher = self.create_publisher(String, 'topic', 10)

        self.get_logger().info('starting controller node')

        self.action_publishers = {}
        for p in PUBLISHER_LIST:
            self.action_publishers[p] = self.create_publisher(String, p, 10)

        self.gui = SmallGui(self)

    def publish(self, animations_msg):
        if animations_msg and isinstance(animations_msg, dict):
            for k, v in animations_msg.items():
                if k in self.action_publishers:
                    self.get_logger().info("publish {} in {}".format(v, k))
                    msg = String()
                    msg.data = v
                    self.action_publishers[k].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ros_tk_app = Ros2TkinterApp()
    #rclpy.spin_once(ros_tk_app, timeout_sec=0.1)

    while rclpy.ok():
        ros_tk_app.gui.update()
        rclpy.spin_once(ros_tk_app, timeout_sec=0)

    ros_tk_app.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
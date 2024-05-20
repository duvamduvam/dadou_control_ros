import logging

from rclpy.node import Node
from robot_interfaces.msg._string_time import StringTime

from controller.control_config import config
from dadou_utils_ros.logging_conf import LoggingConf
from dadou_utils_ros.utils_static import LOGGING_FILE_NAME


class AbstractControllerNode(Node):
    def __init__(self, action_type, topic_name, action):
        self.action_type = action_type
        node_name = action_type + "_node"
        logging.config.dictConfig(LoggingConf.get(config[LOGGING_FILE_NAME], "controller"))

        super().__init__(node_name)

        self.action = action

        logging.info("Starting {} with topic {}".format(node_name, topic_name))

        self.subscription = self.create_subscription(
            StringTime,
            topic_name,
            self.listener_callback,
            10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def publish(self, animations_msg):
        if animations_msg and isinstance(animations_msg, dict):
            for k, v in animations_msg.items():
                if k in self.action_publishers:
                    logging.info("publish {} in {}".format(v, k))
                    msg = StringTime()
                    msg.msg = json.dumps(v)
                    if(DURATION in animations_msg):
                        msg.time = animations_msg[DURATION]
                    self.action_publishers[k].publish(msg)

    def listener_callback(self, ros_msg):
        msg = json.loads(ros_msg.msg)
        logging.info("input {} : ".format(ros_msg))
        action_msg = {self.action_type: msg}
        if ros_msg.time != 0:
            action_msg[DURATION] = ros_msg.time
        self.action.update(action_msg)
    def timer_callback(self):
        # Logique à exécuter en continu ici
        logging.debug('Action en temps réel')
        try:
            self.action.process()
        except Exception as e:
            logging.error(e, exc_info=True)
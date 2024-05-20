import logging
import logging.config
import time
import unittest
import pytest
import board

from controller.control_config import config
#from controller.control_config import config
from controller.tests.circuitpython.glove_keys import GloveKeys
from dadou_utils_ros.logging_conf import LoggingConf
from dadou_utils_ros.utils_static import LOGGING_TEST_FILE_NAME, LOGGING_FILE_NAME


#from dadou_utils_ros.logging_conf import LoggingConf
#from dadou_utils_ros.utils_static import LOGGING_TEST_FILE_NAME


class MyTestCase(unittest.TestCase):

    #logging.config.dictConfig(LoggingConf.get("/home/pi/deploy/conf/logging/logging-pi.conf", "tests"))
    logging.config.dictConfig(LoggingConf.get(config[LOGGING_TEST_FILE_NAME], "tests"))

    def test_glove(self):
        #print(dir(board))
        #print(config[LOGGING_TEST_FILE_NAME])

        #logging.info("test")

        keyboard = GloveKeys((("c", "b", "a"), ("f", "e", "d"), ("i", "h", "g"), ("l", "k", "j")),
                             (board.D16, board.D20, board.D21),
                             (board.D6, board.D13, board.D19, board.D26))

        logging.info("after init")

        keys = keyboard.check()
        #print(keys[0])

        while True:
            #logging.info("loop")
            keys = keyboard.check()
        #    =("truc")
            if len(keys) > 0 and keys[0]:
                logging.info(keys[0])
            time.sleep(0.1)


if __name__ == '__main__':
    unittest.main()

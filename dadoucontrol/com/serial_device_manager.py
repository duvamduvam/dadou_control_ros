from os.path import exists

import pyudev as pyudev
import logging

from dadoucontrol.control_config import ControlConfig
from dadoucontrol.files.control_json_manager import ControlJsonManager
from dadou_utils.com.serial_device import SerialDevice


class SerialDeviceManager:

    #TODO use config

    def __init__(self, jsonManager:ControlJsonManager):

        gloveLeftId = jsonManager.get_device_id("glove_left")
        self.gloveLeft = SerialDevice('glove left', gloveLeftId)

        context = pyudev.Context()
        monitor = pyudev.Monitor.from_netlink(context)
        monitor.filter_by(subsystem='usb')

    def checkNewDevice(self, serial_device) -> bool:
        #logging.info("check glove left plugged in " + str(serial_device.plugged))
        serial_plugged = exists(serial_device.serialPath)

        if not serial_device.plugged and serial_plugged:
            self.gloveLeft.connect()

        serial_device.plugged = serial_plugged
        #    logging.info('{} connected'.format(self.gloveLeftId))
        #if self.gloveLeft.plugedIn and not exists(self.gloveLeftId):
        #    self.gloveLeft.connect()
        #    logging.info('{} connected'.format(self.gloveLeftId))
        #elif not self.gloveLeft.plugedIn:
        #    self.gloveLeft.plugedIn = False
        return serial_device.plugged

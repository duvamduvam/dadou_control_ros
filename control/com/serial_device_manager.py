from os.path import exists

import pyudev as pyudev
import logging
from control.com.serial_device import SerialDevice


class SerialDeviceManager:


    #TODO use config
    USB_ID_PATH = "/dev/serial/by-id/"
    gloveLeftId = USB_ID_PATH + "usb-Raspberry_Pi_Pico_E6611CB6976B8D28-if00"
    gloveLeft = SerialDevice(gloveLeftId)

    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by(subsystem='usb')

    def checkNewDevice(self) -> bool:
        logging.info("check glove left plugged in " + str(self.gloveLeft.plugedIn))
        if(not self.gloveLeft.plugedIn and exists(self.gloveLeftId)):
            self.gloveLeft = SerialDevice(self.gloveLeftId)
            logging.info('{} connected'.format(self.gloveLeftId))
            return True
        return False

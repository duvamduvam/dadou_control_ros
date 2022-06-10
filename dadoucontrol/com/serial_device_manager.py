from os.path import exists

import pyudev as pyudev
import logging

from dadoutils.com.serial_device import SerialDevice


class SerialDeviceManager:

    #TODO use config

    gloveLeftId = SerialDevice.USB_ID_PATH + "usb-Raspberry_Pi_Pico_E6611CB6976B8D28-if00"
    gloveLeft = SerialDevice(gloveLeftId)

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

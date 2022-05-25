from builtins import staticmethod
from os.path import exists

from serial import SerialException

from control.message import Message
import serial
import logging
import sys

class SerialDevice:
    arduino_enable = True
    TERMINATOR = '\n'.encode('UTF8')

    leftGlove = "usb-Raspberry_Pi_Pico_E6611CB6976B8D28-if00"
    plugedIn = False

    def __init__(self, serialId):
        self.plugedIn = False
        serialPath = "/dev/serial/by-id/"+serialId
        try:
            device_exists = exists(serialPath)
            if(device_exists):
                self.device = serial.Serial(serialPath, 115200, timeout=1)
                logging.info('serial connected  : ' + serialId)
                self.plugedIn = True
        except SerialException:
            logging.error("serial id "+serialId+"not present\n", sys.exc_info()[0])
            pass

    def get_msg(self) -> Message:

        if self.plugedIn and self.arduino_enable and self.device.isOpen():
            # logging.info("{} connected!".format(self.arduino.port))
            if self.device.inWaiting() > 0:
                line = self.device.read_until(self.TERMINATOR)
                msg = line.decode('UTF8').strip()
                logging.info('received from arduino : ' + msg)
                return msg
        return None

    def send_msg(self, msg):
        if self.device.isOpen():
            self.device.write(msg)

    @staticmethod
    def decode(msg: str):
        if msg.startswith(Message.PREFIX) and msg.endswith(Message.POSTFIX):
            return Message(msg[1], msg[2], msg[3], msg[4] + msg[5])
        else:
            logging.error("wrong message : \"" + msg + "\"")
            return None
# This is a sample Python script.

# Press Maj+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
from control.com.serial_device import SerialDevice
from control.gui.gui_main import MainGui
import tkinter as tk
import logging.config


logging.config.fileConfig('/home/dadou/Nextcloud/Didier/python/DadouControl/conf/logging.conf', disable_existing_loggers=False)
logging.info('Starting didier')

leftGlove = SerialDevice(SerialDevice.leftGlove)

def main():
    gui = MainGui()
    gui.mainloop()

if __name__ == '__main__':
    main()


#while True:
#    msg = com.get_msg()
    #if msg:
    #    #audio.process(msg.key)
    #    face.update(msg.key)
    #    neck.update(msg.neck)
    #    wheel.update(msg.left_wheel, msg.right_wheel)
    #    lights.update(msg.key)
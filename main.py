# This is a sample Python script.
import platform
# Press Maj+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import sys
import logging
import os

from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.gui.normal_gui import NormalGui
from dadoucontrol.gui.small_gui import SmallGui

ControlFactory()

sys.path.append('..')

if len(sys.argv) > 1:
    device = sys.argv[1]
else:
    device = platform.uname()[1]
logging.info("Starting control device {}".format(device))

def main():
    try:
        if "right" in device or "left" in device:
            gui = SmallGui()
        elif "gp" in device or "gamepad" in device:
            from dadoucontrol.gui.gamepad import GamePadGui
            gui = GamePadGui()
        else:
            gui = NormalGui()
        gui.mainloop()
    except Exception as e:
        logging.error(e, exc_info=True)

if __name__ == '__main__':
    main()

# This is a sample Python script.

# Press Maj+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import sys
import logging
import os

from dadoucontrol.control_factory import ControlFactory

ControlFactory()

from dadoucontrol.gui.tkinter_gui import MainGui

logging.info('Starting remote control')

def main():
    try:
        gui = MainGui()
        gui.mainloop()
    except Exception as e:
        logging.error(e, exc_info=True)

if __name__ == '__main__':
    main()

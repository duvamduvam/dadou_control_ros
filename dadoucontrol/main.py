# This is a sample Python script.

# Press Maj+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import sys
import logging
import os
import traceback

import control_static
from control_factory import ControlFactory

base_path = os.path.dirname(__file__)
ControlFactory(base_path)

from gui.tkinter_gui import MainGui

sys.path.append('..')

logging.info('Starting remote control')

def main():
    try:
        gui = MainGui()
        gui.mainloop()
    except Exception as e:
        logging.error(e, exc_info=True)

if __name__ == '__main__':
    main()

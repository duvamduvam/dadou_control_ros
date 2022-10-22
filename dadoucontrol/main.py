# This is a sample Python script.

# Press Maj+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import sys
import logging
import os
from dadoucontrol.control_factory import ControlFactory
base_path = os.path.dirname(__file__)
ControlFactory(base_path)

from dadoucontrol.gui.tkinter_gui import MainGui

sys.path.append('..')



logging.info('Starting didier')

def main():
    gui = MainGui()
    gui.mainloop()

if __name__ == '__main__':
    main()

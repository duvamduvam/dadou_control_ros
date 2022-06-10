# This is a sample Python script.

# Press Maj+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import logging

from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.gui.tkinter_gui import MainGui

ControlFactory()

logging.info('Starting didier')

def main():
    gui = MainGui()
    gui.mainloop()

if __name__ == '__main__':
    main()

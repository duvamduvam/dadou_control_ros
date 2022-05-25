import tkinter as tk
from tkinter import Label
from tkinter import LabelFrame
from tkinter import E, W, N, S

from control.com.serial_device import SerialDevice
from control.com.serial_device_manager import SerialDeviceManager


class GloveFrame(tk.LabelFrame):

    #TODO add observer
    serialDevice = SerialDevice(SerialDevice.leftGlove)
    deviceManager = SerialDeviceManager()

    def __init__(self, parent, *args, **kwargs):
        tk.LabelFrame.__init__(self, parent)
        #self.grid(row=1, column=0, sticky=W + E)
        # Group1 Frame ----------------------------------------------------
        group1 = LabelFrame(self.master, text="Text Box", padx=5, pady=5)
        #group1.grid(row=1, column=0, columnspan=3, padx=10, pady=10, sticky=E + W + N + S)

        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(1, weight=1)

        group1.rowconfigure(0, weight=1)
        group1.columnconfigure(0, weight=1)

        # Create the textbox
        self.txtbox = Label(group1, font=('calibri', 40, 'bold'), width=40, height=50)
        self.txtbox.grid(row=0, column=0, sticky=E + W + N + S)
        self.txtbox.config(text="glove")

        self.checkNewUsb()
        self.gloveInput()



    def checkNewUsb(self) -> None:
        self.txtbox.after(1000, self.checkNewUsb)
        #if(self.deviceManager.checkNewDevice()):
        #    msg = "new usb !"
        #    self.txtbox.config(text=msg)
        #    self.serialDevice = SerialDevice(SerialDevice.leftGlove)


    def gloveInput(self) -> None:
        self.txtbox.after(100, self.gloveInput)
        #if(self.serialDevice.plugedIn):
        #    msg = self.serialDevice.get_msg()
        #    self.txtbox.config(text=msg)



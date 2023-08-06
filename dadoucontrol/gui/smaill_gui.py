import logging
import tkinter as tk
import time
from tkinter import TOP, BOTH, ttk, LEFT
from tkinter.messagebox import showinfo


from dadou_utils.com.input_messages_list import InputMessagesList
from dadou_utils.misc import Misc
from dadou_utils.utils.time_utils import TimeUtils
from dadou_utils.utils_static import ORANGE, BORDEAUX, YELLOW, CYAN, FONT1, PURPLE, FONT3, BASE_PATH, PATHS, ICONS, \
    BUTTON_GRID, CMD, DEVICE, MSG, MODE, CONTROL, PLAYLIST, CONFIG, DEFAULT, FONT2, HOST_NAME, ERROR

from dadoucontrol.control_config import config, FONT_DROPDOWN, SINGLE_GLOVE
from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.gui.windows.frames.widgets.Icons_widget import Icons_widget
from dadoucontrol.gui.windows.mod_window import ModWindow

from dadoucontrol.gui.windows.small.small_config import SmallConfig
from dadoucontrol.gui.windows.small.small_control import SmallControl
from dadoucontrol.gui.windows.small.small_playlist import SmallPlaylist
#from dadoucontrol.input.gampad import GamePad

MESSAGE_INPUT_TIMEOUT = 1000
MENU = [CONTROL, PLAYLIST, CONFIG]

class SmallGui(tk.Tk):
    def __init__(self, tkMessageBox=None, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.geometry("480x320")
        if config[HOST_NAME] != '5401':
            self.wm_attributes('-type', 'splash')
        self.bind('<Escape>', lambda e: self.destroy())
        screen_width = self.winfo_screenwidth()
        if screen_width < 1000:
            self.attributes("-fullscreen", True)

        self.menu = tk.Frame(self, height=60)
        self.menu.pack(fill='x', side=TOP)

        self.new_msg = False
        self.popup_closing = False

        self.listCombo, self.selected_menu = self.create_menu(MENU, 7)
        self.listCombo.bind('<<ComboboxSelected>>', self.menu_changed)
        self.mod_button = tk.Button(self.menu, text="M", width=2, font=FONT_DROPDOWN, command=lambda: self.change_window(MODE, None))
        self.mod_button.pack(side=LEFT)
        #self.listMod, self.selected_mod = self.create_combox(1,  [key for key in CONTROL_MOD], 2)
        #self.listMod.bind('<<ComboboxSelected>>', self.mod_changed)

        self.icons_widget = Icons_widget(self, self.menu)

        self.main = None
        self.change_window(PLAYLIST, DEFAULT)
        self.popup, self.popup_label, self.popup_info = self.create_popup()

        self.main.pack(fill=BOTH, side=TOP)

        self.check_input()
        self.check_sliders_input()
        self.check_remove_feedback_msg()

        #self.gamepad = GamePad()
        #self.check_gamepad()

    def create_menu(self, items, width):
        selected_menu = tk.StringVar()
        combo = ttk.Combobox(self.menu, values=items, textvariable=selected_menu, state="readonly", font=FONT_DROPDOWN, width=width)
        combo.option_add("*TCombobox*Listbox*Background", config[PURPLE])
        combo.option_add("*TCombobox*Listbox*Font", FONT_DROPDOWN)
        combo.current(0)
        combo.pack(side=LEFT)
        return combo, selected_menu

    # bind the selected value changes
    def menu_changed(self, event):
        self.change_window(self.selected_menu.get(), DEFAULT)

    def change_window(self, name, mode):
        #TODO switch to withdraw or deiconify
        if self.main:
            try:
                for child in self.main.winfo_children():
                    child.destroy()
            except Exception as e:
                logging.error(e)
            self.main.forget()
            self.main.destroy()

        if name == CONFIG:
            self.main = SmallConfig(self)
        elif name == CONTROL:
            self.main = SmallControl(self, mode)
        elif name == PLAYLIST:
            self.main = SmallPlaylist(self, mode)
        elif name == MODE:
            self.main = ModWindow(self)

        self.main.pack(fill=BOTH, expand=True, side=TOP)
        time.sleep(0.1)
        self.update_idletasks()

    def feedback_popup(self, input):
        if self.new_msg:
            self.feedback_message.destroy()
        self.feedback_message = tk.Message(self, )
        self.feedback_message.config(bg=config[BORDEAUX])
        input_var = tk.StringVar()
        input_var.set(input)
        self.feedback_msg_label = tk.Label(self.feedback_message, font=config[FONT1],
                                           textvariable=input_var, wraplength=500)
        self.feedback_msg_label.pack()
        self.feedback_message.pack(fill=BOTH, expand=True)
        self.new_msg = True
        self.feedback_message_time = TimeUtils.current_milli_time()


    def create_popup(self):
        popup = tk.Toplevel(self, bg=config[BORDEAUX])
        popup.wm_attributes('-type', 'splash')
        popup.geometry("400x100")
        popup_info = tk.StringVar()
        popup_label = tk.Label(popup, textvariable=popup_info, font=config[FONT2], bg=config[BORDEAUX])
        popup_label.place(x=30, y=30)
        popup.withdraw()
        return popup, popup_label, popup_info

    def show_popup(self, text):
        self.popup_info.set(text)
        self.popup.deiconify()
        if not self.popup_closing:
            self.after(3000, self.hide_popup)
            self.popup_closing = True

    def hide_popup(self):
        self.popup.withdraw()
        self.popup_closing = False

    def update_feedback_panel(self, label: tk.Label, activ: bool):
        if activ:
            label.config(bg=config[YELLOW])
        else:
            label.config(bg=config[BORDEAUX])

    def check_remove_feedback_msg(self):
        self.after(100, self.check_remove_feedback_msg)
        if self.new_msg and TimeUtils.is_time(self.feedback_message_time, MESSAGE_INPUT_TIMEOUT):
            self.feedback_message.destroy()
            self.new_msg = False

    def check_gamepad(self):
        self.after(150, self.check_gamepad())
        self.gamepad.check()

    def check_input(self):
        self.after(100, self.check_input)

        for device in ControlFactory().input_key_devices:
            msg = device.get_msg_separator()
            if msg:
                InputMessagesList().add_msg({DEVICE: device.name, MSG: msg})
                if self.icons_widget.hand_label:
                    self.icons_widget.glove_feedback()

    def check_sliders_input(self):
            self.after(100, self.check_sliders_input)

            if ControlFactory().sliders and len(ControlFactory().sliders) == 1:
                msg = ControlFactory().sliders[0].get_msg_separator()
                if msg:
                    #value = CONTROL_MOD[self.default_mod][KEYS_MAPPING[msg]][CMD]
                    logging.info("input sliers {}".format(msg))
                    self.show_popup("sliders : {}".format(msg))
                    ControlFactory().message.send_sliders(msg)




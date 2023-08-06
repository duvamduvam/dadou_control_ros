from tkinter import LEFT

from PIL import ImageTk, Image
import tkinter as tk

from dadou_utils.misc import Misc
from dadou_utils.utils.time_utils import TimeUtils
from dadou_utils.utils_static import BASE_PATH, PATHS, ICONS
from dadoucontrol.control_config import config
from dadoucontrol.control_factory import ControlFactory

GLOVE_FEEDBACK_TIMEOUT = 3000

class Icons_widget:
    def __init__(self, parent, menu):

        self.parent = parent
        self.menu = menu
        self.glove_feedback_time = 0
        ##### feedback icons
        icon_pos = 2
        if ControlFactory().input_connected(ControlFactory().input_key_devices, "glove"):
            self.hand_label, self.hand_icon, self.hand_image = self.create_label_icon("hand.png")

        if ControlFactory().input_connected(ControlFactory().sliders, "slider"):
            self.slider_label, self.slider_icon, self.hand_image = self.create_label_icon("sliders.png")

        if ControlFactory().device_connected('sceno'):
            self.sceno_label, self.sceno_icon, self.robot_image = self.create_label_icon("music.png")

        if ControlFactory().device_connected('harddrive'):
            self.helmet_label, self.helmet_icon, self.helmet_image = self.create_label_icon("helmet.png")

        if ControlFactory().device_connected('robot'):
            self.robot_label, self.robot_icon, self.robot_image = self.create_label_icon("robot.png")

        if Misc.wifi_connected():
            if Misc.internet_connected():
                self.wifi_label, self.wifi_icon, self.wifi_image = self.create_label_icon("wifi-green.png")
            else:
                self.wifi_label, self.wifi_icon, self.wifi_image = self.create_label_icon("wifi-red.png")

    def create_label_icon(self, icon):
        image = Image.open(config[BASE_PATH]+config[PATHS][ICONS]+icon)
        image = image.resize((35, 40))  # Redimensionner l'image à la taille souhaitée
        icon = ImageTk.PhotoImage(image)

        # Créer un widget Label pour afficher l'icône
        label = tk.Label(self.menu, image=icon)
        label.pack(side=LEFT)

        return label, icon, label

    def glove_feedback(self):
        self.hand_label.config(bg="red")
        self.glove_feedback_time = TimeUtils.current_milli_time()
        self.glove_feedback_timeout()

    def glove_feedback_timeout(self):
        if TimeUtils.is_time(self.glove_feedback_time, GLOVE_FEEDBACK_TIMEOUT):
            self.hand_label.config(bg="white")
        else:
            self.hand_label.config(bg="red")
            self.parent.after(100, self.glove_feedback_timeout)


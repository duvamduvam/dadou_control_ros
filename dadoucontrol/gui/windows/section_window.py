import logging
import tkinter as tk
from tkinter import BOTH, TOP, font

from dadou_utils.utils_static import FACE, LIGHTS, WHEELS, NECK

from control_static import CYAN, BORDEAUX, PURPLE, YELLOW, ORANGE, FONT1
from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.gui.windows.frames.abstract.rectangle_text import RectangleText
from dadoucontrol.gui.windows.frames.music_frame import MusicFrame
from dadoucontrol.gui.windows.frames.widgets.navigation_widget import NavigationWidget
from dadoucontrol.gui.windows.frames.neck_frame import NeckFrame
from dadoucontrol.gui.windows.frames.wheels_frame import WheelsFrame

from dadoucontrol.gui.windows.frames.widgets.sequences_widget import SequencesManagerWidget


class SectionFrame(tk.Frame):

    current_position = 0

    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, bg=CYAN, *args, **kwargs)
        #self.pack(fill=BOTH, expand=True, side=TOP)

        self.left_menu = tk.Frame(self, bg=CYAN, width=100)
        self.left_menu.pack(fill='y', side='left')

        music_frame = MusicFrame(self, BORDEAUX)

        NavigationWidget(self.left_menu, width=100)
        sequence_widget = SequencesManagerWidget(self, self.left_menu, music_frame)

        self.new_section = self.new_frame()

        self.expressions = ControlFactory().control_json_manager.get_expressions_names()
        #self.faces_frame = RectangleText2(self, 'face', 'green', expressions)
        self.lights = list(ControlFactory().control_json_manager.get_lights().keys())
        """self.lights_frame = RectangleText2(self, 'lights', 'yellow', lights)
        self.neck_frame = NeckFrame(self, 'orange')
        self.wheels_frame = WheelsFrame(self, 'violet')"""

        #sequence_widget.load_first_sequence()

    def new_frame(self):
        new_frame = tk.Frame(self.left_menu, bg=CYAN)
        new_frame.pack(fill='x', side=TOP)
        helv36 = font.Font(family='Helvetica', size=36, weight=font.BOLD)
        tk.Button(new_frame, text='new section', bg=PURPLE, font=FONT1, command=lambda: self.choice_popup())\
            .pack(ipadx=10, ipady=30, fill='x', expand=True, side='left')
        return new_frame

    def choice_popup(self):
        popup = tk.Toplevel()
        popup.wm_title("New section")
        popup.geometry("500x200")

        tk.Button(popup, text=FACE, bg=PURPLE, command=lambda: self.add_section(FACE)).pack(fill='x', side=TOP)
        tk.Button(popup, text=LIGHTS, bg=PURPLE, command=lambda: self.add_section(LIGHTS)).pack(fill='x', side=TOP)
        tk.Button(popup, text=NECK, bg=PURPLE, command=lambda: self.add_section(NECK)).pack(fill='x', side=TOP)
        tk.Button(popup, text=WHEELS, bg=PURPLE, command=lambda: self.add_section(WHEELS)).pack(fill='x', side=TOP)

    def add_section(self, section):
        logging.info('new section {}'.format(section))

        if section == FACE:
            RectangleText(self, FACE, CYAN, self.expressions)
        elif section == LIGHTS:
            RectangleText(self, LIGHTS, YELLOW, self.lights)
        elif section == WHEELS:
            WheelsFrame(self, PURPLE)
        elif section == NECK:
            NeckFrame(self, ORANGE)

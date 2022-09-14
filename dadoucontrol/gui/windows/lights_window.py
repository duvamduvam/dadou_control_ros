import inspect
import logging
import tkinter as tk
from tkinter import HORIZONTAL, DISABLED, ACTIVE, W, YES, NW, LEFT, TOP, RIGHT, X, N
from tkinter.colorchooser import askcolor

from dadou_utils.misc import Misc

from dadoucontrol.control_factory import ControlFactory


class LightsFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        json_manager = ControlFactory().control_json_manager

        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.pack(fill='both', expand=True, side='top')

        left_menu = tk.Frame(self, width=100, bg='blue')
        left_menu.pack(fill='y', ipadx=20, side='left')

        self.right_param = tk.Frame(self, bg='green', height=200)
        self.right_param.pack(fill='x', side='top')
        self.right_lights_control = tk.Frame(self, bg='orange', height=100)
        self.right_lights_control.pack(side=LEFT, anchor=N, expand=YES, fill=X)

        self.add_lights_button = tk.Button(self.right_lights_control, text="add")
        self.add_lights_button.grid(row=1, column=1)
        self.text_add = tk.Text(self.right_lights_control, width=15, height=1)
        self.text_add.grid(row=2, column=1)
        self.delete_lights_button = tk.Button(self.right_lights_control, text="delete")
        self.delete_lights_button.grid(row=3, column=1)
        self.save_lights_button = tk.Button(self.right_lights_control, text="save")
        self.save_lights_button.grid(row=4, column=1)

        self.lights_names = tk.StringVar()
        self.lights_values = json_manager.get_lights_elements()
        self.get_light_names(self.lights_values)
        self.lights_list = tk.Listbox(self.right_lights_control, listvariable=self.lights_names, height=10)
        self.lights_list.grid(row=1, column=2, rowspan=4)
        self.lights_list.bind('<<ListboxSelect>>', self.select_light)

        #lights params
        self.base_light_label = tk.StringVar()
        self.name_light_label = tk.StringVar()
        tk.Label(self.right_param, text='light base').grid(row=1, column=3)
        tk.Label(self.right_param, textvariable=self.base_light_label).grid(row=1, column=4)
        tk.Label(self.right_param, text='light name').grid(row=1, column=5)
        tk.Label(self.right_param, textvariable=self.name_light_label).grid(row=1, column=6)

        tk.Label(self.right_param, text='color', width=10).grid(row=2, column=1)
        self.color_scale = tk.Button(self.right_param, text='color', command=self.change_color)
        self.color_scale.grid(row=2, column=2, padx=10, pady=4)

        self.speed_scale = self.create_scale('speed', 3, 1, 0.001, 10, 200, 0.001, 0.01)
        self.size_scale = self.create_scale('size', 4, 1, 0, 100, 200, 1, 20)
        self.spacing_scale = self.create_scale('spacing', 2, 3, 0, 100, 200, 1, 0)
        self.step_scale = self.create_scale('step', 3, 3, 0, 50, 200, 1, 8)
        self.reverse_scale = self.create_scale('reverse', 4, 3, 0, 1, 200, 1, 0)
        self.period_scale = self.create_scale('period', 5, 3, 0, 100, 200, 1, 0)
        self.tail_length_scale = self.create_scale('tail length', 2, 5, 0, 100, 200, 1, 25)
        self.bounce_scale = self.create_scale('bounce', 3, 5, 0, 1, 200, 1, 1)
        self.ring_scale = self.create_scale('ring', 4, 5, 0, 1, 200, 1, 0)
        self.mask_scale = self.create_scale('mask', 5, 5, 0, 1, 200, 1, 0)
        self.num_sparkles_scale = self.create_scale('num sparkles', 2, 7, 1, 0, 200, 1, 1)
        self.max_intensity_scale = self.create_scale('max intensity', 3, 7, 0, 1, 200, 0.01, 1)
        self.min_intensity_scale = self.create_scale('min intensity', 4, 7, 0, 1, 200, 0.01, 0)
        self.precompute_rainbow_scale = self.create_scale('precompute rainbow', 2, 9, 0, 1, 200, 1, 1)
        self.colorwheel_offset_scale = self.create_scale('colorwheel offset', 3, 9, 0, 1, 200, 1, 1)
        self.background_brightness_scale = self.create_scale('background brightness', 4, 9, 0, 1, 200, 0.01, 0.2)

        self.lights_base = json_manager.get_lights_base()
        self.lights_base_names = tk.StringVar()
        self.get_base_names(self.lights_base) #json_manager.get_lights_base()
        self.lights_base_listbox = tk.Listbox(left_menu, listvariable=self.lights_base_names, height=16)
        self.lights_base_listbox.grid(row=1, column=6, padx=10)
        self.selected_sequence_var = tk.StringVar()
        self.lights_base_listbox.bind('<<ListboxSelect>>', self.select_base)

    def save_light(self):
        all_vars = vars(self)
        scale_params = {}
        for var in all_vars:
            if var.__contains__('_scale'):
                scale = getattr(self, var)
                if scale.cget('state') == ACTIVE:
                    scale_params[var.replace('_scale', '')] = scale.get()

        #self.lights_values[self.name_light_label.get()] =

        logging.info('truc')

    def select_base(self, evt):
        w = evt.widget
        index = int(w.curselection()[0])
        value = w.get(index)
        self.selected_sequence_var.set(value)
        #self.load_sequence(value)
        self.base_light_label.set(value)
        logging.info('You selected item %d: "%s"' % (index, value))
        self.enable_right_sliders(value)

    def select_light(self, evt):
        w = evt.widget
        index = int(w.curselection()[0])
        value = w.get(index)
        self.name_light_label.set(value)
        self.set_sliders(value)

    def get_base_names(self, bases):
        base_names = []
        for base in bases:
            base_names.append(base)
        self.lights_base_names.set(value=base_names)

    def get_light_names(self, elements):
        element_names = []
        for element in elements:
            element_names.append(list(element.keys()))
        self.lights_names.set(value=element_names)

    def change_color(self):
        colors = askcolor(title="Tkinter Color Chooser")
        self.configure(bg=colors[1])

    def create_scale(self, label, row, col, min, max, length, resolution, default):
        tk.Label(self.right_param, text=label, width=14).grid(row=row, column=col)
        scale = tk.Scale(self.right_param, from_=min, to=max, length=length,  resolution=resolution, orient=HORIZONTAL)
        scale.grid(row=row, column=col+1, padx=5, pady=6)
        scale.set(default)
        #state=DISABLED

        return scale

    def enable_right_sliders(self, name):
        try:
            all_vars = vars(self)
            for var in all_vars:
                if var.__contains__('_scale'):
                    scale = getattr(self, var)
                    scale.configure(state=DISABLED, bg='grey')

            scale_params = self.lights_base[name]
            for param in scale_params:
                scale = getattr(self, param+'_scale')
                scale.configure(state=ACTIVE, bg='orange')

        except AttributeError as err:
            logging.error(err)

    def set_sliders(self, light):
        params = Misc.get_dict_value(self.lights_values, light)
        light_base = list(params.keys())[0]
        self.base_light_label.set(light_base)
        self.enable_right_sliders(light_base)
        for param in params.get(light_base):
            scale = getattr(self, list(param.keys())[0] + '_scale')
            scale.set(list(param.values())[0])

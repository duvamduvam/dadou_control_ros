import inspect
import logging
import tkinter as tk
from tkinter import HORIZONTAL, DISABLED, ACTIVE, W, YES, NW, LEFT, TOP, RIGHT, X, N, END, NORMAL, BOTH
from tkinter.colorchooser import askcolor

from dadou_utils.misc import Misc

from control_static import CYAN, YELLOW, ORANGE
from control_factory import ControlFactory


class LightsFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        self.json_manager = ControlFactory().control_json_manager

        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.pack(fill=BOTH, expand=True, side=TOP)

        left_menu = tk.Frame(self, width=100, bg=CYAN)
        left_menu.pack(fill='y', ipadx=20, side=LEFT)

        self.right_param = tk.Frame(self, bg=YELLOW, height=200)
        self.right_param.pack(fill='x', side=TOP)
        self.right_lights_control = tk.Frame(self, bg=ORANGE, height=100)
        self.right_lights_control.pack(side=LEFT, anchor=N, expand=YES, fill=X)

        self.add_lights_button = tk.Button(self.right_lights_control, command=self.add_light,  text="add")
        self.add_lights_button.grid(row=1, column=1)
        self.text_add = tk.Text(self.right_lights_control, width=15, height=1)
        self.text_add.grid(row=2, column=1)
        self.delete_lights_button = tk.Button(self.right_lights_control, command=self.delete_light, text="delete")
        self.delete_lights_button.grid(row=3, column=1)
        self.save_lights_button = tk.Button(self.right_lights_control, command=self.save_light, text="save")
        self.save_lights_button.grid(row=4, column=1)

        self.lights_names = tk.StringVar()
        self.lights_values = self.json_manager.get_lights()
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

        self.color_label = tk.Label(self.right_param, text='color', width=10)
        self.color_label.grid(row=2, column=1)
        self.color_scale = tk.Button(self.right_param, bg=ORANGE, text='color', command=self.change_color)
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

        self.lights_base = self.json_manager.get_lights_base()
        self.lights_base_names = tk.StringVar()
        self.get_base_names(self.lights_base) #json_manager.get_lights_base()

        self.lights_base_listbox.grid(row=1, column=6, padx=10)
        self.selected_sequence_var = tk.StringVar()
        self.lights_base_listbox.bind('<<ListboxSelect>>', self.select_base)

    def save_light(self):
        all_vars = vars(self)
        scale_params = {}
        for var in all_vars:
            if var.__contains__('_scale'):
                scale = getattr(self, var)
                if scale.cget('state') == ACTIVE or scale.cget('state') == NORMAL:
                    if'color' not in var:
                        scale_params[var.replace('_scale', '')] = scale.get()
                    else:
                        scale_params['color'] = self.get_color()

        scale_params["base"] = self.base_light_label.get()
        if self.name_light_label.get():
            self.lights_values[self.name_light_label.get()] = scale_params
        self.json_manager.save_lights(self.lights_values)
        logging.info('truc')

    def select_base(self, evt):
        w = evt.widget
        if len(w.curselection()):
            index = int(w.curselection()[0])
            value = w.get(index)
            self.selected_sequence_var.set(value)
            #self.load_sequence(value)
            self.base_light_label.set(value)
            logging.info('You selected item %d: "%s"' % (index, value))
            self.enable_right_sliders(value)

    def select_light(self, evt):
        w = evt.widget
        if len(w.curselection()):
            index = int(w.curselection()[0])
            value = w.get(index)
            self.name_light_label.set(value)
            self.base_light_label.set(self.lights_values[value]["base"])
            self.set_sliders(value)

    def get_base_names(self, bases):
        base_names = []
        for base in bases:
            base_names.append(base)
        self.lights_base_names.set(value=base_names)

    def get_light_names(self, elements):
        element_names = []
        for element in elements:
            element_names.append(element)
        self.lights_names.set(value=element_names)

    def change_color(self):
        colors = askcolor(title="Tkinter Color Chooser")
        self.set_color(colors[1])

    def set_color(self, color):
        self.color_label.configure(bg=color)

    def get_color(self):
        return self.color_label.cget("bg")

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
                    scale.configure(state=DISABLED, bg=CYAN)

            scale_params = self.lights_base[name]
            for param in scale_params:
                scale = getattr(self, param+'_scale')
                scale.configure(state=ACTIVE, bg=ORANGE)
                #scale.refresh()

        except AttributeError as err:
            logging.error(err)

    def set_sliders(self, light):
        self.enable_right_sliders(self.lights_values[light]["base"])
        for param in self.lights_values[light]:
            if type(self.lights_values[light][param]) == int:
                scale = getattr(self, param + '_scale')
                scale.set(self.lights_values[light][param])
            elif 'color' in param:
                self.set_color(self.lights_values[light][param])

    def add_light(self):
        new_light = self.text_add.get(1.0, END+"-1c")
        self.name_light_label.set(new_light)
        self.save_light()
        self.lights_values = self.json_manager.get_lights()
        self.get_light_names(self.lights_values)

    def delete_light(self):
        self.name_light_label.set('')
        delete_index = self.lights_list.curselection()[0]
        to_delete = self.lights_list.get(delete_index)
        self.lights_values.pop(to_delete, None)
        self.save_light()
        self.lights_values = self.json_manager.get_lights()
        self.get_light_names(self.lights_values)

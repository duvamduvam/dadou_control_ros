import logging
import tkinter as tk

from dadou_utils.singleton import SingletonMeta

from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.gui.expression_duration import ExpressionDuration
from dadoucontrol.gui.gui_utils import GuiUtils
from dadoucontrol.gui.visuals_object.visual_eye import VisualEye
from dadoucontrol.gui.visuals_object.visual_mouth import VisualMouth
from dadoucontrol.gui.windows.frames.abstract.rectangle_frame_image import RectangleFrameImage
from dadoucontrol.gui.windows.frames.timeline_frame import TimeLineFrame
from dadoucontrol.gui.windows.frames.widgets.galley_widget import GalleryWidget
from dadoucontrol.gui.windows.frames.abstract.rectangle_frame import RectangleFrame
from dadoucontrol.gui.windows.frames.widgets.image_observer_feedback import ImageObserverFeedBack


class ExpressionFrame(tk.Frame):

    def __init__(self, parent, *args, **kwargs):

        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.pack(fill='both', expand=True, side='top')

        left_menu = tk.Frame(self, width=50, bg='blue')
        left_menu.pack(fill='y', ipadx=20, side='left')

        GalleryWidget(left_menu, VisualEye('truc'), 10, width=100, height=800).grid(row=0, column=0, columnspan=2)
        GalleryWidget(left_menu, VisualMouth('truc'), 6, width=150, height=800).grid(row=0, column=2, columnspan=2)

        top_frame = tk.Frame(self, width=800, bg='blue')
        self.top_canvas = tk.Canvas(top_frame, height=300, bg='purple')
        self.top_canvas.grid(row=0, column=0, rowspan=5, columnspan=5)

        #self.right_eye = GuiUtils.set_image(top_canvas, 10, 10, VisualEye.TYPE, 'oeil-droit.png', 10)
        #self.lef_eye = GuiUtils.set_image(top_canvas, 210, 10, VisualEye.TYPE, 'oeil-gauche.png', 10)
        #self.mouth = GuiUtils.set_image(top_canvas, 30, 120, VisualMouth.TYPE, 'bouche-ferme.png', 10)

        play_button = tk.Button(top_frame, text='play')
        play_button.grid(row=1, column=6, padx=10)
        pause_button = tk.Button(top_frame, text='pause')
        pause_button.grid(row=1, column=7, padx=10)
        stop_button = tk.Button(top_frame, text='stop')
        stop_button.grid(row=1, column=8, padx=10)
        tk.Button(top_frame, text='loop').grid(row=1, column=9, padx=10)

        tk.Label(top_frame, text='length', width=5).grid(row=2, column=6)
        tk.Button(top_frame, text='-', command=self.decrement).grid(row=2, column=7)
        tk.Button(top_frame, text='+', command=self.increment).grid(row=2, column=8)
        self.expression_duration = tk.StringVar()
        self.expression_duration.set('0')
        tk.Label(top_frame, textvariable=self.expression_duration, width=5).grid(row=2, column=9)

        tk.Button(top_frame, text='load', command=self.load).grid(row=3, column=6, padx=10)
        tk.Button(top_frame, text='save', command=self.save).grid(row=3, column=7, padx=10)
        tk.Button(top_frame, text='new', command=self.add).grid(row=3, column=8, padx=10)
        tk.Button(top_frame, text='delete', command=self.delete).grid(row=3, column=9, padx=10)

        self.new_expression_name = tk.Text(top_frame, width=10, height=1)
        self.new_expression_name.grid(row=4, column=6, padx=10)

        self.current_expression_name = tk.StringVar()
        current_expression_label = tk.Label(top_frame, textvariable=self.current_expression_name)
        current_expression_label.grid(row=4, column=8, columnspan=2)

        self.expressions_var = tk.StringVar()
        self.expression_list = tk.Listbox(top_frame, listvariable=self.expressions_var, height=16)
        self.expression_list.grid(row=1, column=10, rowspan=4)
        self.expression_list.bind('<<ListboxSelect>>', self.select_expression)

        top_frame.pack(fill='x', side='top')

        self.time_line = TimeLineFrame(self)
        play_button.configure(command=self.time_line.play)
        pause_button.configure(command=self.time_line.pause)
        stop_button.configure(command=self.time_line.stop)

        self.right_eye_frame = RectangleFrameImage(self, VisualEye.TYPE, 'Left eye', 'yellow')
        self.left_eye_frame = RectangleFrameImage(self, VisualEye.TYPE, 'Right eye', 'orange')
        self.mouth_frame = RectangleFrameImage(self, VisualMouth.TYPE, 'Mouths', 'red')

        self.right_eye_observer = ImageObserverFeedBack(self.top_canvas, self.right_eye_frame, 10, 10, VisualEye.TYPE)
        self.left_eye_observer = ImageObserverFeedBack(self.top_canvas, self.left_eye_frame, 210, 10, VisualEye.TYPE)
        self.mouth_observer = ImageObserverFeedBack(self.top_canvas, self.mouth_frame, 30, 120, VisualMouth.TYPE)

        self.load_listbox()

    def increment(self):
        self.expression_duration.set(str(int(self.expression_duration.get())+1))
        self.time_line.duration = int(self.expression_duration.get())
        ExpressionDuration.value = int(self.expression_duration.get())

    def decrement(self):
        self.expression_duration.set(str(int(self.expression_duration.get())-1))
        self.time_line.duration = int(self.expression_duration.get())
        ExpressionDuration.value = int(self.expression_duration.get())

    def load_listbox(self):
        expressions = ControlFactory().control_json_manager.get_expressions()
        results = []
        for expression in expressions:
            results.append(expression['name'])
        self.expressions_var.set(results)

    def add(self):
        new_name = self.new_expression_name.get("1.0", 'end-1c')
        self.current_expression_name.set(new_name)
        self.save()
        self.load_listbox()
        #logging.info('add new expression')

    def delete(self):
        name = self.current_expression_name.get()
        logging.info('delete expression '.format(name))
        ControlFactory().control_json_manager.delete_expression(name)
        self.load_listbox()

    def save(self):
        ControlFactory().control_json_manager.save_expressions(
            self.current_expression_name.get(),
            ExpressionDuration.value,
            self.right_eye_frame.export(),
            self.left_eye_frame.export(),
            self.mouth_frame.export()
        )

    def load(self):
        name = self.current_expression_name.get()
        expression = ControlFactory().control_json_manager.get_expressions_name(name)
        ExpressionDuration.value = expression['duration']
        self.expression_duration.set(ExpressionDuration.value)
        self.time_line.duration = int(self.expression_duration.get())
        self.right_eye_frame.load(expression['left_eyes'])
        self.left_eye_frame.load(expression['right_eyes'])
        self.mouth_frame.load(expression['mouths'])

    def select_expression(self, e):
        w = e.widget
        if(len(w.curselection())>0):
            index = int(w.curselection()[0])
            value = w.get(index)
            self.current_expression_name.set(value)
        else:
            logging.error("no line selected")





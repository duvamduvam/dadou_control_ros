import logging
import random
from enum import Enum
import tkinter as tk

from dadoucontrol.files.file_manager import FileManager
from dadoucontrol.gui.gui_utils import GuiUtils
from dadoucontrol.gui.windows.expression_window import ExpressionDuration
from dadoucontrol.gui.windows.frames.abstract.abstract_sequence_frame import AbstractSequenceFrame
from dadoucontrol.gui.windows.frames.timeline_frame import TimeLineFrame


class CanvasType(Enum):
    BAR = 1
    RECTANGLE = 2


class RectangleFrameImage(AbstractSequenceFrame):
    def __init__(self, parent, visual_type, name, color):
        super().__init__(parent, name, color)
        self.visual_type = visual_type
        logging.info('rectangle 2')
        self.rectangle_bars = []
        self.lastX = 0
        self.y1 = 40
        self.y2 = 120
        self.images = []

        self.items = FileManager.list_folder_files(visual_type)
        self.canvas.bind("<Double-Button-1>", self.create_rectangle_click)
        self.canvas_root_x = self.canvas.winfo_rootx()
        self.current_image_index = 0
        self.rectangle_bar_index = 0
        self.focus_bar = 0

    def create_rectangle(self, x1, x2):
        #rectangle = self.canvas.create_rectangle(x1, self.y1, x2, self.y2, fill="pink", )
        rectangle_canvas = tk.Canvas(self.canvas, width=x2-x1, bg=self.random_color())
        rectangle_canvas.place(x=x1, y=10)

        bar = rectangle_canvas.create_line(x2-x1, self.y1, x2-x1, self.y2, width=5)
        rectangle_canvas.bind('<Button-3>', self.delete_click)
        rectangle_canvas.bind('<Button-2>', self.scroll_item)
        rectangle_canvas.bind("<Double-Button-1>", self.create_rectangle_click)
        #self.canvas.tag_bind(rectangle, '<Button-2>', self.scroll_item)
        rectangle_canvas.bind('<Enter>', self.bar_focus_on)
        rectangle_canvas.bind('<Shift-ButtonRelease-1>', self.click_resize)
        #rectangle_canvas.bind('<Shift-B1-Motion>', self.click_resize)
        rectangle_canvas.bind('<Leave>', self.bar_focus_out)
        rectangle_bar = RectangleBar(self.rectangle_bar_index, x1, x2, rectangle_canvas, bar, self.canvas)
        image = self.attach_item_index(rectangle_bar.canvas_rectangle,  self.current_image_index)
        rectangle_bar.image_name = image[0]
        rectangle_bar.image = image[1]

        rectangle_bar.time = int(x2 / self.canvas.winfo_width() * ExpressionDuration.value)

        self.rectangle_bars.append(rectangle_bar)
        self.rectangle_bar_index += 1
        self.lastX = x2
        self.set_all_bar_on_top()
        return rectangle_bar

    def create_rectangle_click(self, e):
        if len(self.rectangle_bars) == 0:
            self.create_rectangle(0, self.canvas.winfo_width())
        else:
            rectangle_bar = self.find_canvas_rectangle(e.widget)
            self.split_rectangle(rectangle_bar, e.x, e.x_root)

    def split_rectangle(self, rectangle_bar, x, x_root):
        rectangle_bar.canvas_rectangle.config(width=x)
        x1 = x_root - self.canvas_root_x
        self.create_rectangle(x1, rectangle_bar.x2)
        rectangle_bar.x2 = x1

    def delete_click(self, e):
        rectangle_bar = self.find_canvas_rectangle(e.widget)
        self.delete_canvas(rectangle_bar, True)

    def delete_canvas(self, rectangle_bar, fill_gap):
        x1 = rectangle_bar.canvas_rectangle.winfo_x()
        x2 = x1+rectangle_bar.canvas_rectangle.winfo_width()

        logging.info('x1:{} x2:{}'.format(x1, x2))
        rectangle_bar.canvas_rectangle.destroy()
        #self.canvas.delete(rectangle_bar.bar_id)
        self.rectangle_bars.remove(rectangle_bar)
        if fill_gap:
            self.fill_gap(x1, x2)

    def fill_gap(self, removed_x1, removed_x2):
        for rectangle_bar in self.rectangle_bars:
            logging.info('rectangle bar x:{} removed_x2:{}'.format(rectangle_bar.x1, removed_x2))
            if abs(removed_x2 - rectangle_bar.x1) <= 3:
                rectangle_bar.canvas_rectangle.update()
                x1 = removed_x1
                x2 = rectangle_bar.x2
                self.delete_canvas(rectangle_bar, False)
                self.create_rectangle(x1, x2)
        self.lastX = removed_x1


    def find_canvas_rectangle(self, rectangle):
        for rectangle_bar in self.rectangle_bars:
            if rectangle_bar.canvas_rectangle == rectangle:
                return rectangle_bar
        print("no matching rectangle bar")

    def set_all_bar_on_top(self):
        for rectangle_bar in self.rectangle_bars:
            self.canvas.tag_raise(rectangle_bar.bar_id)

    def bar_focus_on(self, e):
        rectangle_bar = self.find_canvas_rectangle(e.widget)

        if rectangle_bar and abs(e.x-rectangle_bar.canvas_rectangle.coords(rectangle_bar.bar_id)[0]) < 5:
            rectangle_bar.canvas_rectangle.itemconfig(rectangle_bar.bar_id, width=20)

            logging.debug("bar focus id {} click x {} x2 {} cord {} ".format(rectangle_bar.bar_id, e.x,
                rectangle_bar.x2, e.x-rectangle_bar.canvas_rectangle.coords(rectangle_bar.bar_id)[0]))

    def click_resize(self, e):
        rectangle_bar = self.find_canvas_rectangle(e.widget)
        logging.debug("resize {}".format(e.x))
        self.resize(rectangle_bar, e.x, e.x_root)

    def bar_focus_out(self, e):
        rectangle_bar = self.find_canvas_rectangle(e.widget)
        if rectangle_bar:
            rectangle_bar.canvas_rectangle.itemconfig(rectangle_bar.bar_id, width=5)

    def is_bar(self, canvas_id):
        for rectangle_bar in self.rectangle_bars:
            if rectangle_bar.bar_id == canvas_id[0]:
                return True

    def scroll_item(self, e):
        rectangle_bar = self.find_canvas_rectangle(e.widget)
        rectangle_bar.canvas_rectangle.delete(rectangle_bar.image)
        self.current_image_index = (self.current_image_index + 1) % len(self.items)
        #TODO update this bullshit part
        text = self.attach_item_index(rectangle_bar.canvas_rectangle,  self.current_image_index)
        rectangle_bar.text = text
        rectangle_bar.image_name = text[0]
        self.images.append(rectangle_bar.image)

    def attach_item_index(self, canvas, item_index):
        return [self.items[item_index], GuiUtils.set_image(canvas, 0, 0, self.visual_type, self.items[item_index], 5)]

    def attach_item_name(self, canvas, name):
        return GuiUtils.set_image(canvas, 0, 0, self.visual_type, name, 5)

    def export(self):
        result = []
        for rectangle_bar in self.rectangle_bars:
            pos = round(rectangle_bar.x2 / self.canvas.winfo_width(), 3)
            result.append([rectangle_bar.image_name, pos])
        return result

    def load(self, datas):
        for data in datas:
            image_name = data[0]
            pos = data[1]
            x2 = int(pos * self.canvas.winfo_width())
            rectangle_bar = self.create_rectangle(self.lastX, x2)
            image = self.attach_item_name(rectangle_bar.canvas_rectangle, image_name)
            rectangle_bar.image = image
            rectangle_bar.image_name = image_name
            #rectangle_bar.time = time

    def resize(self, rectangle_bar, width, x_pos):
        x_pos = x_pos - self.canvas_root_x
        rectangle_bar.canvas_rectangle.update()
        rectangle_bar.canvas_rectangle.config(width=width)
        rectangle_bar.canvas_rectangle.delete(rectangle_bar.bar_id)
        rectangle_bar.bar_id = rectangle_bar.canvas_rectangle.create_line(x_pos, self.y1, x_pos, self.y2, width=5)
        #rectangle_bar.canvas_rectangle.move(rectangle_bar.bar_id, x_pos-rectangle_bar.x2, 0)
        rectangle_bar.x2 = x_pos
        #logging.debug("resize width {} old width {} move bar {}".format(width, old_width, move_x))

        self.move_next_rectangle(rectangle_bar)

    def move_next_rectangle(self, input_rectangle_bar):
        next_rectangle_bar = self.get_next_rectangle(input_rectangle_bar.index)
        if next_rectangle_bar:
            self.rectangle_bars.remove(next_rectangle_bar)
            next_rectangle_bar.canvas_rectangle.destroy()
            new_rectangle_bar = self.create_rectangle(input_rectangle_bar.x2, next_rectangle_bar.x2)
            new_rectangle_bar.index = next_rectangle_bar.index
            logging.debug("move next rectangle {} {} ".format(input_rectangle_bar.x2, next_rectangle_bar.x2))
        else:
            logging.debug("no rectangle after {} ".format(input_rectangle_bar.index))

    def get_next_rectangle(self, index):
        if index < len(self.rectangle_bars):
            for rectangle_bar in self.rectangle_bars:
                logging.info("next rectangle index {}".format(rectangle_bar.index))
                if index + 1 == rectangle_bar.index:
                    return rectangle_bar
        return None

    def random_color(self):
        return "#{:06x}".format(random.randint(0, 0xFFFFFF))

    def current_image(self):
        x = TimeLineFrame.x_pos
        for rectangle_bar in self.rectangle_bars:
            if rectangle_bar.x1 <= x <= rectangle_bar.x2:
                #logging.debug("current frame image {}".format(rectangle_bar.image_name))
                return rectangle_bar.image_name





class RectangleBar:
    image = None
    image_name = None
    name = None
    time = 0

    def __init__(self, index, x1, x2, rectangle, bar, canvas):
        self.index = index
        self.x1 = x1
        self.x2 = x2
        self.canvas_rectangle = rectangle
        self.bar_id = bar
        self.canvas = canvas

    def set_image(self, image):
        self.image = image


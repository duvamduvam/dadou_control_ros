from tkinter import NW

import PIL
from PIL import ImageTk
from dadou_utils.misc import Misc
from dadou_utils.singleton import SingletonMeta


class HighlightedRectangle(metaclass=SingletonMeta):
    rectangle = None
    zoom = 8

    @staticmethod
    def set_image(image):
            canvas = HighlightedRectangle.rectangle.canvas_rectangle
            canvas.configure(bg=Misc.random_color())
            canvas.delete("all")
            #canvas = tk.Canvas(parent, width=self.tk_image.width(), height=self.tk_image.height())
            canvas.create_image(0, 0, anchor=NW, image=image)
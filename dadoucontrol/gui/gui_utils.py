import tkinter as tk

import PIL
from PIL import Image, ImageTk

from dadoucontrol.control_factory import ControlFactory


class GuiUtils:

    @staticmethod
    def set_image(parent, x, y, image_type, image, zoom):
        folder = ControlFactory().control_json_manager.get_folder_path_from_type(image_type)
        image = PIL.Image.open(folder+'/'+image)
        tk_image = ImageTk.PhotoImage(image)
        tk_image = tk_image._PhotoImage__photo.zoom(zoom)
        parent.create_image(x, y, anchor=tk.NW, image=tk_image)
        return tk_image

    @staticmethod
    def set_text(parent, x, y, text):
        return parent.create_text(x+5, y+120, text=text, angle=90)


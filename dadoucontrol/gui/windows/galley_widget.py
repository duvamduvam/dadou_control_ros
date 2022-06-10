import logging
import tkinter as tk
import tkinter
from os import listdir
from os.path import isfile, join

from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.files.file_manager import FileManager
from dadoucontrol.gui.gui_utils import GuiUtils


class GalleryWidget(tkinter.Canvas):
    def __init__(self, parent, visual_type, zoom, *args, **kwargs):
        tk.Canvas.__init__(self, parent, *args, **kwargs)

        items = FileManager.list_folder_files(visual_type.TYPE)
        xpos = 10
        ypos = 10
        ymargin = 10
        self.images = []
        for item in items:
            self.images.append(GuiUtils.set_image(self, xpos, ypos, visual_type.TYPE, item, zoom))
            ypos += ymargin + visual_type.HEIGHT * zoom



import tkinter as tk

from PIL import Image, ImageTk

from dadoucontrol.com.serial_device_manager import SerialDeviceManager
from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.gui.gui_utils import GuiUtils
from dadoucontrol.gui.visuals_object.visual_eye import VisualEye
from dadoucontrol.gui.visuals_object.visual_mouth import VisualMouth
from dadoucontrol.gui.windows.galley_widget import GalleryWidget
from dadoucontrol.gui.windows.sequences.face_frame import FaceFrame


class ExpressionFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.pack(fill='both', expand=True, side='top')

        left_menu = tk.Frame(self, width=50, bg='blue')
        left_menu.pack(fill='y', ipadx=20, side='left')
        tk.Label(left_menu, text='length', width=5).grid(row=0, column=0)
        tk.Text(left_menu, height=1, width=20).grid(row=0, column=1)

        GalleryWidget(left_menu, VisualEye('truc'), 10, width=100, height=800).grid(row=1, column=0)
        GalleryWidget(left_menu, VisualMouth('truc'), 6, width=200, height=800).grid(row=1, column=1)

        top_frame = tk.Frame(self, width=800, bg='blue')
        top_canvas = tk.Canvas(top_frame, height=300, bg='purple')
        top_canvas.grid(row=0, column=0, rowspan=5, columnspan=5)

        self.right_eye = GuiUtils.set_image(top_canvas, 10, 10, VisualEye.TYPE, 'oeil-droit.png', 10)
        self.lef_eye = GuiUtils.set_image(top_canvas, 210, 10, VisualEye.TYPE, 'oeil-gauche.png', 10)
        self.mouth = GuiUtils.set_image(top_canvas, 30, 120, VisualMouth.TYPE, 'bouche-ferme.png', 10)

        tk.Button(top_frame, text='play', ).grid(row=1, column=6, padx=10)
        tk.Button(top_frame, text='pause').grid(row=2, column=6, padx=10)
        tk.Button(top_frame, text='stop').grid(row=3, column=6, padx=10)
        tk.Button(top_frame, text='loop').grid(row=4, column=6, padx=10)

        top_frame.pack(fill='x', side='top')

        FaceFrame(self, 'Left eye', 'yellow')
        FaceFrame(self, 'Right eye', 'orange')
        FaceFrame(self, 'Mouths', 'red')

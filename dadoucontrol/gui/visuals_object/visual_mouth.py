from dadoucontrol.gui.visuals_object.visual_object import VisualObject
from utils_static import MOUTH


class VisualMouth(VisualObject):
    TYPE = MOUTH
    WIDTH = 24
    HEIGHT = 16

    def __init__(self):
        super().__init__(MOUTH)


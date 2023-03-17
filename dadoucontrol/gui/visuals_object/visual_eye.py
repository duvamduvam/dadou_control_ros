from gui.visuals_object.visual_object import VisualObject
from dadou_utils.utils_static import EYE

from utils_static import EYES


class VisualEye(VisualObject):

    TYPE = EYES
    WIDTH = 8
    HEIGHT = 8

    def __init__(self):
        super().__init__(EYE)


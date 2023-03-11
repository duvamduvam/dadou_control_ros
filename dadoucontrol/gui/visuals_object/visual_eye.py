from gui.visuals_object.visual_object import VisualObject
from dadou_utils.utils_static import EYE


class VisualEye(VisualObject):

    TYPE = EYE
    WIDTH = 8
    HEIGHT = 8

    def __init__(self):
        super().__init__(EYE)


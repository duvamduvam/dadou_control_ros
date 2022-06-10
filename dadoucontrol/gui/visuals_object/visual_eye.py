from dadoucontrol.gui.visuals_object.visual_object import VisualObject


class VisualEye(VisualObject):

    TYPE = 'eye'
    WIDTH = 8
    HEIGHT = 8

    def __init__(self, name):
        super().__init__(name)


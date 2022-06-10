from dadoucontrol.gui.visuals_object.visual_object import VisualObject


class VisualMouth(VisualObject):
    TYPE = 'mouth'
    WIDTH = 24
    HEIGHT = 16

    def __init__(self, name):
        super().__init__(name)


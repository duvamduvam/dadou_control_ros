from dadoucontrol.gui.windows.sequences.abstract_sequence_frame import AbstractSequenceFrame


class NeckFrame(AbstractSequenceFrame):
    def __init__(self, parent, color):
        self.FRAME_NAME = 'neck'
        super().__init__(parent, self.FRAME_NAME, color)
        self.lastX = 0

        self.canvas.bind("<Button-1>", self.create_circle)

    def create_circle(self, e):
        circle = self.canvas.create_oval(e.x, e.y, e.x+10, e.y+10, width=3, fill="#476042")
        self.canvas.tag_bind(circle, '<Button-3>', self.delete)

    def delete(self, e):
        circle = self.canvas.find_closest(e.x, e.y)
        self.canvas.delete(circle)

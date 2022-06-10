from dadoucontrol.gui.windows.sequences.abstract_sequence_frame import AbstractSequenceFrame


class WheelsFrame(AbstractSequenceFrame):
    def __init__(self, parent, color):
        self.FRAME_NAME = 'wheels'
        super().__init__(parent, self.FRAME_NAME, color)

        self.UP_LINE_Y = 40
        self.DOWN_LINE_Y = 90

        left = self.canvas.create_line(15, self.UP_LINE_Y , self.winfo_width()-15, self.UP_LINE_Y , width=3)
        right = self.canvas.create_line(15, self.DOWN_LINE_Y, self.winfo_width()-15, self.DOWN_LINE_Y, width=3)

        self.canvas.bind("<Button-1>", self.create_wheels_point)

    def create_wheels_point(self, e):
        print("create_wheels_point x: "+str(e.x)+ " y : "+str(e.y))

        y2 = self.UP_LINE_Y if e.y > (self.UP_LINE_Y+self.DOWN_LINE_Y)/2 else self.DOWN_LINE_Y
        self.create_point(e.x, e.y, e.x + 5, e.y + 5)
        self.create_point(e.x, y2, e.x + 5, y2 + 5)
        self.canvas.create_line(e.x+3, e.y, e.x+3, y2, dash=(5, 1))


    def create_point(self, x1, y1, x2, y2):
        self.canvas.create_oval(x1, y1, x2, y2, width=6, fill="#476042")

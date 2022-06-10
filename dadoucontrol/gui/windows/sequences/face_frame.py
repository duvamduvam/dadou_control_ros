from dadoucontrol.gui.windows.sequences.abstract_sequence_frame import AbstractSequenceFrame


class FaceFrame(AbstractSequenceFrame):
    def __init__(self, parent, name, color):
        super().__init__(parent, name, color)

        self.rectangle_bars = []
        self.lastX = 0

        self.canvas.bind("<Button-1>", self.create_rectangle)

    def create_rectangle(self, e):
        rectangle = self.canvas.create_rectangle(self.lastX, 40, e.x, 120, fill="pink")
        bar = self.canvas.create_line(e.x, 40, e.x, 120, width=5)
        self.canvas.tag_bind(rectangle, '<Button-3>', self.delete)
        self.rectangle_bars.append(RectangleBar(rectangle, bar, self.canvas))
        self.lastX = e.x

    def delete(self, e):
        rectangle = self.canvas.find_closest(e.x, e.y)
        rectangle_bar = self.find_rectangle(rectangle)
        self.canvas.delete(rectangle_bar.rectangle)
        self.canvas.delete(rectangle_bar.bar)
        self.rectangle_bars.remove(rectangle_bar)
        # rectangle = self.canvas.find_en
        # print("delete item")

    def find_rectangle(self, rectangle):
        for rectangle_bar in self.rectangle_bars:
            if rectangle_bar.compare(rectangle):
                return rectangle_bar
        print("no matching rectangle bar")


class RectangleBar:
    def __init__(self, rectangle, bar, canvas):
        self.rectangle = rectangle
        self.bar = bar
        self.canvas = canvas

    def compare(self, rectangle):
        # self.rectangle.
        coords_to_compare = self.canvas.coords(rectangle)
        coords = self.canvas.coords(self.rectangle)
        print(self.canvas.coords(rectangle))
        return coords[0] == coords_to_compare[0]

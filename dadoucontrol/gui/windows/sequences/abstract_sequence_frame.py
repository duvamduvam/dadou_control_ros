import tkinter as tk

from dadoucontrol.gui.windows.sequences.time_line_bar import TimeLineBar


class AbstractSequenceFrame(tk.Frame):
    time_line_bar_pos = 0
    def __init__(self, parent, name, color):
        tk.Frame.__init__(self, parent,  bg='orange')
        self.pack(fill='x', side='top')
        self.update()

        self.canvas = tk.Canvas(self, height=150, bg=color)
        self.set_frame_name(name)
        self.canvas.pack(fill='x')

        TimeLineBar(self.canvas)

    def set_frame_name(self, name):
        self.canvas.create_text(100, 20, text=name, fill="black", font=('Helvetica 15 bold'))

    def listen_time_frame(self):
        print(AbstractSequenceFrame.time_line_bar_pos)

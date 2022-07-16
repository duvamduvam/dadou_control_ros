from dadoucontrol.gui.windows.frames.abstract.abstract_sequence_frame import AbstractSequenceFrame
from dadoucontrol.gui.windows.frames.widgets.time_line_bar import TimeLineBar


class MusicFrame(AbstractSequenceFrame):

    def __init__(self, parent, color):
        self.FRAME_NAME = 'music'
        super().__init__(parent, self.FRAME_NAME, color)

        #self.line = self.canvas.create_line(15, 50, self.winfo_width()-15, 50, width=3)

    def load_audio(self, audio_display):
        width = self.canvas.winfo_width()
        max_height = int((self.winfo_height()/3)*4)
        self.canvas.delete('all')
        TimeLineBar(self.canvas)
        for i in range(width):
            height = int(max_height*audio_display[i])
            self.canvas.create_line(i, max_height, i, max_height-height, width=1)


        
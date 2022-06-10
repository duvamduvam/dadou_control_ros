class TimeLineBar:
    def __init__(self, canvas):
        self.canvas = canvas
        self.time_line = self.canvas.create_line(15, 0, 15, 100, width=5)
        self.canvas.tag_bind(self.time_line, '<Enter>', self.object_enter_event)
        self.canvas.tag_bind(self.time_line, '<B1-Motion>', self.move)
        self.canvas.tag_bind(self.time_line, '<Leave>', self.release_line)

    def release_line(self, event):
        self.canvas.itemconfigure(self.time_line, width=5)

    def changeObject(self, e):
        e.widget.configure(bg = 'red')

    def move(self, event):
        item_x = self.canvas.coords(self.time_line)[0]
        pos_x = event.x
        self.canvas.move(self.time_line, pos_x - item_x, 0)  # <--- Use Canvas.move method.
        #print('{}, {}'.format(pos_x, pos_y))
        print("Coordinates of the object are:", self.canvas.coords(self.time_line), " state : " + str(event.char))

    def object_enter_event(self, event):
        self.canvas.itemconfigure(self.time_line, width=10)
        print('Enter event:', event.x, event.y)

    def play(self):
        self.parent.after(200, self.play)
        pos = 300 * self.canvas.parent.current_pos
        self.canvas.move(self.time_line, pos, 0)

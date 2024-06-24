import logging
import tkinter as tk
from PIL import ImageTk
from pynput import keyboard

from controller.gui.pillow.ImageGenerator import ImageGenerator
from controller.gui.pillow.KeyboardListener import KeyboardListener
from dadou_utils_ros.utils_static import NAME, DEFAULT, COLOR, POSITION, BRIGHTNESS, RANDOM, NECK, SPEED, RESTART, TYPE, \
    BUTTON, SLIDE

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240


class PillowGuiApp:
    def __init__(self, node):
        self.root = tk.Tk()
        self.node = node
        self.root.title("Fenêtre avec Image Mise à Jour")

        self.canvas = tk.Canvas(self.root, width=SCREEN_WIDTH, height=SCREEN_HEIGHT)
        self.canvas.pack()

        self.image_generator = ImageGenerator()
        self.keyboard_listener = KeyboardListener(self.image_generator)

        self.process()
        self.root.mainloop()

    def process(self):
        try:
            key = self.keyboard_listener.get_key()
            if key:
                self.image_generator.set_last_key(key)

            image = self.image_generator.process()
            photo = ImageTk.PhotoImage(image)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=photo)
            self.canvas.image = photo
            self.root.after(100, self.process)
        except Exception as e:
            logging.error(e, exc_info=True)



if __name__ == "__main__":
    app = PillowGuiApp()

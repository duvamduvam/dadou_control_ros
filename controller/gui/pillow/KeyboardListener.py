from pynput import keyboard


class KeyboardListener:
    def __init__(self, image_generator):
        self.image_generator = image_generator
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        try:
            self.image_generator.set_last_key(key.char)
        except AttributeError:
            self.image_generator.set_last_key(str(key))
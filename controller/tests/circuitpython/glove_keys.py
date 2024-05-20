import adafruit_matrixkeypad
import board
import digitalio
import time


class GloveKeys:
    lastKeyTime = 0
    timeout = 250

    def __init__(self, keys, cols, rows):

        #self.keys = (("c", "b", "a"), ("f", "e", "d"), ("i", "h", "g"), ("l ", "k", "j"))
        self.keys = keys
        #cols = [digitalio.DigitalInOut(x) for x in (board.GP7, board.GP8, board.GP9)]
        #rows = [digitalio.DigitalInOut(x) for x in (board.GP10, board.GP11, board.GP12, board.GP13)]

        self.keypad = adafruit_matrixkeypad.Matrix_Keypad(
            [digitalio.DigitalInOut(x) for x in rows],
            [digitalio.DigitalInOut(x) for x in cols], keys)

    def get_time(self):
        return time.monotonic()*1000

    def check(self):
        keys = self.keypad.pressed_keys
        #print("current time {} last time {}".format(self.get_time(), self.lastKeyTime))
        if keys and (self.get_time() > (self.lastKeyTime + self.timeout)):
            self.lastKeyTime = self.get_time()
            #print("Pressed: ", keys[0], "time ", time.monotonic(), "lastKeyTime ", self.lastKeyTime)
            #print("Pressed: ", keys[0])
            return keys
        return []
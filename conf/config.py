import logging

from control.json_manager import JsonManager


class Config:

    BASE_PATH = "/home/pi/deploy/"

    def __init__(self, json_manager: JsonManager):
        self.json_config = json_manager.get_config()
        self.load()

    def load(self):
        self.load_pins()
        logging.debug(self.__dict__)

    def reload(self):
        self.json_config = JsonManager().get_config()
        self.load()




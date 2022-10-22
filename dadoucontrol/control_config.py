import logging
import logging.config
import os
from os.path import isfile

from dadoucontrol.control_static import ControlStatic

class ControlConfig:

    def __init__(self, control_json_manager, base_path):
        self.base_path = base_path
        self.control_json_manager = control_json_manager
        self.json_config = self.control_json_manager.get_config
        self.load()

    def load(self):
        logging.debug(self.__dict__)

    def reload(self):
        self.json_config = self.control_json_manager.get_config
        self.load()

    def get_folder(self, path):
        return self.base_path+path

    def get_folder_type(self, folder_type):
        return self.json_config.get_folder_typ

    def get_logging_conf_file(self):
        if isfile(ControlStatic.LOGGING_CONFIG_FILE):
            print('config file accessible')
            return ControlStatic.LOGGING_CONFIG_FILE
        else:
            print('config file not accessible')

    def get_path(self, name):
        self.json
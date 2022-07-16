import os
from os import listdir
from os.path import isfile, join

from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.control_static import ControlStatic


class FileManager:

    SEQUENCE_FOLDER = "frames"

    @staticmethod
    def list_folder_files(folder_type):
        folder = ControlFactory().control_json_manager.get_folder_path_from_type(folder_type)
        return [f for f in listdir(folder) if isfile(join(folder, f))]


    """@staticmethod
    def get_files_path(path):
        files = []
        for f in listdir(path):
            if isfile(join(path, f)):
                files.append(f)
        return files

    def sequences_files(self):
        return self.get_files_path(self.BASE_FOLDER+ControlStatic.JSON_DIRECTORY+self.SEQUENCE_FOLDER)
"""
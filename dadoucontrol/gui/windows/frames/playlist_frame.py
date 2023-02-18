import logging
import tkinter as tk
from tkinter import BOTH, TOP, filedialog, LEFT, X, Y, RIGHT

from dadou_utils.files.files_utils import FilesUtils
from dadou_utils.misc import Misc
from dadou_utils.utils_static import NAME

from dadoucontrol.control_factory import ControlFactory

from dadoucontrol.files.file_manager import FileManager

from dadoucontrol.control_static import BASE_PATH, PLAYLIST_PATH, BORDEAUX, AUDIO_DIRECTORY

from control_static import CYAN


class PlaylistFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):

        self.control_json = ControlFactory().control_json_manager

        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.pack(fill=BOTH, expand=True, side=TOP)
        self.main = tk.Frame(self, width=600, bg=BORDEAUX)
        self.main.pack(fill=X, expand=True, side=RIGHT)

        left_menu  = tk.Frame(self, width=50, bg=CYAN)
        left_menu.pack(fill=X, ipadx=20, side=LEFT)

        self.files = tk.StringVar()
        self.files.set(FilesUtils.get_folder_files_name(BASE_PATH +PLAYLIST_PATH))
        self.files_listbox = tk.Listbox(left_menu, listvariable=self.files, height=16)
        self.files_listbox.bind('<<ListboxSelect>>', self.click_file)
        self.files_listbox.pack(fill=X, ipadx=20, side=LEFT)

        self.playlist_data = None

        self.playlist_var = tk.StringVar()
        self.files.set(FilesUtils.get_folder_files_name(BASE_PATH +PLAYLIST_PATH))
        self.playlist_listbox = tk.Listbox(self.main, listvariable=self.playlist_var, height=16)
        self.playlist_listbox.grid(row=0, column=0, rowspan=2, sticky='new')

        self.add_button = tk.Button(self.main, text='add', command=self.click_add)
        self.add_button.grid(row=0, column=1, sticky='new')

    def click_add(self):
        filepath = filedialog.askopenfilename(initialdir=BASE_PATH +AUDIO_DIRECTORY,title="Open a Text File",
                                              filetypes=(("text    files", "*"), ("all files", "*.*")))
        self.files.append(filepath)

    def click_file(self, evt):
        w = evt.widget
        if len(w.curselection()):
            index = int(w.curselection()[0])
            value = w.get(index)
            #self.playlist_var.set(value)
            logging.info('You selected item %d: "%s"' % (index, value))

            self.playlist = value
            self.get_playlist(self.playlist)

    def get_playlist(self, file):
        items = []
        self.playlist_data = self.control_json.open_json(PLAYLIST_PATH + file, 'r')
        for item in self.playlist_data:
            items.append(item[NAME])

        self.playlist_var.set(items)

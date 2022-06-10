import logging
import tkinter as tk

from dadoucontrol.control_factory import ControlFactory
from dadoucontrol.files.file_manager import FileManager
from dadoucontrol.gui.windows.sequences.audio_select_popup import AudioPopupWidget
from dadoucontrol.gui.windows.sequences.music_frame import MusicFrame


class SequencesManagerWidget(tk.Frame):
    def __init__(self, parent, music_frame: MusicFrame):
        self.music_frame = music_frame
        #TODO improve 'sequences' parameters
        self.sequence_manager = ControlFactory().sequence_management
        #self.sequence_manager.set_music_frame(music_frame)
        tk.Frame.__init__(self, parent)
        files = FileManager.list_folder_files('sequences')
        files_var = tk.StringVar(self)
        files_var.set(files)

        self.audio_name_var = tk.StringVar(self)
        files_var.set(files)
        self.audio_name = tk.Label(self, textvariable=self.audio_name_var, wraplength=150)
        self.audio_name.grid(row=0, column=0, sticky='ns')
        self.audio_duration = tk.Label(self)
        self.audio_duration.grid(row=0, column=1, )
        tk.Label(self, text='current sequence : ').grid(row=1, column=0, )
        self.selected_sequence = tk.Label(self, text=files[0])
        self.selected_sequence.grid(row=1, column=1)

        audio_dialog = tk.Button(self, text="Ajouter une sequence", command=lambda: AudioPopupWidget(parent))
        audio_dialog.grid(row=2, column=0)

        select_audios = tk.Listbox(self, listvariable=files_var, height=6)
        select_audios.bind('<<ListboxSelect>>', self.select_sequence)
        select_audios.grid(row=2, column=1)

        self.pack(fill='x', side='top')
        self.update_widget(files[0])

    def select_sequence(self, evt):
        w = evt.widget
        index = int(w.curselection()[0])
        value = w.get(index)
        self.selected_sequence.config(text=value)
        self.update_widget(value)
        logging.info('You selected item %d: "%s"' % (index, value))

    def update_widget(self, sequence_name):
        self.sequence_manager.load_sequence(sequence_name)
        audio_name = self.sequence_manager.audio_segment.audio_name
        self.audio_name_var.set(audio_name)
        #self.audio_name.config(text=audio_name)
        audio_duration = self.sequence_manager.audio_segment.get_duration()
        self.audio_duration.config(text=audio_duration)

        self.music_frame.update()

        display_data = self.sequence_manager.audio_segment.get_audio_data_display(self.music_frame.winfo_width())
        self.music_frame.load_audio(display_data)
        #self.sequence_manager.get_audio_name()


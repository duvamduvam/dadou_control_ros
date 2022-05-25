import IPython.display as ipd
import librosa
import librosa.display
import matplotlib.pyplot as plt

music = '/home/dadou/Nextcloud/Didier/python/didier-python/audios/gig.wav'
data, sampling_rate = librosa.load(music)
plt.figure(figsize=(12, 4))
librosa.display.waveshow(data, sr=sampling_rate)
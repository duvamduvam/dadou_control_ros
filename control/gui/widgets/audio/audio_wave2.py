import matplotlib.pyplot as plt
import numpy as np
import wave, sys


def visualize(path: str):
    raw = wave.open(path)
    signal = raw.readframes(-1)
    signal = np.frombuffer(signal, dtype="int16")
    f_rate = raw.getframerate()
    time = np.linspace(
        0,
        len(signal) / f_rate,
        num=len(signal)
    )

    plt.figure(1)
    plt.title("Sound Wave")
    plt.xlabel("Time")
    plt.plot(time, signal)
    plt.show()


if __name__ == "__main__":
    music = '/home/dadou/Nextcloud/Didier/python/didier-python/audios/gig.wav'
    visualize(music)
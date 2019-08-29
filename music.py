import simpleaudio as sa
import threading


def playing(file):
    wave_obj = sa.WaveObject.from_wave_file(file)
    play_obj = wave_obj.play()
    play_obj.wait_done()

def play(file):
    t = threading.Thread(target = playing, args = (file,))
    t.start()


if __name__ == "__main__":
    file = 'music/rap.wav'
    play(file)
import simpleaudio as sa

def play(file):
    wave_obj = sa.WaveObject.from_wave_file(file)
    play_obj = wave_obj.play()
    play_obj.wait_done()

if __name__ == "__main__":
    file = 'music/rap.wav'
    play(file)
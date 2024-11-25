import sounddevice as sd
import soundfile as sf

data, samplerate = sf.read("c.mp3")
sd.play(data, samplerate)
sd.wait()
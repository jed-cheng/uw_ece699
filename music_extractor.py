import essentia.streaming as ess
import essentia.standard as esd
import essentia
import time
from mutagen import File
import numpy as np


FRAME_SIZE = 44100  # Assuming a sample rate of 44.1 kHz
HOP_SIZE = FRAME_SIZE  # Non-overlapping frames for 1-second segments


def get_audio_chords(file_path):
  # Initialize algorithms we will use.
  loader = ess.MonoLoader(filename=file_path)
  framecutter = ess.FrameCutter(frameSize=FRAME_SIZE, hopSize=HOP_SIZE, silentFrames='noise')
  windowing = ess.Windowing(type='blackmanharris62')
  spectrum = ess.Spectrum()
  spectralpeaks = ess.SpectralPeaks(orderBy='magnitude',
                                    magnitudeThreshold=0.00001,
                                    minFrequency=20,
                                    maxFrequency=3500,
                                    maxPeaks=60)
  hpcp = ess.HPCP()

  # Use pool to store data.
  pool = essentia.Pool()

  # Connect streaming algorithms.
  loader.audio >> framecutter.signal
  framecutter.frame >> windowing.frame >> spectrum.frame
  spectrum.spectrum >> spectralpeaks.spectrum
  spectralpeaks.magnitudes >> hpcp.magnitudes
  spectralpeaks.frequencies >> hpcp.frequencies
  hpcp.hpcp >> (pool, 'tonal.hpcp')

  # Run streaming network.
  essentia.run(loader)

  chords, strength = esd.ChordsDetection(windowSize=2)(pool['tonal.hpcp'])
  return chords

def get_audio_duration(file_path):
  audio = File(file_path)
  if audio is not None and audio.info is not None:
    return audio.info.length
  return None


def get_audio_tempo(file_path):
    loader = esd.MonoLoader(filename=file_path)
    audio = loader()

    sample_rate = 44100  # Assuming a sample rate of 44.1 kHz
    window_size = sample_rate  # 1 second window
    hop_size = window_size     # Non-overlapping windows

    rhythm_extractor = esd.RhythmExtractor2013(method="multifeature")

    tempos = []

    for i in range(0, len(audio), hop_size):
        segment = audio[i:i + window_size]
        
        if len(segment) < window_size:
            break
        
        bpm, _, _, _, _ = rhythm_extractor(segment)
        tempos.append(bpm)

    return tempos


if __name__ == '__main__':
  audio_file = 'c.mp3'
  tempo = get_audio_tempo(audio_file)
  print(len(tempo))
  chords = get_audio_chords(audio_file)
  print(chords)
  print(len(chords))
  
  duration = get_audio_duration(audio_file)
  print(duration)

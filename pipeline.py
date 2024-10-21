from enum import Enum
# a pipeline that receives audio data and outputs music features


class MusicPipeline:
  def __init__(self):
    self.audio_data = None
    self.music_features = None

  def receive_audio_data(self, audio_data):
    self.audio_data = audio_data

  def extract_chords(self):
    pass

  def get_music_features(self):
    pass


# a pipeline that receives music features and outputs emotions
class Emotion(Enum):
  EXCITED = 1
  HAPPY = 2
  PLEASESD = 3
  RELAXED = 4
  PEACEFUL = 5
  CALM = 6
  SAD = 7
  BORED = 8
  SLEEPY = 9
  ANGRY = 10
  NERVOUS = 11
  ANNOYING = 12


class EmotionPipeline:
  def __init__(self):
    self.music_features = None
    self.emotions = None

  def receive_music_features(self, music_features):
    self.music_features = music_features

  def predict_emotions(self):
    pass

  def get_emotions(self):
    pass

# color with hex values
class Color(Enum):
  YELLOW = '#FFFF00'
  GREEN = '#008000'
  BLUE = '#0000FF'
  GRAY = '#808080'
  PURPLE = '#800080'
  ORANGE = '#FFA500'
  RED = '#FF0000'
  BLACK = '#000000'
  LIGHT_BLUE = '#ADD8E6'
  LIGHT_GREEN = '#90EE90'
  YELLOW_RED = '#FFD700'


# a pipeline that receives emotions and outputs colors
class ColorPipline:
  def __init__(self):
    self.emotions = None
    self.colors = None

  def receive_emotions(self, emotions):
    self.emotions = emotions

  def predict_colors(self):
    pass

  def get_colors(self):
    pass


# a pipeline that receives emotion and outputs shapes
class ShapePipeline:
  def __init__(self):
    self.emotions = None
    self.shapes = None

  def receive_emotions(self, emotions):
    self.emotions = emotions

  def predict_shapes(self):
    pass

  def get_shapes(self):
    pass

# a pipeline that receives emotions and outputs centers coordinates
class CenterPipeline:
  def __init__(self):
    self.emotions = None
    self.centers = None

  def receive_emotions(self, emotions):
    self.emotions = emotions

  def predict_centers(self):
    pass

  def get_centers(self):
    pass


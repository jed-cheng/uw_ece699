from utils import Color, Emotion, Chord
import numpy as np

# chord to emotion
class EmotionPipeline:
  def __init__(self):
    self.emotions = None

    self.symbol_to_chord = {
      'A': Chord.MAJOR,
      'Am': Chord.MINOR,
      'A#' : Chord.MINOR,
      'A#m': Chord.MINOR,
      'Ab': Chord.MINOR,
      'B': Chord.MINOR,
      'Bm': Chord.MINOR,
      'Bb': Chord.MINOR,
      'C': Chord.MAJOR,
      'Cm': Chord.MINOR,
      'C#': Chord.MINOR,
      'C#m': Chord.MINOR,
      'D': Chord.MAJOR,
      'Dm': Chord.MINOR,
      'D#': Chord.MINOR,
      'D#m': Chord.MINOR,
      'E': Chord.MAJOR,
      'Em': Chord.MINOR,
      'Eb': Chord.MINOR,
      'F': Chord.MAJOR,
      'Fm': Chord.MINOR,
      'F#': Chord.MINOR,
      'F#m': Chord.MINOR,
      'G': Chord.MAJOR,
      'Gm': Chord.MINOR,
      'G#' : Chord.MINOR,
      'G#m': Chord.MINOR,
    }
    self.chord_to_emotion = {
      Chord.MAJOR: (Emotion.SERENITY, Emotion.ACCEPTANCE, Emotion.TRUST),
      Chord.MINOR: (Emotion.GRIEF, Emotion.SADNESS, Emotion.ANGER, Emotion.PENSIVENESS)
    }
  
  def receive_chord(self, chord):
    self.chords = chord

  def predict_emotions(self, chord):
    # flatten the chord to emotion mapping
    emotions = self.chord_to_emotion[self.symbol_to_chord[chord]]
    self.emotions = emotions
    return emotions
  
  def get_emotions(self):
    return self.emotions



class LocationPipeline:
  def __init__(self):
    self.locations = None
    self.emotions = None

    eight_divide_1 = 2.121320343559643
    eight_divide_2 = eight_divide_1 * 2
    eight_divide_3 = eight_divide_1 * 3
    self.emotion_to_location = {

      Emotion.TERROR: [3, 0],
      Emotion.FEAR: [6, 0],
      Emotion.APPREHENSION: [9, 0],

      Emotion.ECSTASY: [0, 3],
      Emotion.JOY: [0, 6],
      Emotion.SERENITY: [0, 9],

      Emotion.RAGE: [-3, 0],
      Emotion.ANGER: [-2, 0],
      Emotion.ANNOYANCE: [-1, 2],


      Emotion.GRIEF: [0, -3],
      Emotion.SADNESS: [0, -6],
      Emotion.PENSIVENESS: [0, -9],

      Emotion.AMAZEMENT: [eight_divide_1, -eight_divide_1],
      Emotion.SURPRISE: [eight_divide_2, -eight_divide_2],
      Emotion.DISTRACTION: [eight_divide_3, -eight_divide_3],
      
      Emotion.LOATHING: [-eight_divide_1, -eight_divide_1],
      Emotion.DISGUST: [-eight_divide_2, -eight_divide_2],
      Emotion.BOREDOM: [-eight_divide_3, -eight_divide_3],

      Emotion.ADMIRATION: [eight_divide_1, eight_divide_1],
      Emotion.TRUST: [eight_divide_2, eight_divide_2],
      Emotion.ACCEPTANCE: [eight_divide_3, eight_divide_3],

      Emotion.VIGILANCE: [-eight_divide_1, eight_divide_1],
      Emotion.ANTICIPATION: [-eight_divide_2, eight_divide_2],
      Emotion.INTEREST: [-eight_divide_3, eight_divide_3],
    }

  def receive_emotions(self, emotions):
    self.emotions = emotions

  def predict_locations(self):
    locations = [self.emotion_to_location[emotion] for emotion in self.emotions]
    self.locations = locations
    return locations



  def get_locations(self):
    return self.locations



# a pipeline that receives emotions and outputs colors
class ColorPipeline:
  def __init__(self):
    self.emotions = None
    self.colors = None
    self.emotion_to_color = {
      Emotion.TERROR: Color.TERROR_COLOR,
      Emotion.FEAR: Color.FEAR_COLOR,
      Emotion.APPREHENSION: Color.APPREHENSION_COLOR,
      
      Emotion.ECSTASY: Color.ECSTASY_COLOR,
      Emotion.JOY: Color.ECSTASY_COLOR,
      Emotion.SERENITY: Color.SERENITY_COLOR,
      
      Emotion.RAGE: Color.RAGE_COLOR,
      Emotion.ANGER: Color.ANGER_COLOR,
      Emotion.ANNOYANCE: Color.ANGER_COLOR,
      
      Emotion.GRIEF: Color.GRIEF_COLOR,
      Emotion.SADNESS: Color.GRIEF_COLOR,
      Emotion.PENSIVENESS: Color.PENSIVENESS_COLOR,
      
      Emotion.AMAZEMENT: Color.AMAZEMENT_COLOR,
      Emotion.SURPRISE: Color.TERROR_COLOR,
      Emotion.DISTRACTION: Color.DISTRACTION_COLOR,
      
      Emotion.LOATHING: Color.LOATHING_COLOR,
      Emotion.DISGUST: Color.SERENITY_COLOR,
      Emotion.BOREDOM: Color.BOREDOM_COLOR,
      
      Emotion.ADMIRATION: Color.TERROR_COLOR,
      Emotion.TRUST: Color.TERROR_COLOR,
      Emotion.ACCEPTANCE: Color.TERROR_COLOR,
      
      Emotion.VIGILANCE: Color.VIGILANCE_COLOR,
      Emotion.ANTICIPATION: Color.SERENITY_COLOR,
      Emotion.INTEREST: Color.INTEREST_COLOR
    }

  def receive_emotions(self, emotions):
    self.emotions = emotions

  def predict_colors(self):
    colors = [self.emotion_to_color[emotion] for emotion in self.emotions]
    self.colors = colors
    return colors

  def get_colors(self):
    return self.colors


class LPipeline:
  def __init__(self):
    self.emotions = None
    self.colors = None
    self.locations = None

    self.color_pipeline = ColorPipeline()
    self.location_pipeline = LocationPipeline()

  def receive_emotions(self, emotions):
    self.emotions = emotions
    self.color_pipeline.receive_emotions(emotions)
    self.location_pipeline.receive_emotions(emotions)

  def predict_colors(self):
    return self.color_pipeline.predict_colors()

  def predict_locations(self):
    return self.location_pipeline.predict_locations()

  def get_colors(self):
    return self.color_pipeline.get_colors()

  def get_locations(self):
    return self.location_pipeline.get_locations()
  

class FormationPipeline:
  def __init__(self):
    self.emotions = None
    self.colors = None
    self.locations = None

    self.color_pipeline = ColorPipeline()
    self.location_pipeline = LocationPipeline()

  def receive_emotions(self, emotions):
    self.emotions = emotions
    self.color_pipeline.receive_emotions(emotions)
    self.location_pipeline.receive_emotions(emotions)

  def predict_colors(self):
    return self.color_pipeline.predict_colors()

  def predict_locations(self):
    return self.location_pipeline.predict_locations()

  def get_colors(self):
    return self.color_pipeline.get_colors()

  def get_locations(self):
    return self.location_pipeline.get_locations()
  

class ShapePipeline:
  def __init__(self):
    self.emotions = None
    self.colors = None
    self.locations = None

    self.color_pipeline = ColorPipeline()
    self.location_pipeline = LocationPipeline()

  def receive_emotions(self, emotions):
    self.emotions = emotions
    self.color_pipeline.receive_emotions(emotions)
    self.location_pipeline.receive_emotions(emotions)

  def predict_colors(self):
    return self.color_pipeline.predict_colors()

  def predict_locations(self):
    return self.location_pipeline.predict_locations()

  def get_colors(self):
    return self.color_pipeline.get_colors()

  def get_locations(self):
    return self.location_pipeline.get_locations()
  



if __name__ == "__main__":
  pipeline = ColorPipeline()
  emotions = [Emotion.ACCEPTANCE, Emotion.ANGER, Emotion.ANTICIPATION, Emotion.ADMIRATION]
  pipeline.receive_emotions(emotions)
  colors = pipeline.predict_colors()
  print(colors) 

  localtion_pipeline = LocationPipeline()
  localtion_pipeline.receive_emotions(emotions)
  locations = localtion_pipeline.predict_locations()
  print(locations)
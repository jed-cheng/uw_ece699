from utils import Color, Emotion, Chord
import numpy as np


def  cart2pol(x, y):
  rho = np.sqrt(x**2 + y**2)
  phi = np.arctan2(y, x)
  return(rho, phi)

def pol2cart(phi, rho):
  x = rho * np.cos(phi)
  y = rho * np.sin(phi)
  return(x, y)

# chord to emotion
class EmotionPipeline:
  def __init__(self):
    self.emotions = None

    self.symbol_to_chord = {
    'A': Chord.MAJOR,
    'Am': Chord.MINOR,
    'A#': Chord.MAJOR,
    'A#m': Chord.MINOR,
    'Ab': Chord.MAJOR,
    
    'B': Chord.MAJOR,
    'Bm': Chord.MINOR,
    'Bb': Chord.MAJOR,
    
    'C': Chord.MAJOR,
    'Cm': Chord.MINOR,
    'C#': Chord.MAJOR,
    'C#m': Chord.MINOR,
    
    'D': Chord.MAJOR,
    'Dm': Chord.MINOR,
    'D#': Chord.MAJOR,
    'D#m': Chord.MINOR,
    
    'E': Chord.MAJOR,
    'Em': Chord.MINOR,
    'Eb': Chord.MAJOR,
    
    'F': Chord.MAJOR,
    'Fm': Chord.MINOR,
    'F#': Chord.MAJOR,
    'F#m': Chord.MINOR,
    
    'G': Chord.MAJOR,
    'Gm': Chord.MINOR,
    'G#': Chord.MAJOR,
    'G#m': Chord.MINOR
    }
    self.chord_to_emotion = {
      Chord.MAJOR: (Emotion.SERENITY, Emotion.ACCEPTANCE, Emotion.TRUST),
      Chord.MINOR: (Emotion.GRIEF, Emotion.SADNESS, Emotion.ANGER, Emotion.PENSIVENESS)
    }
  
  def receive_chord(self, chord):
    self.chords = chord

  def predict_emotion(self, chord):
    # flatten the chord to emotion mapping
    emotions = self.chord_to_emotion[self.symbol_to_chord[chord]]
    self.emotions = emotions
    return emotions
  
  def get_emotions(self):
    return self.emotions



class CenterPipeline:
  def __init__(self):
    self.locations = None
    self.emotions = None
    self.R = 3
    # eight_divide_1 = 2.121320343559643
    # eight_divide_2 = eight_divide_1 * 2
    # eight_divide_3 = eight_divide_1 * 3
    # self.emotion_to_location = {

    #   Emotion.TERROR: [3, 0],
    #   Emotion.FEAR: [6, 0],
    #   Emotion.APPREHENSION: [9, 0],

    #   Emotion.ECSTASY: [0, 3],
    #   Emotion.JOY: [0, 6],
    #   Emotion.SERENITY: [0, 9],

    #   Emotion.RAGE: [-3, 0],
    #   Emotion.ANGER: [-2, 0],
    #   Emotion.ANNOYANCE: [-1, 2],


    #   Emotion.GRIEF: [0, -3],
    #   Emotion.SADNESS: [0, -6],
    #   Emotion.PENSIVENESS: [0, -9],

    #   Emotion.AMAZEMENT: [eight_divide_1, -eight_divide_1],
    #   Emotion.SURPRISE: [eight_divide_2, -eight_divide_2],
    #   Emotion.DISTRACTION: [eight_divide_3, -eight_divide_3],
      
    #   Emotion.LOATHING: [-eight_divide_1, -eight_divide_1],
    #   Emotion.DISGUST: [-eight_divide_2, -eight_divide_2],
    #   Emotion.BOREDOM: [-eight_divide_3, -eight_divide_3],

    #   Emotion.ADMIRATION: [eight_divide_1, eight_divide_1],
    #   Emotion.TRUST: [eight_divide_2, eight_divide_2],
    #   Emotion.ACCEPTANCE: [eight_divide_3, eight_divide_3],

    #   Emotion.VIGILANCE: [-eight_divide_1, eight_divide_1],
    #   Emotion.ANTICIPATION: [-eight_divide_2, eight_divide_2],
    #   Emotion.INTEREST: [-eight_divide_3, eight_divide_3],
    # }

    self.chord_to_center = {
    # Inner Circle (Major Chords)
    'C': pol2cart(0, self.R),
    'G': pol2cart(30, self.R),
    'D': pol2cart(60, self.R),
    'A': pol2cart(90, self.R),
    'E': pol2cart(120, self.R),
    'B': pol2cart(150, self.R),
    'F#': pol2cart(180, self.R),
    'Db': pol2cart(210, self.R),
    'Ab': pol2cart(240, self.R),
    'Eb': pol2cart(270, self.R),
    'Bb': pol2cart(300, self.R),
    'F': pol2cart(330, self.R),
    
    # Middle Circle (Minor Chords)
    'Am': pol2cart(0, 2*self.R),
    'Em': pol2cart(30, 2*self.R),
    'Bm': pol2cart(60, 2*self.R),
    'F#m': pol2cart(90, 2*self.R),
    'C#m': pol2cart(120, 2*self.R),
    'G#m': pol2cart(150, 2*self.R),
    'D#m': pol2cart(180, 2*self.R),
    'Bbm': pol2cart(210, 2*self.R),
    'Fm': pol2cart(240, 2*self.R),
    'Cm': pol2cart(270, 2*self.R),
    'Gm': pol2cart(300, 2*self.R),
    'Dm': pol2cart(330, 2*self.R),
    
    # Outer Circle (Diminished Chords and others) 
    'B°': pol2cart(0, 3*self.R),
    'F#°': pol2cart(30, 3*self.R),
    'C#°': pol2cart(60, 3*self.R),
    'G#°': pol2cart(90, 3*self.R),
    'D#°': pol2cart(120, 3*self.R),
    'A#°': pol2cart(150, 3*self.R),
    'E#°': pol2cart(180, 3*self.R),
    'B#°': pol2cart(210, 3*self.R),
    'F°': pol2cart(240, 3*self.R),
    'C°': pol2cart(270, 3*self.R),
    'G°': pol2cart(300, 3*self.R),
    'D°': pol2cart(330, 3*self.R)
    }

  def receive_emotions(self, emotions):
    self.emotions = emotions

  def predict_center(self, chord):
    return self.chord_to_center[chord]




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

  def predict_color(self, emotion):
    return self.emotion_to_color[emotion]
  
  def predict_colors(self, emotions):
    return [self.emotion_to_color[emotion] for emotion in emotions]


  def get_colors(self):
    return self.colors


class LPipeline:
  def __init__(self):
    self.emotions = None
    self.colors = None
    self.locations = None

    self.color_pipeline = ColorPipeline()
    self.location_pipeline = CenterPipeline()

  def receive_emotions(self, emotions):
    self.emotions = emotions
    self.color_pipeline.receive_emotions(emotions)
    self.location_pipeline.receive_emotions(emotions)

  def predict_colors(self):
    return self.color_pipeline.predict_color()

  def predict_locations(self):
    return self.location_pipeline.predict_center()

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
    self.location_pipeline = CenterPipeline()

  def receive_emotions(self, emotions):
    self.emotions = emotions
    self.color_pipeline.receive_emotions(emotions)
    self.location_pipeline.receive_emotions(emotions)

  def predict_colors(self):
    return self.color_pipeline.predict_color()

  def predict_locations(self):
    return self.location_pipeline.predict_center()

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
    self.location_pipeline = CenterPipeline()

  def receive_emotions(self, emotions):
    self.emotions = emotions
    self.color_pipeline.receive_emotions(emotions)
    self.location_pipeline.receive_emotions(emotions)

  def predict_colors(self):
    return self.color_pipeline.predict_color()

  def predict_locations(self):
    return self.location_pipeline.predict_center()

  def get_colors(self):
    return self.color_pipeline.get_colors()

  def get_locations(self):
    return self.location_pipeline.get_locations()
  



if __name__ == "__main__":
  chords = ['C', 'Am', 'G', 'Em']
  localtion_pipeline = CenterPipeline()
  emotion_pipline = EmotionPipeline()
  color_pipeline = ColorPipeline()
  for chord in chords:
    location = localtion_pipeline.predict_center(chord)
    emotions = emotion_pipline.predict_emotion(chord)
    colors = color_pipeline.predict_colors(emotions)
    print(chord,location, colors)

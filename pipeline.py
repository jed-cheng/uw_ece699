from utils import Color, Emotion


class LocationPipeline:
  def __init__(self):
    self.locations = None
    self.emotions = None
    # evenly map emotions to locations on the cicle with radius 5
    self.emotion_to_location = {
      Emotion.EXCITED: [5, 0],
      Emotion.HAPPY: [4, 3],
      Emotion.PLEASESD: [3, 4],
      Emotion.RELAXED: [0, 5],
      Emotion.PEACEFUL: [-3, 4],
      Emotion.CALM: [-4, 3],
      Emotion.SAD: [-5, 0],
      Emotion.BORED: [-4, -3],
      Emotion.SLEEPY: [-3, -4],
      Emotion.ANGRY: [0, -5],
      Emotion.NERVOUS: [3, -4],
      Emotion.ANNOYING: [4, -3],
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
      Emotion.EXCITED: Color.YELLOW,
      Emotion.HAPPY: Color.GREEN,
      Emotion.PLEASESD: Color.BLUE,
      Emotion.RELAXED: Color.GRAY,
      Emotion.PEACEFUL: Color.PURPLE,
      Emotion.CALM: Color.LIGHT_BLUE,
      Emotion.SAD: Color.BLUE,
      Emotion.BORED: Color.GRAY,
      Emotion.SLEEPY: Color.LIGHT_GREEN,
      Emotion.ANGRY: Color.RED,
      Emotion.NERVOUS: Color.ORANGE,
      Emotion.ANNOYING: Color.YELLOW_RED,
    }

  def receive_emotions(self, emotions):
    self.emotions = emotions

  def predict_colors(self):
    colors = [self.emotion_to_color[emotion] for emotion in self.emotions]
    self.colors = colors
    return colors

  def get_colors(self):
    return self.colors


if __name__ == "__main__":
  pipeline = ColorPipeline()
  emotions = [Emotion.EXCITED, Emotion.HAPPY, Emotion.PLEASESD]
  pipeline.receive_emotions(emotions)
  colors = pipeline.predict_colors()
  print(colors) # ['CYAN', 'MAGENTA', 'YELLOW']

  localtion_pipeline = LocationPipeline()
  emotions = [Emotion.EXCITED, Emotion.HAPPY, Emotion.PLEASESD]
  localtion_pipeline.receive_emotions(emotions)
  locations = localtion_pipeline.predict_locations()
  print(locations) # [[5, 0], [4, 3], [3, 4]]
from enum import Enum
import numpy as np


class DensityFunction:
  def __init__(self, type, color, center=None, variance=None, func=None):
    self.type = type   # 'uniform' or 'gaussian'
    self.color = color # hex cmy color value
    self.center = center
    self.variance = variance

    if self.center and self.variance:
      # gaussian density function mean at center and variance variance
      self.func =  lambda x, y: np.exp(
        -0.5*((x - self.center[0])**2/(self.variance[0]**2) + (y - self.center[1])**2/(self.variance[1]**2))
      )/ (2 * np.pi * self.variance[0] * self.variance[1]) if type == 'gaussian' else 1
    elif func:
      self.func = func
    else:
      raise ValueError('Invalid Density')

  def phi(self, x, y):
    return self.func(x, y)


class Color(Enum):
  CYAN = '#00FFFF'
  MAGENTA = '#FF00FF'
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
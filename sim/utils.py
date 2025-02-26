from enum import Enum
import numpy as np


class DensityFunction:
  def __init__(self,  color, center=None, variance=None, shape=None, func=None,type='gaussian', **kwargs ):
    self.type = type   # 'uniform' or 'gaussian'
    self.color = color # hex cmy color value
    self.center = center
    self.variance = variance
    self.shape = shape

    if func:
      self.func = func
      return

    if shape is None:
      if self.center and self.variance:
        # gaussian density function mean at center and variance variance
        self.func =  lambda x, y: np.exp(
          -0.5*((x - self.center[0])**2/(self.variance[0]**2) + (y - self.center[1])**2/(self.variance[1]**2))
        ) 

    elif shape == 'ellipse':
        if kwargs.get('k') and kwargs.get('a') and kwargs.get('b') and kwargs.get('r') and self.center:
          k = kwargs['k']
          a = kwargs['a']
          b = kwargs['b']
          r = kwargs['r']
          self.func =  lambda x, y: np.exp(
            -k * (a*(x - self.center[0])**2 + b*(y - self.center[1])**2 - r**2)
          )

    elif shape == 'line':
      if kwargs.get('k') and kwargs.get('a') and kwargs.get('b') and kwargs.get('c'):
        k = kwargs['k']
        a = kwargs['a']
        b = kwargs['b']
        c = kwargs['c']
        self.func =  lambda x, y: np.exp(-k * (a*x + b*y + c)**2)

    elif shape == 'disk':
      if kwargs.get('k') and kwargs.get('a') and kwargs.get('b') and kwargs.get('r') and kwargs.get('l') and  self.center:
        k = kwargs['k']
        a = kwargs['a']
        b = kwargs['b']
        r = kwargs['r']
        l = kwargs['l']

        self.func =  lambda x, y: np.exp(
          -k * (self.SR(a*(x - self.center[0]) + b*(y - self.center[1]) - r)**2, l)
        )


    else:
      raise ValueError('Invalid Density')

  def phi(self, x, y):
    return self.func(x, y)
  
  def SR(self, x, l):

    return x*(np.arctan(l*x)/np.pi + 0.5)


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

  TERROR_COLOR = "#00A551"
  FEAR_COLOR = "#31B475"
  APPREHENSION_COLOR = "#7AC698"

  ECSTASY_COLOR = "#FFCA02"
  JOY_COLOR = "#ea984a"
  SERENITY_COLOR = "#221f20"
  

  RAGE_COLOR = "#F05B61"
  ANGER_COLOR = "#F1736D"
  ANNOYANCE_COLOR = "#F48D80"

  GRIEF_COLOR = "#2982C5"
  SADNESS_COLOR = "#74A8DA"
  PENSIVENESS_COLOR = "#9FC0E5"

  AMAZEMENT_COLOR = "#0099CD"
  SURPRISE_COLOR = "#36AED7"
  DISTRACTION_COLOR = "#89C7E4"

  ADMIRATION_COLOR = "#8AC650"
  TRUST_COLOR = "#ACD16A"
  ACCEPTANCE_COLOR = "#CADF8B"

  VIGILANCE_COLOR = "#F6913D"
  ANTICIPATION_COLOR = "#F8AD66"
  INTEREST_COLOR = "#FCC487"

  LOATHING_COLOR = "#8973B3"
  DISGUST_COLOR = "#A390C4"
  BOREDOM_COLOR = "#B9AAD3"


class Emotion(Enum):
    TERROR = 1
    ECSTASY = 2
    RAGE = 3
    GRIEF = 4
    AMAZEMENT = 5
    JOY = 6
    TRUST = 7
    FEAR = 8
    ANTICIPATION = 9
    SADNESS = 10
    ANGER = 11
    DISGUST = 12
    SURPRISE = 13
    INTEREST = 14
    SERENITY = 15
    ACCEPTANCE = 16
    APPREHENSION = 17
    ANNOYANCE = 18
    LOATHING = 19
    BOREDOM = 20
    VIGILANCE = 21
    PENSIVENESS = 22
    DISTRACTION = 23
    ADMIRATION = 24


class Chord(Enum):
  MAJOR = 1,
  MINOR = 2,
  AUGMENTED = 3,
  DIMINISHED = 4,
  SEVENTH = 5,
  SUSPENDED = 6,
  EXTENDED = 7,
  ADDED = 8,

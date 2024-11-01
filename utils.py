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
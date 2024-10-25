from enum import Enum


class DensityFunction:
  def __init__(self, type, phi, color, center=None):
    self.type = type   # 'uniform' or 'gaussian'
    self.phi = phi     # lambda x, y: float
    self.color = color # hex cmy color value
    self.center = center


class Color(Enum):
  CYAN = '#00FFFF'
  MAGENTA = '#FF00FF'
  YELLOW = '#FFFF00'
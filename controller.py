import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider
from matplotlib.gridspec import GridSpec

class Controller:
  def __init__(self):
    fig = plt.figure()
    self.fig = fig

    gs = GridSpec(1, 2, width_ratios=[1, 1], wspace=0.3)

    axL = fig.add_subplot(gs[0, 0])
    self.L_slider  = Slider(
      ax=axL,
      label='L',
      valmin=0.1,
      valmax=1,
      valinit=1,
      orientation="vertical"
    )

    axtrail = fig.add_subplot(gs[0, 1])
    self.trail_width_slider = Slider(
      ax=axtrail,
      label='Trail Width',
      valmin=0.1,
      valmax=1,
      valinit=1,
      orientation="vertical"
    )

    self.L_slider.on_changed(self.update_L)
    self.trail_width_slider.on_changed(self.update_trail_width)

    plt.show()

  def update_L(self,val):
    self.fig.canvas.draw_idle()

  def update_trail_width(self, val):
    self.fig.canvas.draw_idle()


if __name__ == '__main__':
  controller = Controller()

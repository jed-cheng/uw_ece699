import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider
from matplotlib.gridspec import GridSpec

class Controller:
  def __init__(self):
    fig = plt.figure()
    self.fig = fig

    gs = GridSpec(1, 3, width_ratios=[1, 1, 5], wspace=0.3)


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

    self.ax_plot = fig.add_subplot(gs[0, 2])
    self.ax_plot.set_title('Click to get cursor position')
    self.ax_plot.set_xlim(0, 10)
    self.ax_plot.set_ylim(0, 10)


    self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
    self.fig.canvas.mpl_connect('button', self.on_plot_click)

    plt.show()


  def on_plot_click(self, event):
    if event.inaxes == self.ax_plot:
      print('Cursor Position:', event.xdata, event.ydata)

  def on_mouse_release(self, event):
    if event.inaxes == self.L_slider.ax or event.inaxes == self.trail_width_slider.ax:
      print('L:', self.L_slider.val)
      print('Trail Width:', self.trail_width_slider.val)


if __name__ == '__main__':
  controller = Controller()

import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.lines import Line2D
import numpy as np
from matplotlib.colors import to_rgba
import math
from consts import L_DEFAULT, L_MAX, L_MIN, TRAIL_WIDTH, TRAIL_WIDTH_MAX, TRAIL_WIDTH_MIN
from swarm import Swarm
from robot import Robot
import time
from pipeline import ColorPipeline, CenterPipeline
from matplotlib.widgets import  Slider
from utils import Color, DensityFunction, Emotion

class Simulator:
  def __init__(self, swarm, val_trail = TRAIL_WIDTH, val_L = L_DEFAULT):
    self.swarm = swarm
    self.environment = self.swarm.environment

    self.fig, self.ax_sim = plt.subplots()
    ax_l = self.fig.add_axes([0.25, 0.1, 0.65, 0.03])
    ax_trail_width = self.fig.add_axes([0.25, 0.15, 0.65, 0.03])
    self.l_slider = Slider(
      ax=ax_l,
      label='L',
      valmin=L_MIN,
      valmax=L_MAX,
      valinit=val_L,
      valstep=1
    )

    self.trail_width_slider = Slider(
      ax=ax_trail_width,
      label='Trail Width',
      valmin=TRAIL_WIDTH_MIN,
      valmax=TRAIL_WIDTH_MAX,
      valinit=val_trail,
      valstep=1
    )
    
    self.fig.subplots_adjust(bottom=0.25)
    self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
    self.fig.canvas.mpl_connect('button_press_event', self.on_sim_click)

    self.cursor_pos = None
    
    self.p_env = None
    self.p_density = []
    self.p_robots = None
    self.p_trails = [[] for _ in range(len(swarm.robots))]
    self.p_vor_centroid = []
    self.p_vor_cell = []



  def on_sim_click(self, event):
    if event.inaxes == self.ax_sim:
      self.cursor_pos = [event.xdata, event.ydata]

  def get_cursor_pos(self):
    pos =  self.cursor_pos
    self.cursor_pos = None
    return pos

  def on_mouse_release(self, event):
    if event.inaxes == self.l_slider.ax:
      for robot in self.swarm.robots:
        robot.set_L(self.l_slider.val)
    if event.inaxes == self.trail_width_slider.ax:
      for robot in self.swarm.robots:
        robot.set_trail_width(self.trail_width_slider.val)


  def plot_environment(self):
    if self.p_env:
      self.p_env.remove()


    self.p_env = patches.Polygon(self.environment, fill=False)
    self.ax_sim.add_patch(self.p_env)



  def plot_density_functions(self, 
    density_functions, 
    range=10, 
    resolution=100, 
    refresh=True,

  ):
    if refresh:
      for p in self.p_density:
        p.remove()
      self.p_density = []

    
    for density_function in density_functions:
      if density_function.shape is None:
        mux, muy = density_function.center
        varx, vary = density_function.variance
        xmin, xmax = mux - varx*5, mux + varx*5
        ymin, ymax  = muy - vary*5, muy + vary*5

        x = np.linspace(xmin, xmax, resolution)
        y = np.linspace(ymin, ymax, resolution)
        X, Y = np.meshgrid(x, y)

        Z = density_function.phi(X, Y)
        Z_norm = Z.reshape(X.shape)
        base_color = to_rgba(density_function.color)   

        rgba_image = np.zeros((Z.shape[0], Z.shape[1], 4))
        rgba_image[..., :3] = base_color[:3]
        alpha_exponent = 0.3
        rgba_image[...,  3] = Z_norm**alpha_exponent         
        p = self.ax_sim.imshow(
          rgba_image, 
          extent=[xmin, xmax, ymin, ymax],
          origin='lower',
          aspect='auto'
        )
        self.p_density.append(p)
      else:
        x = np.linspace(-range, range, resolution)
        y = np.linspace(-range, range, resolution)
        X, Y = np.meshgrid(x, y)

        Z = density_function.phi(X, Y)
        Z_norm = Z.reshape(X.shape)
        base_color = to_rgba(density_function.color)   

        rgba_image = np.zeros((Z.shape[0], Z.shape[1], 4))
        rgba_image[..., :3] = base_color[:3]
        alpha_exponent = 0.3
        rgba_image[...,  3] = Z_norm**alpha_exponent         
        p = self.ax_sim.imshow(
          rgba_image, 
          extent=[-range, range, -range, range],
          origin='lower',
          aspect='auto'
        )
        self.p_density.append(p)


  def plot_swarm(self):
    if self.p_robots:
      for p in self.p_robots:
        p.remove()


    self.p_robots = []
    for i, robot in enumerate(self.swarm.robots):
      pose = robot.robot_pose
      size = robot.robot_size
      R = np.array([[0.0, 1.0], [-1.0, 0.0]]) @ np.array([
        [math.cos(pose[2]), -math.sin(pose[2])],
        [math.sin(pose[2]), math.cos(pose[2])]
      ])
      t = np.array([pose[0], pose[1]])
      v = np.array([
        [0, size],
        [size * -math.sqrt(3)/2 , -size/2],
        [0,0],
        [size * math.sqrt(3)/2, -size/2]
      ])

      p_robot = patches.Polygon(t+v @ R.T, color=robot.robot_color, fill=True, zorder=3)
      self.p_robots.append(p_robot)
      self.ax_sim.add_patch(p_robot)

      if len(robot.trail) > 1:
        segment = robot.trail[-2:]
        p_segment = Line2D(segment[:,0], segment[:,1], color=robot.trail_color, linewidth=robot.trail_width)
        self.p_trails[i].append(p_segment)
        self.ax_sim.add_line(p_segment)

  def plot_voronoi(self, vor_centroid, vor_cell, 
    refresh=True,
    centroid_color='black',
    boundary_color='black',
    centroid_size=0.2
  ):

    if refresh:
      self.clear_voronoi()

    for centroid in vor_centroid:
      p = patches.Circle(centroid, radius=centroid_size, fill=True, color=centroid_color)
      self.p_vor_centroid.append(p)
      self.ax_sim.add_patch(p)

    for cell in vor_cell:
      p = patches.Polygon(cell, fill=False, closed=True, color=boundary_color)
      self.p_vor_cell.append(p)
      self.ax_sim.add_patch(p)

  def clear_voronoi(self):
    for p_vor_centroid in self.p_vor_centroid:
      p_vor_centroid.remove()
    for p_vor_cell in self.p_vor_cell:
      p_vor_cell.remove()
    self.p_vor_centroid = []
    self.p_vor_cell = []

  def plot(self):
    self.ax_sim.set_aspect('equal')
    self.ax_sim.set_xlim(-10, 10)  # Adjust the limits as needed
    self.ax_sim.set_ylim(-10, 10)  # Adjust the limits as needed
    plt.ion()
    plt.show()

  def update_plot(self):
    self.fig.canvas.draw_idle()
    self.fig.canvas.flush_events()


if __name__ == "__main__":
  robot_1 = Robot( 
    robot_pose=[0, -5, 0.0],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value],
    K=1
  )
  robot_2 = Robot( 
    robot_pose=[0, 5, 0.0],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value],
    K=1
  )
  robot_3 = Robot( 
    robot_pose=[-5, -6, 0.0],
    equiped_color=[Color.CYAN.value]
  )
  robot_4 = Robot( 
    robot_pose=[5, -5, 0.0],
    equiped_color=[Color.CYAN.value]
  )
  robot_5 = Robot(
    robot_pose=[0, 0, 0.0],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value]
  )
  robots = [robot_1, robot_2, robot_3, robot_4, robot_5]


  density_functions = [
    DensityFunction(
      color=Color.CYAN.value,
      center=[5, 0],
      variance=[1, 1]
    )
  ]

  env = np.array([
    [10, 10],
    [10, -10],
    [-10, -10],
    [-10, 10],
    [10, 10]
  ])

  swarm = Swarm(robots, env)
  sim = Simulator(swarm)

  color_pipe = ColorPipeline()
  location_pipe = CenterPipeline()
  
  # get all emotions of Emotion
  emotions = list(Emotion)


  sim.plot_environment(env)
  sim.plot_swarm(swarm)
  sim.plot()

  for i in range(1000):

    vor_robots, vor_prime = swarm.color_coverage_control(density_functions)


    for j in range(len(robots)):
      robot = robots[j]
      vor_robot = vor_robots[j]

      if vor_robot is None:
        continue
      
      vw = robot.coverage_control(vor_robot, delta=1)
      color = robot.coverage_control_color(vor_robot,
        swarm.cyan_density_functions,
        swarm.magenta_density_functions,
        swarm.yellow_density_functions
       ) 

      # print(j,vw, color)


    sim.plot_swarm(swarm)
    sim.clear_voronoi()
    for color, val in vor_prime.items():
      if val is None:
        continue
      centroid, cell, area = val
      sim.plot_voronoi(centroid, cell, refresh=False, centroid_color=color, boundary_color=color)

    # sim.plot_density_functions(density_functions)
    sim.update_plot()
    time.sleep(0.01)



  pass

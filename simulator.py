import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.lines import Line2D
import numpy as np
from matplotlib.colors import to_rgba
import math
from swarm import Swarm
from robot import Robot
from scipy.spatial import Voronoi
import time

from utils import Color, DensityFunction

class Simulator:
  def __init__(self, swarm, environment, **kwargs):
    self.swarm = swarm
    self.environment = environment

    self.figure, self.axes = plt.subplots()
    self.p_env = None
    self.p_density = []
    self.p_robots = None
    self.p_trails = None
    self.p_vor_centroid = []
    self.p_vor_cell = []



    # plt.ion()
    # plt.show(block=True)


  def plot_environment(self, environment):
    if self.p_env:
      self.p_env.remove()

    if environment is not None:
      self.environment = environment

    self.p_env = patches.Polygon(self.environment, fill=False)
    self.axes.add_patch(self.p_env)



  def plot_density_functions(self, 
    density_functions, 
    range=5, 
    resolution=100, 
    refresh=True,

  ):
    if refresh:
      for p in self.p_density:
        p.remove()
      self.p_density = []

    
    for density_function in density_functions:
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
      p = self.axes.imshow(
        rgba_image, 
        extent=[xmin, xmax, ymin, ymax],
        origin='lower',
        aspect='auto'
      )
      self.p_density.append(p)

  def plot_swarm(self, swarm):
    if self.p_robots:
      for p in self.p_robots:
        p.remove()

    if self.p_trails:
      for p in self.p_trails:
        p.remove()

    self.p_robots = []
    self.p_trails = []
    for robot in swarm.robots:
      pose = robot.robot_pose
      size = robot.robot_size
      trail = robot.trail
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

      p_robot = patches.Polygon(t+v @ R.T, color=robot.robot_color, fill=True)
      p_trail = Line2D(trail[:,0], trail[:,1], color=robot.trail_color, linewidth=robot.trail_width)

      self.p_robots.append(p_robot)
      self.p_trails.append(p_trail)

      self.axes.add_patch(p_robot)
      self.axes.add_line(p_trail)


  def plot_voronoi(self, vor_centroid, vor_cell, 
    refresh=True,
    centroid_color='black',
    boundary_color='black',
    centroid_size=0.2
  ):

    if refresh:
      self.refresh_voronoi()

    for centroid in vor_centroid:
      p = patches.Circle(centroid, radius=centroid_size, fill=True, color=centroid_color)
      self.p_vor_centroid.append(p)
      self.axes.add_patch(p)

    for cell in vor_cell:
      p = patches.Polygon(cell, fill=False, closed=True, color=boundary_color)
      self.p_vor_cell.append(p)
      self.axes.add_patch(p)

  def refresh_voronoi(self):
    for p_vor_centroid in self.p_vor_centroid:
      p_vor_centroid.remove()
    for p_vor_cell in self.p_vor_cell:
      p_vor_cell.remove()
    self.p_vor_centroid = []
    self.p_vor_cell = []

  def plot(self):
    self.axes.autoscale()
    self.axes.set_aspect('equal')
    plt.ion()
    plt.show()

  def update_plot(self):
    self.figure.canvas.draw_idle()
    self.figure.canvas.flush_events()

if __name__ == "__main__":
  robot_1 = Robot( 
    robot_pose=[5, 5, 0.0],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value]
  )
  robot_2 = Robot( 
    robot_pose=[-5, 5, 0.0],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value]
  )
  robot_3 = Robot( 
    robot_pose=[-5, -5, 0.0],
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
  robots = [robot_1]


  density_functions = [
    DensityFunction(
      type='gaussian',
      color=Color.CYAN.value,
      center=[5, 0],
      variance=[2, 2]
    ),
    DensityFunction(
      type='gaussian',
      color=Color.MAGENTA.value,
      center=[-5, 0],
      variance=[2, 2]
    ),
  ]

  env = np.array([
    [10, 10],
    [10, -10],
    [-10, -10],
    [-10, 10],
    [10, 10]
  ])

  swarm = Swarm(robots, env, density_functions)

  sim = Simulator(swarm, env)
  # vor_centroid, vor_cell, vor_area = swarm.coverage_control(robots, density_functions[0])

  sim.plot_environment(env)
  sim.plot_density_functions(density_functions)
  sim.plot_swarm(swarm)
  # sim.plot_voronoi(vor_centroid, vor_cell)
  sim.plot()

  for i in range(500):
    vor_centroid, vor_cell, vor_area = swarm.coverage_control(robots, density_functions[0])
    vor_robots, vor_prime = swarm.color_coverage_control()

    for j in range(len(robots)):
      robot = robots[j]
      vor_robot = vor_robots[j]
      vw = robot.coverage_control(vor_robot)
      color = robot.mix_color(vor_robot)

      print(j,vw, color)
      

    # for j, robot in enumerate(robots):
    #   vw = robot.move_to_point(vor_centroid[j], step=10)

    #   print(j, vw)


    sim.plot_swarm(swarm)
    sim.refresh_voronoi()
    for color, val in vor_prime.items():
      if val is None:
        continue
      centroid, cell, area = val
      sim.plot_voronoi(centroid, cell, refresh=False, centroid_color=color, boundary_color=color)

    sim.update_plot()
    time.sleep(0.01)



  pass

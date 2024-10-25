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

from utils import DensityFunction

class Simulator:
  def __init__(self, swarm, environment, **kwargs):
    self.swarm = swarm
    self.environment = environment

    self.figure, self.axes = plt.subplots()
    self.p_env = None
    self.p_density = None
    self.p_robots = None
    self.p_trails = None
    self.p_vor_centroid = None
    self.p_vor_cell = None



    # plt.ion()
    # plt.show(block=True)


  def plot_environment(self, environment):
    if self.p_env:
      self.p_env.remove()

    if environment is not None:
      self.environment = environment

    self.p_env = patches.Polygon(self.environment, fill=False)
    self.axes.add_patch(self.p_env)



  def plot_density_functions(self, density_functions, range=5, resolution=100):
    if self.p_density:
      for p in self.p_density:
        p.remove()

    self.p_density = []
    for density_function in density_functions:
      xmin, xmax = density_function.center[0] - range, density_function.center[0] + range
      ymin, ymax  = density_function.center[1] - range, density_function.center[1] + range

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


  def plot_voronoi(self, vor_centroid, vor_cell):

    if self.p_vor_centroid:
      for p_vor_centroid in self.p_vor_centroid:
        p_vor_centroid.remove()
    if self.p_vor_cell:
      for p_vor_cell in self.p_vor_cell:
        p_vor_cell.remove()

    self.p_vor_centroid = []
    self.p_vor_cell = []

    for centroid in vor_centroid:
      p = patches.Circle(centroid, radius=0.1, fill=True)
      self.p_vor_centroid.append(p)
      self.axes.add_patch(p)

    for cell in vor_cell:
      p = patches.Polygon(cell, fill=False, closed=True)
      self.p_vor_cell.append(p)
      self.axes.add_patch(p)

  def plot(self):
    self.axes.autoscale()
    self.axes.set_aspect('equal')
    # plt.ion()
    plt.show()

  def update_plot(self):
    self.figure.canvas.draw_idle()
    self.figure.canvas.flush_events()

if __name__ == "__main__":
  robots = [
    Robot(robot_pose=[5, 5, 1], equiped_color=[]),
    Robot(robot_pose=[-5, -5, -1],equiped_color=[]),
    Robot(robot_pose=[5, -5, 0.0],equiped_color=[]),
    Robot(robot_pose=[-5, 5, 0.0],equiped_color=[])
  ]

  env = np.array([
    [10, 10],
    [10, -10],
    [-10, -10],
    [-10, 10],
    [10, 10]
  ])

  density_functions = [
    DensityFunction(
      type='gaussian',
      phi = lambda x, y: np.exp(-0.5 * (x**2 + y**2))/ (2 * np.pi),
      color='#ff7f0e',
      center=[0, 0]
    ),
    DensityFunction(
      type='gaussian',
      phi = lambda x, y: np.exp(-0.5 * ((x-5)**2 + (y-5)**2))/ (2 * np.pi),
      color='#1f77b4',
      center=[5, 5]
    )
  ]

  swarm = Swarm(robots, env, density_functions)

  sim = Simulator(swarm, env)
  vor_centroid, vor_cell, _ = swarm.coverage_control()
  sim.plot_environment(env)
  sim.plot_density_functions(density_functions)
  sim.plot_swarm(swarm)
  sim.plot_voronoi(vor_centroid, vor_cell)
  sim.plot()
  # for i in range(100):
  #   time.sleep(0.1)
  # for i in range(500):
  #   vor_centroid, vor_cell, _ = swarm.converage_control()
  #   for i, robot in enumerate(robots):
  #     robot.move_to_point(vor_centroid[i])
  #   sim.plot_swarm(swarm)
  #   sim.plot_voronoi(vor_centroid, vor_cell)

  #   sim.update_plot()
  #   time.sleep(0.01)



  pass

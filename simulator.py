import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.lines import Line2D
import numpy as np
from matplotlib.colors import to_rgba
import math
from swarm import Swarm
from robot import Robot
from converage_control import DensityFunction
from scipy.spatial import Voronoi
import time

class Simulator:
  def __init__(self, swarm, environment, **kwargs):
    self.swarm = swarm
    self.environment = environment

    self.robot_color = kwargs.get('robot_color', 'black')
    self.robot_size = kwargs.get('robot_size', 0.5)
    self.trail_color = kwargs.get('trail_color', 'black')
    self.trail_width = kwargs.get('trail_width', 5)

    self.figure, self.axes = plt.subplots()
    self.p_env = None
    self.p_density = None
    self.p_robots = None



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

    self.p_robots = []
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

      p_robot = patches.Polygon(t+v @ R.T, color=self.robot_color, fill=True)
      p_trail = Line2D(trail[:,0], trail[:,1], color=self.trail_color, linewidth=self.trail_width)

      self.p_robots.append(p_robot)
      self.p_robots.append(p_trail)

      self.axes.add_patch(p_robot)
      self.axes.add_line(p_trail)

  def plot_voronoi(self):
    pass

  def plot(self):
    self.axes.autoscale()
    self.axes.set_aspect('equal')
    # self.figure.canvas.draw_idle()
    # self.figure.canvas.flush_events()
    plt.show()


if __name__ == "__main__":
  robots = [
    Robot(robot_pose=[5, 5, 1]),
    Robot(robot_pose=[-5, -5, -1]),
    Robot(robot_pose=[5, -5, 0.0]),
    Robot(robot_pose=[-5, 5, 0.0])
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
    )
  ]

  swarm = Swarm(robots, env, density_functions)

  sim = Simulator(swarm, env)
  sim.plot_environment(env)
  sim.plot_swarm(swarm)
  sim.plot_density_functions(density_functions)
  sim.plot()
  # for i in range(100):
  #   time.sleep(0.1)
  
  pass

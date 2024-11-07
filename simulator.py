import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.lines import Line2D
import numpy as np
from matplotlib.colors import to_rgba
import math
from swarm import Swarm
from robot import Robot
import time
from pipeline import ColorPipeline, LocationPipeline

from utils import Color, DensityFunction, Emotion

class Simulator:
  def __init__(self, swarm, environment):
    self.swarm = swarm
    self.environment = environment

    self.figure, self.axes = plt.subplots()
    self.p_env = None
    self.p_density = []
    self.p_robots = None
    self.p_trails = [[] for _ in range(len(swarm.robots))]
    self.p_vor_centroid = []
    self.p_vor_cell = []


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


    self.p_robots = []
    for i, robot in enumerate(swarm.robots):
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

      p_robot = patches.Polygon(t+v @ R.T, color=robot.robot_color, fill=True)
      self.p_robots.append(p_robot)
      self.axes.add_patch(p_robot)

      if len(robot.trail) > 1:
        segment = robot.trail[-2:]
        p_segment = Line2D(segment[:,0], segment[:,1], color=robot.trail_color, linewidth=robot.trail_width)
        self.p_trails[i].append(p_segment)
        self.axes.add_line(p_segment)

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
      self.axes.add_patch(p)

    for cell in vor_cell:
      p = patches.Polygon(cell, fill=False, closed=True, color=boundary_color)
      self.p_vor_cell.append(p)
      self.axes.add_patch(p)

  def clear_voronoi(self):
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
  robots = [robot_1, robot_2]


  density_functions = [
    DensityFunction(
      type='gaussian',
      color=Color.CYAN.value,
      center=[5, 0],
      variance=[1, 1]
    ),
    DensityFunction(
      type='gaussian',
      color=Color.MAGENTA.value,
      center=[-5, 0],
      variance=[1, 1]
    ),
  ]

  env = np.array([
    [10, 10],
    [10, -10],
    [-10, -10],
    [-10, 10],
    [10, 10]
  ])

  swarm = Swarm(robots, env)
  sim = Simulator(swarm, env)

  color_pipe = ColorPipeline()
  location_pipe = LocationPipeline()
  
  # get all emotions of Emotion
  emotions = list(Emotion)


  sim.plot_environment(env)
  sim.plot_swarm(swarm)
  sim.plot()

  for i in range(1000):
    if i % 100 == 0:
      idx = i // 100
      color_pipe.receive_emotions([emotions[idx]])
      colors = color_pipe.predict_colors()

      location_pipe.receive_emotions([emotions[idx]])
      locations = location_pipe.predict_locations()

      density_functions = [
        DensityFunction(
          type='gaussian',
          color=color_pipe.get_colors()[j].value,
          center=location_pipe.get_locations()[j],
          variance=[3, 3]
        ) for j in range(len(colors))
      ]
      sim.plot_density_functions(density_functions)

    vor_robots, vor_prime = swarm.color_coverage_control(density_functions)


    for j in range(len(robots)):
      robot = robots[j]
      vor_robot = vor_robots[j]

      if vor_robot is None:
        continue
      
      vw = robot.coverage_control(vor_robot, L=2, delta=10)
      color = robot.mix_color(vor_robot,
        swarm.cyan_density_functions,
        swarm.magenta_density_functions,
        swarm.yellow_density_functions
       ) 

      print(j,vw, color)


    sim.plot_swarm(swarm)
    sim.clear_voronoi()
    for color, val in vor_prime.items():
      if val is None:
        continue
      centroid, cell, area = val
      sim.plot_voronoi(centroid, cell, refresh=False, centroid_color=color, boundary_color=color)


    sim.update_plot()
    time.sleep(0.01)



  pass

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from robot import Robot
from converage_control import DensityFunction
from scipy.spatial import Voronoi
from voronoi import voronoi_centroids

class Swarm: 
  def __init__(self, 
    robots,
    environment,
    density_functions,
    show_voronoi: bool = False,
    show_density: bool = False,
    show_trail: bool = False,
  ):
    self.robots = robots
    self.environment = environment
    self.density_functions = density_functions
    self.show_voronoi = show_voronoi
    self.show_density = show_density
    self.show_trail = show_trail



    self.__init_plot()

  def get_robot_locations(self):
    return np.array([robot.robot_pose[:2] for robot in self.robots])


  def mirror_robots_about_environment(self):
    robot_locations = self.get_robot_locations()
    environment = self.environment

    num_robots = robot_locations.shape[0]
    num_sides = environment.shape[0] - 1
    mirrored_robots = np.full((num_robots* num_sides, 2), np.nan)
    for i in range(num_robots):
      robot_location = robot_locations[i, :]
      for j in range(num_sides):
        side_start = environment[j, :]
        side_end = environment[j + 1, :]
        side_vector = side_end - side_start

        vector_to_robot = robot_location - side_start

        side_length_squared = np.dot(side_vector, side_vector)
        projection_length = np.dot(vector_to_robot, side_vector) / side_length_squared
        projection_point = side_start + projection_length * side_vector

        mirrored_point = robot_location - 2 * (robot_location - projection_point)

        index = i * num_sides + j
        mirrored_robots[index, :] = mirrored_point
    return mirrored_robots


  def get_voronoi_cell(self):
    robot_locations = self.get_robot_locations()
    Np = robot_locations.shape[0]

    mirrored_robots =self.mirror_robots_about_environment()
    P = np.concatenate([robot_locations, mirrored_robots])

    vor = Voronoi(P)
    vor_cell = np.zeros(Np, dtype=object)
    V = vor.vertices
    
    for i in range(Np):
      point_idx = vor.point_region[i]
      vor_cell[i] = vor.regions[point_idx]

    for i in range(len(vor_cell)):
      vor_cell[i] = V[vor_cell[i]]

    return vor_cell



  def converage_control(self):
    robot_locations = self.get_robot_locations()
    density_function = self.density_functions[0]
    Np = robot_locations.shape[0]

    mirrored_robots =self.mirror_robots_about_environment()
    P = np.concatenate([robot_locations, mirrored_robots])

    vor = Voronoi(P)
    V = vor.vertices
    vor_cell = np.zeros(Np, dtype=object)

    for i in range(Np):
      point_idx = vor.point_region[i]
      vor_cell[i] = vor.regions[point_idx]


    vor_centroid = np.zeros((len(vor_cell), 2))
    area = np.zeros(len(vor_cell))

    for i in range(len(vor_cell)):
      vor_cell[i] = V[vor_cell[i]]
      Gi, area_i = voronoi_centroids(vor_cell[i], density_function)
      vor_centroid[i, :] = Gi
      area[i] = np.abs(area_i)

    return vor_centroid, vor_cell, area 

  def __init_plot(self):
    self.figure, self.axes = plt.subplots()
    self.p_vor_centroid = []
    self.p_vor_cell = []

    for robot in self.robots:
      self.axes.add_patch(robot.p_robot)
      if self.show_trail:
        self.axes.add_line(robot.p_trail)


    self.__plot_environment()
    self.__plot_voronoi()

    self.axes.autoscale()
    self.axes.axis('equal')
    plt.ion()
    # plt.show(block=True)
    plt.show()

  def update_plot(self):
    for robot in self.robots:
      robot.update_plot()
    
    self.__plot_voronoi()

    self.figure.canvas.draw_idle()
    self.figure.canvas.flush_events()

  def __plot_environment(self):
    p_env = patches.Polygon(self.environment, fill=False)
    self.axes.add_patch(p_env)

  def __plot_voronoi(self):
    vor_centroid, vor_cell, area = self.converage_control()
    
    for p_vor_centroid in self.p_vor_centroid:
      p_vor_centroid.remove()
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

  # def __plot_density(self, density_functions=None):
  #   # plot  the current density functions
  #   for p_density_function in self.p_density_functions:
  #     p_density_function.remove()
  #   self.p_density_functions = []

  #   if density_functions is not None:
  #     self.density_functions = density_functions
  #   elif self.density_functions is None:
  #     self.density_functions = []

  #   for density_function in self.density_functions:
  #     self.p_density_functions.append(self.axes.contour(
  #       density_function.X, 
  #       density_function.Y, 
  #       density_function.Z, levels=10, colors=density_function.color))




  def plot_environment(self):
    pass

  def plot_robots(self):
    pass






if __name__ == "__main__":
  robot_1 = Robot( robot_pose=[5, 5, 1])
  robot_2 = Robot( robot_pose=[-5, -5, -1])
  robot_3 = Robot( robot_pose=[5, -5, 0.0])
  robot_4 = Robot( robot_pose=[-5, 5, 0.0])
  robots = [robot_1, robot_2, robot_3, robot_4]

  density_functions = [
    DensityFunction(
      type='gaussian',
      phi = lambda x, y: np.exp(-0.5 * ((x-5)**2 + (y-5)**2))/ (2 * np.pi),
      color=None
    )
  ]

  env = np.array([
    [10, 10],
    [10, -10],
    [-10, -10],
    [-10, 10],
    [10, 10]
  ])

  swarm = Swarm(robots, env, density_functions)


  for i in range(1000):
    vor_centroid, vor_cell, area = swarm.converage_control()
    for i, robot in enumerate(robots):
      robot.move_to_point(vor_centroid[i])

    swarm.update_plot()
    time.sleep(0.001)
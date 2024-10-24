import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from robot import Robot
from converage_control import DensityFunction
from scipy.spatial import Voronoi
from voronoi import voronoi_centroids
from matplotlib.colors import hex2color, to_hex
import random

# rgb hex to cmy
def hex_to_cmy(color):
  rgb = np.array(hex2color(color))
  return 1 - rgb

def cmy_to_hex(cmy):
  rgb = 1 - cmy
  return to_hex(rgb)

class Swarm: 
  def __init__(self, 
    robots,
    environment,
    density_functions,
  ):
    self.robots = robots
    self.environment = environment
    self.density_functions = density_functions




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


  def get_cym_density_functions(self):
    C_density_function =  []
    M_density_functions = []
    Y_density_functions = []

    for density_function in self.density_functions:
      c, m, y = hex_to_cmy(density_function.color)
      C_density_function.append((c, density_function))
      M_density_functions.append((m, density_function))
      Y_density_functions.append((y, density_function))

    C_density_function = DensityFunction(
      type='gaussian',
      phi=lambda x, y: max([c * density_function.phi(x, y) for c, density_function in C_density_function]),
      color=cmy_to_hex([1, 0, 0])
    )

    M_density_function = DensityFunction(
      type='gaussian',
      phi=lambda x, y: max([m * density_function.phi(x, y) for m, density_function in M_density_functions]),
      color=cmy_to_hex([0, 1, 0])
    )

    Y_density_function = DensityFunction(
      type='gaussian',
      phi=lambda x, y: max([y * density_function.phi(x, y) for y, density_function in Y_density_functions]),
      color=cmy_to_hex([0, 0, 1])
    )

    return C_density_function, M_density_function, Y_density_function



  def converage_control(self):
    robot_locations = self.get_robot_locations()
    density_function = self.density_functions[0]
    for density_function in self.density_functions:
      c, m, y = hex_to_cmy(density_function.color)
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








if __name__ == "__main__":
  # robot_1 = Robot( robot_pose=[5, 5, 1])
  # robot_2 = Robot( robot_pose=[-5, -5, -1])
  # robot_3 = Robot( robot_pose=[5, -5, 0.0])
  # robot_4 = Robot( robot_pose=[-5, 5, 0.0])
  # robots = [robot_1, robot_2, robot_3, robot_4]

  # robots = []
  # for i in range(-5, -3):
  #   for j in range(-5, -3):
  #     robot = Robot(robot_pose=[i, j, 0])
  #     robots.append(robot)


  # density_functions = [
  #   DensityFunction(
  #     type='gaussian',
  #     phi = lambda x, y: np.exp(-0.5 * (x**2 + y**2))/ (2 * np.pi),
  #     color='#ff7f0e'
  #   ),
  #   # DensityFunction(
  #   #   type='gaussian',
  #   #   phi = lambda x, y: np.exp(-0.5 * ((x-3)**2 + (y-6)**2))/ (2 * np.pi),
  #   #   color='#ff7f0e'
  #   # ),
  # ]

  # env = np.array([
  #   [10, 10],
  #   [10, -10],
  #   [-10, -10],
  #   [-10, 10],
  #   [10, 10]
  # ])




  # swarm = Swarm(robots, env, density_functions)


  # for i in range(500):
  #   vor_centroid, vor_cell, area = swarm.converage_control()
  #   for i, robot in enumerate(robots):
  #     robot.move_to_point(vor_centroid[i])

  #   swarm.update_plot()
  #   time.sleep(0.001)
  pass

 
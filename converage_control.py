import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
from voronoi import voronoi_centroids
from matplotlib.colors import to_rgb


class DensityFunction:
  def __init__(self, type, phi, color):
    self.type = type   # 'uniform' or 'gaussian'
    self.phi = phi     # lambda x, y: float
    self.color = color # hex cmy color value


'''
  robot_locations: np.array
  environment: np.array, must be a closed polygon
'''
def mirror_robots_about_environment(
    robot_locations,
    environment,
):
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


def converage_control(robot_locations, environment, density_function):
  Np = robot_locations.shape[0]

  mirrored_robots =mirror_robots_about_environment(robot_locations, environment)
  P = np.concatenate([robot_locations, mirrored_robots])

  vor = Voronoi(P)
  V = vor.vertices
  vor_cell = np.zeros(Np, dtype=object)
  # find the regions for each robot
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
  robot_locations = np.array([
    [1,0],
    [0,1],
  ])

  environment = np.array([
    [2,2],
    [2,-2],
    [-2,-2],
    [-2,2],
    [2,2]
  ])

  density_function = DensityFunction(
    type='gaussian',
    phi = lambda x, y: np.exp(-0.5 * (x**2 + y**2))/ (2 * np.pi),
    color=None
  )
  vor_centroids, vor_cells, areas = converage_control(robot_locations, environment, density_function)
  print(vor_centroids)
  print(vor_cells)
  print(areas)
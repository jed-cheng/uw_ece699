import numpy as np
def voronoi():
  pass

def mirror_robots_about_environment(
    robot_locations,
    environment,
):
  # each robot have  a mirrored point for each edge of the environment

  num_robots = robot_locations.shape[1]
  num_sides = environment.shape[1] - 1
  mirrored_robots = np.full((2, num_robots * num_sides), np.nan)
  for i in range(num_robots):
    robot_location = robot_locations[:, i]
    for j in range(num_sides):
      side_start = environment[:, j]
      side_end = environment[:, j + 1]
      side_vector = side_end - side_start

      vector_to_robot = robot_location - side_start

      side_length_squared = np.dot(side_vector, side_vector)
      projection_length = np.dot(vector_to_robot, side_vector) / side_length_squared
      projection_point = side_start + projection_length * side_vector

      mirrored_point = robot_location - 2 * (robot_location - projection_point)

      index = i * num_sides + j
      mirrored_robots[:, index] = mirrored_point
  return mirrored_robots


def voronoi_centroids():
  pass


if __name__ == '__main__':
  robot_locations = np.array([
    [0,0]
  ]).T

  environment = np.array([
    [2,2],
    [2,-2],
    [-2,-2],
    [-2,2],
    [2,2]
  ]).T
  
  mirrored_robots = mirror_robots_about_environment(
    robot_locations,
    environment,
  )

  print(mirrored_robots.T)

  pass
import numpy as np
from robot import Robot
from scipy.spatial import Voronoi
from voronoi import voronoi_centroids
from matplotlib.colors import hex2color, to_hex
from utils import Color, DensityFunction

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
  ):
    self.robots = robots
    self.environment = environment
    self.cyan_density_functions = None
    self.magenta_density_functions = None
    self.yellow_density_functions = None

  


  def get_robot_locations(self):
    return np.array([robot.robot_pose[:2] for robot in self.robots])


  

  def mirror_robots_about_environment(self, robots):
    robot_locations = np.array([robot.robot_pose[:2] for robot in robots])
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


  def get_prime_density_functions(self, density_functions):
    prime_density_functions_map  = dict({
      Color.CYAN.value: [],
      Color.MAGENTA.value: [],
      Color.YELLOW.value: []
    })

    for density_function in density_functions:
      c, m, y = hex_to_cmy(density_function.color)

      if c > 0:
        prime_density_functions_map[Color.CYAN.value].append((c, density_function))
      if m > 0:
        prime_density_functions_map[Color.MAGENTA.value].append((m, density_function))
      if y > 0:
        prime_density_functions_map[Color.YELLOW.value].append((y, density_function))

    if len(prime_density_functions_map[Color.CYAN.value]):
      cyan_density_functions = prime_density_functions_map[Color.CYAN.value]
      self.cyan_density_functions = DensityFunction(
          type='gaussian',
          func = lambda x, y: max([value * prime_density_function.phi(x, y) for value, prime_density_function in cyan_density_functions]) if len(cyan_density_functions) > 0 else 0,
          color=Color.CYAN.value,
        )
      
    if len(prime_density_functions_map[Color.MAGENTA.value]):
      magenta_density_functions = prime_density_functions_map[Color.MAGENTA.value]
      self.magenta_density_functions = DensityFunction(
          type='gaussian',
          color=Color.MAGENTA.value,
          func = lambda x, y: max([value * prime_density_function.phi(x, y) for value, prime_density_function in magenta_density_functions]) if len(magenta_density_functions) > 0 else 0,
        )

    if len(prime_density_functions_map[Color.YELLOW.value]):
      yellow_density_functions = prime_density_functions_map[Color.YELLOW.value]
      self.yellow_density_functions = DensityFunction(
          type='gaussian',
          color=Color.YELLOW.value,
          func = lambda x, y: max([value * prime_density_function.phi(x, y) for value, prime_density_function in yellow_density_functions]) if len(yellow_density_functions) > 0 else 0,
        )
    return prime_density_functions_map



  def heterogenous_coverage_control(self, density_functions = None):
    if density_functions is not None:
      self.get_prime_density_functions(density_functions)
    
    vor_robots = dict({i: 
      dict({
        Color.CYAN.value: None,
        Color.MAGENTA.value: None,
        Color.YELLOW.value: None
      }) for i in range(len(self.robots))})
    vor_prime = dict({
      Color.CYAN.value: None,
      Color.MAGENTA.value: None,
      Color.YELLOW.value: None
    })

    if self.cyan_density_functions is None and self.magenta_density_functions is None and self.yellow_density_functions is None:
      return vor_robots, vor_prime

    for prime_color in [Color.CYAN.value, Color.MAGENTA.value, Color.YELLOW.value]:
      robots_with_color_list = [(i, robot) for i, robot in enumerate(self.robots) if prime_color in robot.equiped_color]
      if len(robots_with_color_list) == 0:
        continue
      
      if prime_color == Color.CYAN.value:
        density_function = self.cyan_density_functions
      elif prime_color == Color.MAGENTA.value:
        density_function = self.magenta_density_functions
      elif prime_color == Color.YELLOW.value:
        density_function = self.yellow_density_functions

      if density_function is None:
        continue

        
      robots_idx, robots_with_color = zip(*robots_with_color_list)
      vor_centroid, vor_cell, vor_area = self.coverage_control(robots_with_color, density_function)

      for i in range(len(robots_with_color_list)):
        vor_robots[robots_idx[i]][prime_color] = (vor_centroid[i], vor_cell[i], vor_area[i])
      
      vor_prime[prime_color] = (vor_centroid, vor_cell, vor_area)


    return vor_robots, vor_prime

        

  def coverage_control(self, robots, density_function):
    robot_locations = np.array([robot.robot_pose[:2] for robot in robots])
    Np = robot_locations.shape[0]

    mirrored_robots =self.mirror_robots_about_environment(robots)
    P = np.concatenate([robot_locations, mirrored_robots])

    vor = Voronoi(P)
    V = vor.vertices
    vor_cell = np.zeros(Np, dtype=object)

    # get bounded voronoi cell for each robot
    for i in range(Np):
      point_idx = vor.point_region[i]
      vor_cell[i] = vor.regions[point_idx]


    vor_centroid = np.zeros((len(vor_cell), 2))
    vor_area = np.zeros(len(vor_cell))

    # get voronoi centroid and area for each robot
    for i in range(len(vor_cell)):
      vor_cell[i] = V[vor_cell[i]]
      Gi, area_i = voronoi_centroids(vor_cell[i], density_function)
      vor_centroid[i, :] = Gi
      vor_area[i] = np.abs(area_i)

    return vor_centroid, vor_cell, vor_area 


  def run(self, density_functions, L=1):
    vor_robots, vor_prime = self.heterogenous_coverage_control(density_functions)

    # run until all robots are in their voronoi cell
    while True:
      for j in range(len(robots)):
        robot = robots[j]
        vor_robot = vor_robots[j]

        if vor_robot is None:
          continue
        
        vw = robot.coverage_control(vor_robot, L, delta=10)
        
        color = robot.coverage_control_color(vor_robot,
          swarm.cyan_density_functions,
          swarm.magenta_density_functions,
          swarm.yellow_density_functions
        )





if __name__ == "__main__":
  robot_1 = Robot( 
    robot_pose=[5, 5, 1],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value]
  )
  robot_2 = Robot( 
    robot_pose=[-5, -5, -1],
    equiped_color=[Color.CYAN.value]
  )
  robot_3 = Robot( 
    robot_pose=[5, -5, 0.0],
    equiped_color=[Color.CYAN.value, Color.MAGENTA.value]
  )
  robot_4 = Robot( 
    robot_pose=[-5, 5, 0.0],
    equiped_color=[Color.CYAN.value]
  )
  robots = [robot_1, robot_2, robot_3, robot_4]


  density_functions = [
    DensityFunction(
      type='gaussian',
      color=Color.CYAN.value,
      center=[0, 0],
      variance=[2, 2]
    ),
    DensityFunction(
      type='gaussian',
      color=Color.MAGENTA.value,
      center=[5, 5],
      vraiance=[2, 2]
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

  vor_robots = swarm.heterogenous_coverage_control()
  for i, vor_robot in vor_robots.items():
    print(i)
    centroid, cell, area = zip(*vor_robot)
    print(centroid)
    print(cell)
    print(area)

  for i in range(len(robots)):
    robot = robots[i]
    robot.coverage_control(vor_robots[i])
  pass

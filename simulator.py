import math
import random
import time
import matplotlib.patches as patches
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np

# 1. robot
# 2. trail
# 3. density
# 4. coverage control
class Robot:

  def __init__(self, 
    robot_pose,
    robot_size = 1.0,
    robot_color = 'yellow',
    trail = None,
    trail_color = None,
    trail_width = 5,
    TIMEOUT_SET_MOBILE_BASE_SPEED = 0
  ):

    self.robot_pose = robot_pose
    self.robot_size = robot_size
    self.robot_color = robot_color

    self.trail = np.array([robot_pose[:2]]) if trail is None else trail
    self.trail_color = trail_color
    self.trail_width = trail_width

    self.TIMEOUT_SET_MOBILE_BASE_SPEED = TIMEOUT_SET_MOBILE_BASE_SPEED
    self.TIMEOUT_GET_POSES = 0 # milliseconds

    # initialize robot
    self.__init_plot()
    self.last_time_set_mobile_base_speed = int(round(time.time()*1000))
    self.last_time_get_poses = int(round(time.time()*1000))

  def set_mobile_base_speed(self, v:float,  w:float):
    delta_time_set_mobile_base_speed = int(round(time.time()*1000)) - self.last_time_set_mobile_base_speed
    if delta_time_set_mobile_base_speed > self.TIMEOUT_SET_MOBILE_BASE_SPEED:
      self.robot_pose[0] = self.robot_pose[0] + (v * math.cos(self.robot_pose[2])) * delta_time_set_mobile_base_speed / 1000.0
      self.robot_pose[1] = self.robot_pose[1] + (v * math.sin(self.robot_pose[2])) * delta_time_set_mobile_base_speed / 1000.0
      self.robot_pose[2] = self.robot_pose[2] + w * delta_time_set_mobile_base_speed / 1000.0
      self.last_time_set_mobile_base_speed = int(round(time.time()*1000))

  def get_poses(self):
    while int(round(time.time()*1000)) - self.last_time_get_poses < self.TIMEOUT_GET_POSES:
      pass
    self.last_time_get_poses = int(round(time.time()*1000))
    return self.robot_pose

  def __init_plot(self):
    robot_pose = self.robot_pose
    robot_size = self.robot_size
    robot_trail = self.trail
    R = np.array([
      [math.cos(robot_pose[2]), -math.sin(robot_pose[2])],
      [math.sin(robot_pose[2]), math.cos(robot_pose[2])]
    ])
    t = np.array([robot_pose[0], robot_pose[1]])
    # v = np.array([
    #   [0, robot_size[1] / 2.0],  
    #   [-robot_size[0] / 2.0, -robot_size[1] / 2.0],  
    #   [robot_size[0] / 2.0, -robot_size[1] / 2.0],  
    # ])

    self.p_robot = patches.Circle(t, radius=robot_size, color=self.robot_color, fill=True)
    self.p_trail = Line2D(robot_trail[:,0], robot_trail[:,1], color=self.trail_color, linewidth=self.trail_width)

  def update_plot(self):
    # update robot
    robot_pose = self.robot_pose
    R = np.array([
      [math.cos(robot_pose[2]), -math.sin(robot_pose[2])],
      [math.sin(robot_pose[2]), math.cos(robot_pose[2])]
    ])
    t = np.array([robot_pose[0], robot_pose[1]])    
    # xy_robot = t + (np.array([0, self.robot_size[1] / 2.0]) @ R.T)
    self.p_robot.center = t

    # update trail
    self.trail = np.append(self.trail, [robot_pose[:2]], axis=0)
    self.p_trail.set_data(self.trail[:,0], self.trail[:,1])



class Swarm: 
  def __init__(self, 
    robots,
    density,
    environment = [[-10, -10], [-10, 10], [10, 10], [10, -10]],
    show_voronoi: bool = False,
    show_density: bool = False,
  ):
    self.robots = robots
    self.environment = environment
    self.density = density
    self.show_voronoi = show_voronoi
    self.show_density = show_density

    self.__init_plot()

  def __init_plot(self):
    self.figure, self.axes = plt.subplots()
    p_env = patches.Polygon(self.environment, fill=False)
    self.axes.add_patch(p_env)

    for robot in self.robots:
      self.axes.add_patch(robot.p_robot)
      self.axes.add_line(robot.p_trail)
    
    self.axes.autoscale()
    plt.ion()
    plt.show()

  def update_plot(self):
    for robot in self.robots:
      robot.update_plot()

    self.figure.canvas.draw_idle()
    self.figure.canvas.flush_events()

  def plot_density(self):
    pass

  def plot_voronoi(self):
    pass

  def plot_environment(self):
    pass

  def plot_robots(self):
    pass









if __name__ == "__main__":
  robot = Robot( robot_pose=[0.5, 0.5, 0.1])
  swarm = Swarm(robots=[robot], density=None)

  for i in range(100):
    robot.set_mobile_base_speed(v=1, w=0)
    swarm.update_plot()
    print(i)
    print(robot.get_poses())
    time.sleep(0.1)
  # import numpy as np
  # points = np.array([[0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2],
  #                   [2, 0], [2, 1], [2, 2]])
  # from scipy.spatial import Voronoi, voronoi_plot_2d
  # vor = Voronoi(points)
  # import matplotlib.pyplot as plt
  # fig = voronoi_plot_2d(vor)
  # plt.show()
import math, time
import numpy as np
from matplotlib import patches
from matplotlib.lines import Line2D



class Robot:

  def __init__(self, 
    robot_pose,
    equiped_color,
    robot_size = 0.5,
    robot_color = 'black',
    trail_width = 5,
    TIMEOUT_SET_MOBILE_BASE_SPEED = 0,
    K = 1
  ):

    self.robot_pose = robot_pose
    self.equiped_color = equiped_color

    self.robot_size = robot_size
    self.robot_color = robot_color

    self.trail = np.array([robot_pose[:2]])
    self.trail_width = trail_width

    self.trail_color = robot_color

    self.TIMEOUT_SET_MOBILE_BASE_SPEED = TIMEOUT_SET_MOBILE_BASE_SPEED
    self.TIMEOUT_GET_POSES = 0 # milliseconds
    self.K = K

    # initialize robot
    # self.__init_plot()
    self.last_time_set_mobile_base_speed = int(round(time.time()*1000))
    self.last_time_get_poses = int(round(time.time()*1000))

  def set_mobile_base_speed(self, v:float,  w:float, step=None):
    delta_time_set_mobile_base_speed = int(round(time.time()*1000)) - self.last_time_set_mobile_base_speed if step is None else step
    if delta_time_set_mobile_base_speed > self.TIMEOUT_SET_MOBILE_BASE_SPEED:
      self.robot_pose[0] = self.robot_pose[0] + (v * math.cos(self.robot_pose[2])) * delta_time_set_mobile_base_speed / 1000.0
      self.robot_pose[1] = self.robot_pose[1] + (v * math.sin(self.robot_pose[2])) * delta_time_set_mobile_base_speed / 1000.0
      self.robot_pose[2] = self.robot_pose[2] + w * delta_time_set_mobile_base_speed / 1000.0
      self.last_time_set_mobile_base_speed = int(round(time.time()*1000))
      
      self.trail = np.append(self.trail, [self.robot_pose[:2]], axis=0)

  def get_poses(self):
    while int(round(time.time()*1000)) - self.last_time_get_poses < self.TIMEOUT_GET_POSES:
      pass
    self.last_time_get_poses = int(round(time.time()*1000))
    return self.robot_pose
  
  def move_to_point(self, point):
    u = self.K * (point - self.robot_pose[:2])
    R = np.array([
      [math.cos(self.robot_pose[2]), math.sin(self.robot_pose[2])],
      [-math.sin(self.robot_pose[2]), math.cos(self.robot_pose[2])]
    ])
    vw = np.array([[1,0], [0,1]]) @ R @ u

    self.set_mobile_base_speed(vw[0], vw[1])


  def coverage_control(self, vor_prime):

    u = np.zeros(2)
    for centroid, _,  area in vor_prime:
      u += (centroid - self.robot_pose[:2]) * area
    R = np.array([
      [math.cos(self.robot_pose[2]), math.sin(self.robot_pose[2])],
      [-math.sin(self.robot_pose[2]), math.cos(self.robot_pose[2])]
    ])
    vw = np.array([[1,0], [0,1]]) @ R @ u

    self.set_mobile_base_speed(vw[0], vw[1])

  def set_trail_color(self, color):
    self.trail_color = color



if __name__ == '__main__':
  robot = Robot(robot_pose=[0, 0, 0])
  robot.move_to_point(np.array([10, 10]))
  
  pass
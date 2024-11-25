import math, time
import numpy as np

from utils import Color
from consts import L_MAX, L_MIN, TEMPO_MAX, TRAIL_WIDTH, L_DEFAULT, K_DEFAULT
from scipy.special import expit



class Robot:

  def __init__(self, 
    robot_pose,
    equiped_color,
    robot_size = 0.5,
    robot_color = 'black',
    trail_width = TRAIL_WIDTH,
    K = K_DEFAULT,
    L = L_DEFAULT,
    TIMEOUT_SET_MOBILE_BASE_SPEED = 0,
  ):

    self.robot_pose = robot_pose
    self.equiped_color = equiped_color

    self.robot_size = robot_size
    self.robot_color = robot_color

    self.trail = np.array([robot_pose[:2]])
    self.trail_width = trail_width
    self.trail_width_scalar = 1

    self.trail_color = robot_color

    self.TIMEOUT_SET_MOBILE_BASE_SPEED = TIMEOUT_SET_MOBILE_BASE_SPEED
    self.TIMEOUT_GET_POSES = 0 # milliseconds
    self.MAX_LINEAR_SPEED = 4 # meters / second
    self.MAX_ANGULAR_SPEED = 30 # degrees / second
    self.K = K
    self.L = L
    self.L_scalar = 10


    self.last_time_set_mobile_base_speed = int(round(time.time()*1000))
    self.last_time_get_poses = int(round(time.time()*1000))

  def set_mobile_base_speed(self, v:float,  w:float, delta=None):
    delta_time_set_mobile_base_speed = int(round(time.time()*1000)) - self.last_time_set_mobile_base_speed if delta is None else delta
    if delta_time_set_mobile_base_speed > self.TIMEOUT_SET_MOBILE_BASE_SPEED:
      v, omega = self.__saturate_speeds(v, w)
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


  def __saturate_speeds(self, v, omega):
    v = max(-self.MAX_LINEAR_SPEED, min(v, self.MAX_LINEAR_SPEED))
    omega = max(-self.MAX_ANGULAR_SPEED, min(omega, self.MAX_ANGULAR_SPEED))
    return v, omega

  def move_to_point(self, point, delta=None):
    u = self.K * (point - self.robot_pose[:2])
    R = np.array([
      [math.cos(self.robot_pose[2]), math.sin(self.robot_pose[2])],
      [-math.sin(self.robot_pose[2]), math.cos(self.robot_pose[2])]
    ])
    vw = np.array([[1,0], [0,1]]) @ R @ u

    self.set_mobile_base_speed(vw[0], vw[1], delta)

    return vw


  def coverage_control(self, vor_robot , delta=None):

    u = np.zeros(2)
    for color, val in vor_robot.items():
      if val is None:
        continue
      centroid, _, area = val
      # withouth the area, the robot will move to the centroid of the voronoi cell
      u += (centroid - self.robot_pose[:2]) * area
    u = self.K * u
    R = np.array([
      [math.cos(self.robot_pose[2]), math.sin(self.robot_pose[2])],
      [-math.sin(self.robot_pose[2]), math.cos(self.robot_pose[2])]
    ])
    #
    
    vw = np.array([[1,0], [0,1/self.get_L()]]) @ R @ u

    self.set_mobile_base_speed(vw[0], vw[1], delta)
    return vw

  def coverage_control_color(self, vor_robot, cyan_density, magenta_density, yellow_density):
    color = np.zeros(4)
    sum_area = 0
    for c, val in vor_robot.items():
      if val is None:
        continue
      _, _, area = val
      sum_area += area

    for c, val in vor_robot.items():
      if val is None:
        continue

      x, y = self.robot_pose[:2]
      _, _, area = val
      if c == Color.CYAN.value:
        color[0] = area / sum_area 
        color[3] += cyan_density.phi(x, y)
      elif c == Color.MAGENTA.value:
        color[1] = area / sum_area
        color[3] += magenta_density.phi(x, y) 
      elif c == Color.YELLOW.value:
        color[2] = area / sum_area
        color[3] += yellow_density.phi(x, y)
   
      

    color[3] = 1 - color[3]/len(self.equiped_color)

    color = 1 - color # rgb to cmy
    self.set_trail_color(color)
    return color
  
  def coverage_control_trail_width(self, vor_robot):
    sum_area = 0
    for c, val in vor_robot.items():
      if val is None:
        continue
      _, _, area = val
      sum_area += area
    
    self.trail_width_scalar = sum_area/len(self.equiped_color)

  def tempo_control_L(self, tempo):
    self.L_scalar = 10-((9)/TEMPO_MAX)*min(tempo, TEMPO_MAX)

  def get_L(self):
    return self.L_scalar * self.L

  def set_trail_width(self, width):
    self.trail_width = width

  def get_trail_width(self):
    return min(self.trail_width_scalar, 10)/2+self.trail_width
  
  def set_L(self, L):
    self.L = L
  
  def set_trail_color(self, color):
    self.trail_color = color




if __name__ == '__main__':
  robot = Robot(robot_pose=[0, 0, 0])
  robot.move_to_point(np.array([10, 10]))
  
  pass
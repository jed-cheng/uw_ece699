import math, time
import numpy as np
from matplotlib import patches
from matplotlib.lines import Line2D



class Robot:

  def __init__(self, 
    robot_pose,
    robot_size = 0.5,
    robot_color = 'black',
    trail = None,
    trail_color = None,
    trail_width = 5,
    TIMEOUT_SET_MOBILE_BASE_SPEED = 0,
    K = 1
  ):

    self.robot_pose = robot_pose
    self.robot_size = robot_size
    self.robot_color = robot_color

    self.trail = np.array([robot_pose[:2]]) if trail is None else trail
    self.trail_color = trail_color
    self.trail_width = trail_width

    self.TIMEOUT_SET_MOBILE_BASE_SPEED = TIMEOUT_SET_MOBILE_BASE_SPEED
    self.TIMEOUT_GET_POSES = 0 # milliseconds
    self.K = K

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
  
  def move_to_point(self, point):
    v = self.K * (point - self.robot_pose[:2])
    R = np.array([
      [math.cos(self.robot_pose[2]), math.sin(self.robot_pose[2])],
      [-math.sin(self.robot_pose[2]), math.cos(self.robot_pose[2])]
    ])
    vw = np.array([[1,0], [0,1]]) @ R @ v

    self.set_mobile_base_speed(vw[0], vw[1])


  def __init_plot(self):
    robot_pose = self.robot_pose
    robot_size = self.robot_size
    robot_trail = self.trail
    R = np.array([[0.0, 1.0], [-1.0, 0.0]]) @ np.array([
      [math.cos(robot_pose[2]), -math.sin(robot_pose[2])],
      [math.sin(robot_pose[2]), math.cos(robot_pose[2])]
    ])
    t = np.array([robot_pose[0], robot_pose[1]])
    v = np.array([
      [0, robot_size],
      [robot_size * -math.sqrt(3)/2 , -robot_size/2],
      [0,0],
      [robot_size * math.sqrt(3)/2, -robot_size/2]
    ])

    self.p_robot = patches.Polygon(t+v @ R.T, color=self.robot_color, fill=True)
    self.p_trail = Line2D(robot_trail[:,0], robot_trail[:,1], color=self.trail_color, linewidth=self.trail_width)

  def update_plot(self):
    # update robot
    robot_size = self.robot_size
    robot_pose = self.robot_pose
    R = np.array([[0.0, 1.0], [-1.0, 0.0]]) @ np.array([
      [math.cos(robot_pose[2]), -math.sin(robot_pose[2])],
      [math.sin(robot_pose[2]), math.cos(robot_pose[2])]
    ])
    t = np.array([robot_pose[0], robot_pose[1]])
    v = np.array([
      [0, robot_size],
      [robot_size * -math.sqrt(3)/2 , -robot_size/2],
      [0,0],
      [robot_size * math.sqrt(3)/2, -robot_size/2]
    ])
    
    xy_robot = t + (v @ R.T)
    self.p_robot.xy = xy_robot

    # update trail
    self.trail = np.append(self.trail, [robot_pose[:2]], axis=0)
    self.p_trail.set_data(self.trail[:,0], self.trail[:,1])


if __name__ == '__main__':
  robot = Robot(robot_pose=[0, 0, 0])
  robot.move_to_point(np.array([10, 10]))
  
  pass
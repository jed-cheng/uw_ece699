
import math
import random
import time
import matplotlib.patches as patches
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d

# 3. show density
# 4. coverage control
class Robot:

  def __init__(self, 
    robot_pose,
    robot_size = 0.5,
    robot_color = 'black',
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


class DensityFunction:
  def __init__(self, Z, color, x = np.linspace(-5, 5, 100),y = np.linspace(-5, 5, 100)):
    self.X, self.Y = np.meshgrid(x, y)
    self.Z = Z
    self.color = color


class Swarm: 
  def __init__(self, 
    robots,
    density_functions,
    environment = [[-10, -10], [-10, 10], [10, 10], [10, -10]],
    show_voronoi: bool = False,
    show_density: bool = False,
  ):
    self.robots = robots
    self.environment = environment
    self.density_functions = density_functions
    self.show_voronoi = show_voronoi
    self.show_density = show_density
    self.fininite_segments_line = None
    self.infinite_segments_line = None
    self.p_density_functions = []


    self.__init_plot()

  def __init_plot(self):
    self.figure, self.axes = plt.subplots()
    p_env = patches.Polygon(self.environment, fill=False)
    self.axes.add_patch(p_env)

    for robot in self.robots:
      self.axes.add_patch(robot.p_robot)
      self.axes.add_line(robot.p_trail)
    
    self.plot_voronoi(show_points=False, show_vertices=False)
    self.plot_density()

    self.axes.autoscale()
    self.axes.set_axis_off()
    self.axes.axis('equal')
    plt.ion()
    # plt.show(block=True)
    plt.show()

  def update_plot(self):
    for robot in self.robots:
      robot.update_plot()
    
    self.plot_voronoi(show_points=False, show_vertices=False)
    self.plot_density()

    self.figure.canvas.draw_idle()
    self.figure.canvas.flush_events()

  def plot_density(self, density_functions=None):
    # plot  the current density functions
    for p_density_function in self.p_density_functions:
      p_density_function.remove()
    self.p_density_functions = []

    if density_functions is not None:
      self.density_functions = density_functions
    elif self.density_functions is None:
      self.density_functions = []

    for density_function in self.density_functions:
      self.p_density_functions.append(self.axes.contour(
        density_function.X, 
        density_function.Y, 
        density_function.Z, levels=10, colors=density_function.color))


  def plot_voronoi(self, **kw):
    ax = self.axes
    points = np.array([robot.robot_pose[:2] for robot in self.robots])
    vor = Voronoi(points)

    if self.fininite_segments_line is not None:
      self.fininite_segments_line.remove()
    if self.infinite_segments_line is not None:
      self.infinite_segments_line.remove()

    from matplotlib.collections import LineCollection

    if vor.points.shape[1] != 2:
        raise ValueError("Voronoi diagram is not 2-D")

    if kw.get('show_points', True):
        point_size = kw.get('point_size', None)
        ax.plot(vor.points[:, 0], vor.points[:, 1], '.', markersize=point_size)
    if kw.get('show_vertices', True):
        ax.plot(vor.vertices[:, 0], vor.vertices[:, 1], 'o')

    line_colors = kw.get('line_colors', 'k')
    line_width = kw.get('line_width', 1.0)
    line_alpha = kw.get('line_alpha', 1.0)

    center = vor.points.mean(axis=0)
    ptp_bound = np.ptp(vor.points, axis=0)

    finite_segments = []
    infinite_segments = []
    for pointidx, simplex in zip(vor.ridge_points, vor.ridge_vertices):
        simplex = np.asarray(simplex)
        if np.all(simplex >= 0):
            finite_segments.append(vor.vertices[simplex])
        else:
            i = simplex[simplex >= 0][0]  # finite end Voronoi vertex

            t = vor.points[pointidx[1]] - vor.points[pointidx[0]]  # tangent
            t /= np.linalg.norm(t)
            n = np.array([-t[1], t[0]])  # normal

            midpoint = vor.points[pointidx].mean(axis=0)
            direction = np.sign(np.dot(midpoint - center, n)) * n
            if (vor.furthest_site):
                direction = -direction
            aspect_factor = abs(ptp_bound.max() / ptp_bound.min())
            far_point = vor.vertices[i] + direction * ptp_bound.max() * aspect_factor

            infinite_segments.append([vor.vertices[i], far_point])

    finite_segments_line = LineCollection(finite_segments, colors=line_colors, lw=line_width, alpha=line_alpha, linestyle='solid')
    self.fininite_segments_line = finite_segments_line
    ax.add_collection(finite_segments_line)

    infinite_segments_line = LineCollection(infinite_segments, colors=line_colors, lw=line_width, alpha=line_alpha, linestyle='dashed')
    self.infinite_segments_line = infinite_segments_line
    ax.add_collection(infinite_segments_line)

    self._adjust_bounds(ax, vor.points)


  def _adjust_bounds(self, ax, points):
    margin = 0.1 * np.ptp(points, axis=0)
    xy_min = points.min(axis=0) - margin
    xy_max = points.max(axis=0) + margin
    ax.set_xlim(xy_min[0], xy_max[0])
    ax.set_ylim(xy_min[1], xy_max[1])

  def mirror_robot(self, robot):
    environment = self.environment
    M = environment.shape[1]
    mirrored_positions = np.empty(2, M -1)
    point = robot.robot_pose[:2]

    for j in range(M - 1):
      # Get the start and end points of the environment edge
      side_start = environment[:, j].reshape(2, 1)
      side_end = environment[:, j + 1].reshape(2, 1)
      side = side_end - side_start  # Edge vector

      # Compute the vector from the edge start to the robot position
      point_wrt_side = point - side_start

      # Compute the projection of the point onto the edge
      side_length_squared = np.dot(side.T, side)[0, 0]
      if side_length_squared == 0:
          # Avoid division by zero for degenerate edge
          mirrored_positions[:, j] = point.flatten()
          continue

      length_of_projection = np.dot(point_wrt_side.T, side)[0, 0] / side_length_squared
      projected_vector = side_start + length_of_projection * side

      # Compute the mirrored point
      mirrored_point = point - 2 * (point - projected_vector)

      # Store the mirrored point
      mirrored_positions[:, j] = mirrored_point.flatten()
    return mirrored_positions

  def plot_environment(self):
    pass

  def plot_robots(self):
    pass






if __name__ == "__main__":
  robot_1 = Robot( robot_pose=[5, 5, 1])
  robot_2 = Robot( robot_pose=[-5, -5, -1])
  robot_3 = Robot( robot_pose=[5, -5, 0.1])
  robot_4 = Robot( robot_pose=[-5, 5, 0.1])
  robots = [robot_1, robot_2, robot_3]
  x = np.linspace(-5, 5, 100)
  y = np.linspace(-5, 5, 100)
  X, Y = np.meshgrid(x, y)
  mu = np.array([0, 0])
  Z = np.exp(-((X - mu[0])**2 + (Y - mu[1])**2))
  density_function = DensityFunction(Z, color='blue')
  swarm = Swarm(robots=[robot_1, robot_2, robot_3, robot_4], density_functions=[density_function])

  for i in range(100):
    robot_2.set_mobile_base_speed(v=0.5, w=0)
    swarm.update_plot()
    time.sleep(0.1)

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

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

  # Iterate over each prime color to compute Voronoi regions
  for prime_color in [Color.CYAN.value, Color.MAGENTA.value, Color.YELLOW.value]:
    # Get the list of robots equipped with the current prime color
    robots_with_color_list = [(i, robot) for i, robot in enumerate(self.robots) if prime_color in robot.equiped_color]
    if len(robots_with_color_list) == 0:
      continue
    
    # Select the appropriate density function based on the prime color
    if prime_color == Color.CYAN.value:
      density_function = self.cyan_density_functions
    elif prime_color == Color.MAGENTA.value:
      density_function = self.magenta_density_functions
    elif prime_color == Color.YELLOW.value:
      density_function = self.yellow_density_functions

    if density_function is None:
      continue

    robots_idx, robots_with_color = zip(*robots_with_color_list)
    # Compute the Voronoi centroids, cells, and areas
    vor_centroid, vor_cell, vor_area = self.coverage_control(robots_with_color, density_function)

    # Update the Voronoi regions for each robot
    for i in range(len(robots_with_color_list)):
      vor_robots[robots_idx[i]][prime_color] = (vor_centroid[i], vor_cell[i], vor_area[i])
    
    # Update the Voronoi regions for the prime color
    vor_prime[prime_color] = (vor_centroid, vor_cell, vor_area)

  return vor_robots, vor_prime

def voronoi_centroids(
    voronoi_cell, # array of vertices of voronoi cells
    density_function # density function
):
        
  phiA = lambda x, y: np.maximum(np.finfo(float).eps, density_function.phi(x, y))

  phiSx = lambda x, y: x * density_function.phi(x, y)
  phiSy = lambda x, y: y * density_function.phi(x, y)
  tri = Delaunay(voronoi_cell)
  
  
  area = 0.0
  S = np.zeros(2)

  for simplex in tri.simplices:
      tri_coords = voronoi_cell[simplex]
      area += integrate_over_triangle(phiA, tri_coords)
      
      Sx = integrate_over_triangle(phiSx, tri_coords)
      Sy = integrate_over_triangle(phiSy, tri_coords)
      S += np.array([Sx, Sy])
  
  G = S / area
  return G, area
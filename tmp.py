import numpy as np
from scipy.spatial import Delaunay
from matplotlib.path import Path

def centroid(environment, vertices):
    """
    Computes the centroid and area of a Voronoi cell given a density function.
    
    Parameters:
    - environment: An object containing the density function 'phi'.
                   'phi' can be 'uniform' or a callable function phi(x, y).
    - vertices: A 2D numpy array of shape (2, N) representing the vertices of the Voronoi cell polygon.
    
    Returns:
    - centroid: A numpy array representing the centroid of the Voronoi cell.
    - area: The area of the Voronoi cell.
    """
    if environment.phi == 'uniform':
        centroid, area = compute_uniform_centroid(vertices)
    else:
        centroid, area = compute_nonuniform_centroid(vertices, environment.phi)
    return centroid, area

def compute_uniform_centroid(vertices):
    """
    Computes the centroid and area of a polygon with uniform density.
    
    Parameters:
    - vertices: A 2D numpy array of shape (2, N) representing the polygon vertices.
    
    Returns:
    - centroid: A numpy array representing the centroid of the polygon.
    - area: The area of the polygon.
    """
    num_vertices = vertices.shape[1]
    rotation_matrix = np.array([[0, 1], [-1, 0]])  # 90 degrees counter-clockwise rotation
    area = 0.0
    moment_sum = np.zeros(2)
    
    for i in range(num_vertices):
        current_vertex = vertices[:, i]
        next_vertex = vertices[:, (i + 1) % num_vertices]  # Wrap around to the first vertex
        rotated_next_vertex = rotation_matrix @ next_vertex
        cross_product = current_vertex.T @ rotated_next_vertex
        area += cross_product
        moment_sum += cross_product * (current_vertex + next_vertex)
    
    area *= 0.5
    moment_sum /= 6.0
    centroid = moment_sum / area
    return centroid, area

def compute_nonuniform_centroid(vertices, phi):
    """
    Computes the centroid and area of a polygon with a non-uniform density function.
    
    Parameters:
    - vertices: A 2D numpy array of shape (2, N) representing the polygon vertices.
    - phi: A callable function phi(x, y) representing the density at point (x, y).
    
    Returns:
    - centroid: A numpy array representing the centroid of the polygon.
    - area: The area of the polygon considering the density function.
    """
    x_coords = vertices[0, :]
    y_coords = vertices[1, :]
    points = np.vstack((x_coords, y_coords)).T  # Shape (N, 2)
    
    # Define functions for area and centroid calculations with density
    def phi_area(x, y):
        return np.maximum(np.finfo(float).eps, phi(x, y))
    
    def phi_sx(x, y):
        return x * phi_area(x, y)
    
    def phi_sy(x, y):
        return y * phi_area(x, y)
    
    # Perform Delaunay triangulation
    delaunay = Delaunay(points)
    
    # Create a path object for the polygon to check point inclusion
    polygon_path = Path(points)
    
    area = 0.0
    moment_sum = np.zeros(2)
    
    # Iterate over each simplex (triangle) in the triangulation
    for simplex in delaunay.simplices:
        triangle = points[simplex]
        # Check if the triangle is inside the polygon
        if np.all(polygon_path.contains_points(triangle)):
            # Compute the integrals over the triangle
            triangle_area = integrate_over_triangle(phi_area, triangle)
            sx = integrate_over_triangle(phi_sx, triangle)
            sy = integrate_over_triangle(phi_sy, triangle)
            area += triangle_area
            moment_sum += np.array([sx, sy])
    
    centroid = moment_sum / area
    return centroid, area

def integrate_over_triangle(func, triangle):
    """
    Numerically integrates a function over a triangle using a simple quadrature method.
    
    Parameters:
    - func: The function to integrate. It should accept two arguments (x, y).
    - triangle: A (3, 2) numpy array containing the vertices of the triangle.
    
    Returns:
    - integral: The numerical integral of the function over the triangle.
    """
    # Compute the area of the triangle
    tri_area = triangle_area(triangle)
    
    # Use the centroid of the triangle for a simple quadrature (degree 1)
    centroid = np.mean(triangle, axis=0)
    x_centroid, y_centroid = centroid
    integral = func(x_centroid, y_centroid) * tri_area
    return integral

def triangle_area(triangle):
    """
    Computes the area of a triangle given its vertices.
    
    Parameters:
    - triangle: A (3, 2) numpy array containing the vertices of the triangle.
    
    Returns:
    - area: The area of the triangle.
    """
    x1, y1 = triangle[0]
    x2, y2 = triangle[1]
    x3, y3 = triangle[2]
    return 0.5 * abs((x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1))


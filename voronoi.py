import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay, Voronoi, voronoi_plot_2d
from converage_control import mirror_robots_about_environment


def tri_gauss_points(n):
    if n == 1:
        xyw = [
            (1/3, 1/3, 1.0)
        ]
    elif n == 2:
        xyw = [
            (1/6, 1/6, 1/3),
            (1/6, 2/3, 1/3),
            (2/3, 1/6, 1/3)
        ]
    elif n == 3:
        xyw = [
            (1/3, 1/3, -9/16),
            (0.2, 0.2, 25/48),
            (0.2, 0.6, 25/48),
            (0.6, 0.2, 25/48)
        ]
    elif n == 4:
        xyw = [
            (0.44594849091597, 0.44594849091597, 0.22338158967801),
            (0.44594849091597, 0.10810301816807, 0.22338158967801),
            (0.10810301816807, 0.44594849091597, 0.22338158967801),
            (0.09157621350977, 0.09157621350977, 0.10995174365532),
            (0.09157621350977, 0.81684757298046, 0.10995174365532),
            (0.81684757298046, 0.09157621350977, 0.10995174365532)
        ]
    elif n == 5:
        xyw = [
            (1/3, 1/3, 0.22500000000000),
            (0.47014206410511, 0.47014206410511, 0.13239415278851),
            (0.47014206410511, 0.05971587178977, 0.13239415278851),
            (0.05971587178977, 0.47014206410511, 0.13239415278851),
            (0.10128650732346, 0.10128650732346, 0.12593918054483),
            (0.10128650732346, 0.79742698535309, 0.12593918054483),
            (0.79742698535309, 0.10128650732346, 0.12593918054483)
        ]
    elif n == 6:
        xyw = [
            (0.24928674517091, 0.24928674517091, 0.11678627572638),
            (0.24928674517091, 0.50142650965818, 0.11678627572638),
            (0.50142650965818, 0.24928674517091, 0.11678627572638),
            (0.06308901449150, 0.06308901449150, 0.05084490637021),
            (0.06308901449150, 0.87382197101700, 0.05084490637021),
            (0.87382197101700, 0.06308901449150, 0.05084490637021),
            (0.31035245103378, 0.63650249912140, 0.08285107561837),
            (0.63650249912140, 0.05314504984482, 0.08285107561837),
            (0.05314504984482, 0.31035245103378, 0.08285107561837),
            (0.63650249912140, 0.31035245103378, 0.08285107561837),
            (0.31035245103378, 0.05314504984482, 0.08285107561837),
            (0.05314504984482, 0.63650249912140, 0.08285107561837)
        ]
    elif n == 7:
        xyw = [
            (1/3, 1/3, -0.14957004446768),
            (0.26034596607904, 0.26034596607904, 0.17561525743321),
            (0.26034596607904, 0.47930806784192, 0.17561525743321),
            (0.47930806784192, 0.26034596607904, 0.17561525743321),
            (0.06513010290222, 0.06513010290222, 0.05334723560884),
            (0.06513010290222, 0.86973979419557, 0.05334723560884),
            (0.86973979419557, 0.06513010290222, 0.05334723560884),
            (0.31286549600487, 0.63844418856981, 0.07711376089026),
            (0.63844418856981, 0.04869031542532, 0.07711376089026),
            (0.04869031542532, 0.31286549600487, 0.07711376089026),
            (0.63844418856981, 0.31286549600487, 0.07711376089026),
            (0.31286549600487, 0.04869031542532, 0.07711376089026),
            (0.04869031542532, 0.63844418856981, 0.07711376089026)
        ]
    elif n == 8:
        xyw = [
            (1/3, 1/3, 0.14431560767779),
            (0.45929258829272, 0.45929258829272, 0.09509163426728),
            (0.45929258829272, 0.08141482341455, 0.09509163426728),
            (0.08141482341455, 0.45929258829272, 0.09509163426728),
            (0.17056930775176, 0.17056930775176, 0.10321737053472),
            (0.17056930775176, 0.65886138449648, 0.10321737053472),
            (0.65886138449648, 0.17056930775176, 0.10321737053472),
            (0.05054722831703, 0.05054722831703, 0.03245849762320),
            (0.05054722831703, 0.89890554336594, 0.03245849762320),
            (0.89890554336594, 0.05054722831703, 0.03245849762320),
            (0.26311282963464, 0.72849239295540, 0.02723031417443),
            (0.72849239295540, 0.00839477740996, 0.02723031417443),
            (0.00839477740996, 0.26311282963464, 0.02723031417443),
            (0.72849239295540, 0.26311282963464, 0.02723031417443),
            (0.26311282963464, 0.00839477740996, 0.02723031417443),
            (0.00839477740996, 0.72849239295540, 0.02723031417443)
        ]
    else:
        raise ValueError(f"No quadrature rule implemented for n = {n}")
    return xyw
   


def integrate_over_triangle(f, tri_coords, N=8):
    x1, y1 = tri_coords[0]
    x2, y2 = tri_coords[1]
    x3, y3 = tri_coords[2]
    xyw = tri_gauss_points(N)
    area = triangle_area(tri_coords)
    
    integral = 0.0
    for xi, eta, weight in xyw:
        x = x1 * (1 - xi - eta) + x2 * xi + x3 * eta
        y = y1 * (1 - xi - eta) + y2 * xi + y3 * eta
        integral += f(x, y) * weight
    integral *= area
    return integral

def triangle_area(triangle):
    x1, y1 = triangle[0]
    x2, y2 = triangle[1]
    x3, y3 = triangle[2]
    return 0.5 * abs((x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1))



def voronoi_centroids(
    voronoi_cell, # array of vertices of voronoi cells
    phi # density function
):
    xP = voronoi_cell[:, 0]
    yP = voronoi_cell[:, 1]
        
        # Ensure the density function returns at least machine epsilon to avoid zero division
    phiA = lambda x, y: np.maximum(np.finfo(float).eps, phi(x, y))
    phiSx = lambda x, y: x * phi(x, y)
    phiSy = lambda x, y: y * phi(x, y)
    tri = Delaunay(voronoi_cell)
    
    # Triangulate the polygon using scipy's Delaunay triangulation
    
    area = 0.0
    S = np.zeros(2)  # Initialize moment summation for centroid calculation

    # Iterate over each triangle in the triangulation
    for simplex in tri.simplices:
        tri_coords = voronoi_cell[simplex]
        area += integrate_over_triangle(phiA, tri_coords)
        
        Sx = integrate_over_triangle(phiSx, tri_coords)
        Sy = integrate_over_triangle(phiSy, tri_coords)
        S += np.array([Sx, Sy])
    
    G = S / area
    return G, area

if __name__ == '__main__':
  # if only one robot, the voronoi cell is the environment
  robot_locations = np.array([
    [0,0],
    [1,0]
  ])

  environment = np.array([
    [2,2],
    [2,-2],
    [-2,-2],
    [-2,2],
    [2,2]
  ])

  mirrored_robots = mirror_robots_about_environment(robot_locations, environment)
  # print(mirrored_robots)

  P = np.concatenate([robot_locations, mirrored_robots])
  # print(P)
  vor = Voronoi(P)


  # V = vor.vertices
  # C = vor.regions
  # vc_p_map = vor.point_region
  # for i in range(len(C)):
  #     #  map the index of the voronoi cell to the vertices 
  #     if not -1 in C[i] and C[i]:
  #       VC = [V[j] for j in C[i]]
  #       print(VC)
  #       print(vc_p_map[i])
  #       print(P[vc_p_map[i]])

  # points = np.array([
  #     [0, 0],
  #     [1, 0],
  #     [0, 1],
  #     [1, 1],
  #     [2,2]
  # ])


  # tri = Delaunay(points)
  # plt.triplot(points[:,0], points[:,1], tri.simplices)
  # plt.plot(points[:,0], points[:,1], 'o')
  # plt.show()

  vc = np.array([
    [0, 0],
    [1, 0],
    [0, 1],
    [1, 1],
  ])
  phi = lambda x, y: 1.0

  G, area = voronoi_centroids(vc, phi)



  pass
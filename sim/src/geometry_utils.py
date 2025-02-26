from math import cos, sin
import numpy as np

def transform_velocity_global_to_local(robots_speeds, theta):
    # robots_speeds : np.array, 3 x N
    # theta : np.array, 1 x N
    N = robots_speeds.shape[1]
    robots_speeds_local = np.zeros((3, N))
    for i in range(N):
        x_dot = robots_speeds[0, i]
        y_dot = robots_speeds[1, i]
        c_th = cos(theta[i])
        s_th = sin(theta[i])
        robots_speeds_local[:2, i] = np.array([c_th * x_dot + s_th * y_dot, -s_th * x_dot + c_th * y_dot])
    robots_speeds_local[2, :] = robots_speeds[2, :]
    return robots_speeds_local

def transform_velocity_local_to_global(robots_speeds, theta):
    # robots_speeds : np.array, 3 x N
    # theta : np.array, 1 x N
    N = robots_speeds.shape[1]
    robots_speeds_global = np.zeros((3, N))
    for i in range(N):
        x_dot = robots_speeds[0, i]
        y_dot = robots_speeds[1, i]
        c_th = cos(theta[i])
        s_th = sin(theta[i])
        robots_speeds_global[:2, i] = np.array([c_th * x_dot - s_th * y_dot, s_th * x_dot + c_th * y_dot])
    robots_speeds_global[2, :] = robots_speeds[2, :]
    return robots_speeds_global

# N = 5
# x = np.random.random((3, N))
# x[2, :] = -np.pi/2 * np.ones((1, N))
# u = np.random.random((3, N))

# u_local = transform_velocity_global_to_local(u, x[2, :])
# u_global = transform_velocity_local_to_global(u, x[2, :])

# print(x)
# print(u)
# print(u_local)
# print(u_global)
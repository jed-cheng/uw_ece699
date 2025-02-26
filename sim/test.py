import numpy as np
N = 6
INITIAL_ROBOTS_POSES = np.vstack((np.linspace(-2., 2., N), np.zeros((1, N)), np.pi * np.ones((1, N))))

print(INITIAL_ROBOTS_POSES)
print(INITIAL_ROBOTS_POSES.T)
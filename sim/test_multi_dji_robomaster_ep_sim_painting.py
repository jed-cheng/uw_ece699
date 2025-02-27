from src.multi_dji_robomaster_ep import MultiDJIRoboMasterEP
import numpy as np

ROBOT_IDS = [3,4,5,6,7,8]
N = len(ROBOT_IDS)
INITIAL_ROBOTS_POSES = np.vstack((np.linspace(-2., 2., N), np.zeros((1, N)), np.pi * np.ones((1, N))))

mrs = MultiDJIRoboMasterEP(ROBOT_IDS,
                            backend_server_ip='192.168.0.2',
                            initial_robots_poses=INITIAL_ROBOTS_POSES,
                            safety_layer=True,
                            safety_radius=0.5)

for t in np.arange(0., 10., mrs.DT):
    # get poses
    q = mrs.get_robots_poses()
    print('robot poses', q)

    # music-driven swarm control algortihm
    # ...

    # set control inputs and leds
    v = 1.*np.ones((1, N))
    omega = 1.*np.ones((1, N))
    robots_speeds = np.vstack((v, np.zeros((1, N)), omega)) # longitudinal, lateral=0, angular in a local reference frame
    robots_leds = np.random.randint(0, 255, (3, N))
        
    # send inputs to robots
    mrs.set_robots_speeds_and_grippers_powers(robots_speeds, np.zeros((1, N)), local_frame=True)
    mrs.set_leds(robots_leds)
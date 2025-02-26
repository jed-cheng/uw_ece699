import numpy as np
from qpsolvers import solve_qp
import time
from src.dji_robomaster_ep_mqtt_interface import DJIRoboMasterEPMqttInterface
from src.geometry_utils import transform_velocity_global_to_local, transform_velocity_local_to_global

class MultiDJIRoboMasterEP:
    def __init__(self, robot_ids, backend_server_ip=None, initial_robots_poses=None, safety_layer=True, safety_radius=None):
        # constants
        # robots
        self.ROBOT_IDS = robot_ids
        self.N = len(self.ROBOT_IDS)
        self.robots = []
        for id in self.ROBOT_IDS:
            self.robots.append(DJIRoboMasterEPMqttInterface(robot_id=id, backend_server_ip=backend_server_ip))
        # robot control
        self.MAX_LINEAR_SPEED = 5.0 # meters / second
        self.MAX_ANGULAR_SPEED = 2 * np.pi # degrees / second
        self.SAFETY_LAYER = safety_layer
        self.GAMMA = 10.
        self.P = np.eye(2 * self.N)
        # dimensions
        # TODO: use environment size (or rather corners, to stay implement "wall barriers")
        self.ROBOT_SIZE = [0.24, 0.32] # [w, l]
        if safety_radius is None:
            self.SAFETY_RADIUS = max(self.ROBOT_SIZE) * 2.2
        else:
            self.SAFETY_RADIUS = safety_radius
        self.INIT_POSES_THRESH = 0.5

        # initialize positions of the robots
        self.robots_poses = np.empty((3, self.N))
        print('Trying to receive all robots poses ...')
        all_poses_received = False
        while not all_poses_received:
            _, all_poses_received = self.get_robots_poses(check_all_poses_received=True)
            time.sleep(0.1)
        print('... all poses received')

        if initial_robots_poses is not None:
            print('Initializing robots to specified initial poses ...')
            while np.linalg.norm(initial_robots_poses - self.robots_poses) > self.INIT_POSES_THRESH:
                self.get_robots_poses()
                robots_speeds = 10. * (initial_robots_poses - self.robots_poses)
                self.set_robots_speeds_and_grippers_powers(robots_speeds, np.zeros((self.N, )))
            self.set_leds(np.tile(np.array([[173], [216], [230]], dtype=int), (1, self.N)))
            print('... done (LEDs should be blue now)')
            time.sleep(1.)

    def __safe_and_max_speeds(self, robots_speeds):
        for i in range(self.N):
            ui_norm = np.linalg.norm(robots_speeds[:2, i])
            if ui_norm > self.MAX_LINEAR_SPEED:
                robots_speeds[:2, i] = robots_speeds[:2, i] / ui_norm * self.MAX_LINEAR_SPEED
            robots_speeds[2, :] = np.clip(robots_speeds[2, :], -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED)
        if self.SAFETY_LAYER:
            q = -2. * robots_speeds[:2, :].T.reshape((1, 2 * self.N))
            Aqp = np.zeros((int(self.N * (self.N - 1) / 2), 2 * self.N))
            bqp = np.zeros((int(self.N * (self.N - 1) / 2), ))
            constraint_idx = 0
            for i in range(self.N):
                for j in range(i + 1, self.N):
                    Aqp[constraint_idx, 2 * i : 2 * (i + 1)] = -2. * (self.robots_poses[:2, i] - self.robots_poses[:2, j])
                    Aqp[constraint_idx, 2 * j : 2 * (j + 1)] = 2. * (self.robots_poses[:2, i] - self.robots_poses[:2, j])
                    bqp[constraint_idx] = self.GAMMA * (np.linalg.norm(self.robots_poses[:2, i] - self.robots_poses[:2, j]) ** 2 - self.SAFETY_RADIUS ** 2)
                    constraint_idx = constraint_idx + 1
            u = solve_qp(2 * self.P, q.T, Aqp, bqp, solver='osqp')
            robots_speeds[:2, :] = u.reshape((self.N, 2)).T
        return robots_speeds

    def set_robots_speeds_and_grippers_powers(self, robots_speeds, robots_grippers_powers, local_frame=False):
        # when local_frame is True and self.SAFETY_LAYER is False, suboptimal transformation from local-to-global-to-local
        if local_frame is True:
            robots_speeds = transform_velocity_local_to_global(robots_speeds, self.robots_poses[2, :])
        robots_speeds = self.__safe_and_max_speeds(robots_speeds)
        robots_speeds = transform_velocity_global_to_local(robots_speeds, self.robots_poses[2, :])
        for i in range(self.N):
            self.robots[i].set_mobile_base_speed_and_gripper_power(robots_speeds[0, i], robots_speeds[1, i], robots_speeds[2, i], robots_grippers_powers[i])

    def set_robots_arms_poses(self, robots_arms_poses, wait_s=2.6):
        for i in range(self.N):
            self.robots[i].set_arm_pose(self, robots_arms_poses[0, i], robots_arms_poses[1, i], wait_s=2.6)

    def set_leds(self, robots_leds):
        for i in range(self.N):
            self.robots[i].set_leds(int(robots_leds[0, i]), int(robots_leds[1, i]), int(robots_leds[2, i]))
    
    def get_robots_poses(self, check_all_poses_received=False):
        all_poses_received = True
        for i in range(self.N):
            qi = self.robots[i].get_pose()
            if qi:
                self.robots_poses[0, i] = qi[0]
                self.robots_poses[1, i] = qi[1]
                self.robots_poses[2, i] = np.arctan2(np.sin(qi[2]), np.cos(qi[2]))
                all_poses_received &= True
            else:
                all_poses_received &= False
        if check_all_poses_received:
            return self.robots_poses, all_poses_received
        else:
            return self.robots_poses
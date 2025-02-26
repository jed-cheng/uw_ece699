import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from qpsolvers import solve_qp
import time
from src.geometry_utils import transform_velocity_local_to_global

class MultiDJIRoboMasterEPSim:
    def __init__(self, robot_ids, backend_server_ip=None, initial_robots_poses=None, safety_layer=True, safety_radius=None):
        # constants
        # robots
        self.ROBOT_IDS = robot_ids
        self.N = len(self.ROBOT_IDS)
        # time
        self.TIMEOUT_SET_MOBILE_BASE_SPEED = 50 # milliseconds (RobotHardware frequency set to 20Hz)
        self.TIMEOUT_GET_POSES = 10 # milliseconds (Vicon frequency set to 100Hz)
        self.DT = (self.TIMEOUT_SET_MOBILE_BASE_SPEED + self.TIMEOUT_GET_POSES) / 1000.
        # robot control
        self.MAX_LINEAR_SPEED = 5.0 # meters / second
        self.MAX_ANGULAR_SPEED = 2 * np.pi # degrees / second
        self.SAFETY_LAYER = safety_layer
        self.GAMMA = 10.
        self.P = np.eye(2 * self.N)
        # dimensions
        # TODO: use environment size (or rather corners, to stay implement "wall barriers")
        self.ENV_SIZE = 4 # x,y can vary from -ENV_SIZE/2 to ENV_SIZE/2
        self.ROBOT_SIZE = [0.24, 0.32] # [w, l]
        if safety_radius is None:
            self.SAFETY_RADIUS = max(self.ROBOT_SIZE) * 2.2
        else:
            self.SAFETY_RADIUS = safety_radius
        self.GRIPPER_SIZE = 0.1
        self.INIT_POSES_THRESH = 0.5

        # initialize positions of the robots
        self.robots_poses = np.vstack((-self.ENV_SIZE / 2.0 + self.ENV_SIZE * np.random.random((2, self.N)), 2 * np.pi * np.random.random((1, self.N))))                    

        # init plot
        self.figure = []
        self.axes = []
        self.patches_robots = []
        self.patches_grippers = []
        self.text_ids = []
        self.__init_plot()

        if initial_robots_poses is not None:
            print('Initializing robots to specified initial poses ...')
            while np.linalg.norm(initial_robots_poses - self.robots_poses) > self.INIT_POSES_THRESH:
                robots_speeds = 10. * (initial_robots_poses - self.robots_poses)
                self.set_robots_speeds_and_grippers_powers(robots_speeds, np.zeros((1, self.N)))
            self.set_leds(np.tile(np.array([[173], [216], [230]], dtype=int), (1, self.N)))
            print('... done (LEDs should be blue now)')
            time.sleep(1.)
    
    def __init_plot(self):
        self.figure, self.axes = plt.subplots()
        p_env = patches.Rectangle(np.array([-self.ENV_SIZE / 2.0, -self.ENV_SIZE / 2.0]), self.ENV_SIZE, self.ENV_SIZE, fill=False)
        self.axes.add_patch(p_env)

        for i in range(self.N):
            R = np.array([[0.0, 1.0], [-1.0, 0.0]]) @ np.array([[np.cos(self.robots_poses[2, i]), -np.sin(self.robots_poses[2, i])], [np.sin(self.robots_poses[2, i]), np.cos(self.robots_poses[2, i])]])
            t = np.array([self.robots_poses[0, i], self.robots_poses[1, i]])
            p_robot = patches.Polygon(t + np.array([0, self.ROBOT_SIZE[1]]) @ R.T / 2.0 + (np.array([[-self.ROBOT_SIZE[0] / 2.0, 0.0], [self.ROBOT_SIZE[0] / 2.0, 0.0], [self.ROBOT_SIZE[0] / 2.0, self.ROBOT_SIZE[1]], [-self.ROBOT_SIZE[0] / 2.0, self.ROBOT_SIZE[1]]]) - np.array([0, self.ROBOT_SIZE[1]]) / 2.0) @ R.T, facecolor='k')
            p_gripper = patches.Polygon(t + np.array([0, self.ROBOT_SIZE[1]]) @ R.T / 2.0 + (np.array([0, self.ROBOT_SIZE[1]]) + np.array([[-self.GRIPPER_SIZE / 2.0, 0.0], [self.GRIPPER_SIZE / 2.0, 0.0], [self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [0.8 * self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [0.8 * self.GRIPPER_SIZE / 2.0, 0.0], [-0.8 * self.GRIPPER_SIZE / 2.0, 0.0], [-0.8 * self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [-self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [-self.GRIPPER_SIZE / 2.0, 0.0]]) - np.array([0, self.ROBOT_SIZE[1]]) / 2.0) @ R.T, facecolor='k')
            text_id = plt.text(self.robots_poses[0, i], self.robots_poses[1, i], str(self.ROBOT_IDS[i]))
            self.patches_robots.append(p_robot)
            self.patches_grippers.append(p_gripper)
            self.text_ids.append(text_id)
            self.axes.add_patch(p_robot)
            self.axes.add_patch(p_gripper)
        
        self.axes.set_xlim(-0.55*self.ENV_SIZE, 0.55*self.ENV_SIZE)
        self.axes.set_ylim(-0.55*self.ENV_SIZE, 0.55*self.ENV_SIZE)
        # self.axes.set_axis_off()
        self.axes.axis('equal')

        plt.ion()
        plt.show()

    def __update_plot(self):
        for i in range(self.N):
            R = np.array([[0.0, 1.0], [-1.0, 0.0]]) @ np.array([[np.cos(self.robots_poses[2, i]), -np.sin(self.robots_poses[2, i])], [np.sin(self.robots_poses[2, i]), np.cos(self.robots_poses[2, i])]])
            t = np.array([self.robots_poses[0, i], self.robots_poses[1, i]])
            xy_robot = t + np.array([0, self.ROBOT_SIZE[1]]) @ R.T / 2.0 + (np.array([[-self.ROBOT_SIZE[0] / 2.0, 0.0], [self.ROBOT_SIZE[0] / 2.0, 0.0], [self.ROBOT_SIZE[0] / 2.0, self.ROBOT_SIZE[1]], [-self.ROBOT_SIZE[0] / 2.0, self.ROBOT_SIZE[1]]]) - np.array([0, self.ROBOT_SIZE[1]]) / 2.0) @ R.T
            xy_gripper = t + np.array([0, self.ROBOT_SIZE[1]]) @ R.T / 2.0 + (np.array([0, self.ROBOT_SIZE[1]]) + np.array([[-self.GRIPPER_SIZE / 2.0, 0.0], [self.GRIPPER_SIZE / 2.0, 0.0], [self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [0.8 * self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [0.8 * self.GRIPPER_SIZE / 2.0, 0.0], [-0.8 * self.GRIPPER_SIZE / 2.0, 0.0], [-0.8 * self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [-self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [-self.GRIPPER_SIZE / 2.0, 0.0]]) - np.array([0, self.ROBOT_SIZE[1]]) / 2.0) @ R.T
        
            self.patches_robots[i].xy = xy_robot
            self.patches_grippers[i].xy = xy_gripper
            self.text_ids[i].set_position((self.robots_poses[0, i], self.robots_poses[1, i]))

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

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
        if local_frame is True:
            robots_speeds = transform_velocity_local_to_global(robots_speeds, self.robots_poses[2, :])
        robots_speeds = self.__safe_and_max_speeds(robots_speeds)
        # for the simulator we don't need to transform back to local since we're integrating directly in the global frame
        for i in range(self.N):
            self.robots_poses[0, i] = self.robots_poses[0, i] + robots_speeds[0, i] * self.DT
            self.robots_poses[1, i] = self.robots_poses[1, i] + robots_speeds[1, i] * self.DT
            self.robots_poses[2, i] = self.robots_poses[2, i] + robots_speeds[2, i] * self.DT

        # grippers motion not implemented

        # update plot
        self.__update_plot()

    def set_robots_arms_poses(self, robots_arms_poses, wait_s=2.6):
        # arms motion not implemented
        pass

    def set_leds(self, robots_leds):
        for i in range(self.N):
            self.patches_robots[i].set_facecolor(robots_leds[:, i] / 255.)
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
    
    def get_robots_poses(self):
        return self.robots_poses
import time
import pybullet as p
import pybullet_data
import numpy as np
import franka_control as fc
import robot_helper as rh
import pybullet_tools.utils as pb_utils

FIXED_ROTATION = (1, 0, 0, 0)
MOVABLE_JOINT_NUMBERS = [0,1,2,3,4,5,6]
BOARD_DIMS = np.array((0.381, 0.304))


class Environment:
    def __init__(self, franka, sim_id):
        self.franka = franka
        self.sim_id = sim_id
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=self.sim_id)
        self.board = p.loadURDF("./URDFs/Chopping Board/urdf/Chopping Board.urdf",0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)
        xs, ys = np.random.choice(np.linspace(-0.33/2, 0.33/2, 2000), 100, replace=False), np.random.choice(np.linspace(-0.25/2, 0.25/2, 2000), 100, replace=False)
        # colors = ['red', 'blue', 'orange', 'yellow', 'green', 'light_green']
        # colors = ['green', 'orange']
        colors = ['green']
        for i in range(70):
            x, y, z = xs[i], ys[i], 0.055
            color = np.random.choice(colors)
            objects = [p.loadURDF(f"./URDFs/disc_{color}.urdf", x, y, z, 0, 0, 0, 1)]
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=self.sim_id)
            
    # def go_to_position(self, pos):
    #     desired_joints = rh.inverse_kinematics(self.franka, pos, FIXED_ROTATION)
    #     helper.control_joints(self.franka, MOVABLE_JOINT_NUMBERS, desired_joints)
    
    def sweep_over_chopping_board(self):
        board_pos = rh.get_object_position(self.board, self.sim_id)
        # board_pos[2] += 0.01
        board_pos[:2] -= BOARD_DIMS/2
        return board_pos
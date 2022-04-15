import pybullet as p
import pybullet_tools.utils as pb_utils
import pybullet_data
import numpy as np
import time
import robot_helper as helper
############ GLOBAL VARIABLES #################
BOARD_DIMS = np.array((0.381, 0.304))
FIXED_ROTATION = [1,0,0,0]
MOVABLE_JOINT_NUMBERS = range(7)

pb_utils.connect(use_gui=True)
pb_utils.add_data_path()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.resetSimulation()
p.setTimeStep(1/500, physicsClientId=pb_utils.CLIENT)
p.setPhysicsEngineParameter(solverResidualThreshold=0, physicsClientId=pb_utils.CLIENT)
pb_utils.set_camera(20, -30, 1)
# colors = ['red', 'blue', 'orange', 'yellow', 'green', 'light_green']
colors = ['green', 'orange']

objects = p.loadURDF("plane.urdf", basePosition =[ 0.000000, 0.000000, 0.000000], baseOrientation = [0.000000, 0.000000, 0.000000, 1.000000])
with pb_utils.LockRenderer():
    franka = p.loadURDF('./assets/franka_description/robots/franka_panda.urdf', basePosition =[ -0.4, 0, 0.000000], baseOrientation = [0.000000, 0.000000, 0.000000, 1.000000], useFixedBase=True, globalScaling=1)
    cutting_board = p.loadURDF("./URDFs/Chopping Board/urdf/Chopping Board.urdf", 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)
    xs, ys = np.random.choice(np.linspace(-0.381/2,0.381/2,500), 100, replace=False),np.random.choice(np.linspace(-0.304/2,0.304/2,500), 100, replace=False)
    for i in range(100):
        x, y, z = xs[i], ys[i], 0.1
        color = np.random.choice(colors)
        objects = [p.loadURDF(f"./URDFs/disc_{color}.urdf", x, y, z, 0, 0, 0, 1)]
helper.set_robot_to_reasonable_position(franka)
BLACK = (0.4,0.4,0.4,1)
for i in range(-1,10):
    if i%2==0:
        pb_utils.set_color(franka, link=i, color=BLACK)
    else:
        # color_robot_part(my_robot, i,RED)
        pass

# knife = p.loadURDF("./URDFs/knife.urdf", -0.1, -0.1, 0.05, 0.000000, 0.000000, 0.000000, 1.000000)

def go_to_position(robot, pos):
    desired_joints = helper.inverse_kinematics(robot, pos, FIXED_ROTATION)
    helper.control_joints(robot, MOVABLE_JOINT_NUMBERS, desired_joints)

def sweep_over_chopping_board(robot, board):
    board_pos = helper.get_object_position(board)
    board_pos[2] += 0.02
    board_pos[:2] -= BOARD_DIMS/2
    go_to_position(robot, board_pos)




if __name__=="__main__":
    p.setGravity(0, 0, -9.81)
    # p.setRealTimeSimulation(1)
    # ref_time = time.time()
    running_time = 60  # seconds
    sweep_over_chopping_board(franka, cutting_board)
    while (time.time() < ref_time + running_time):
        # p.setGravity(0, 0, -9.81)
        p.stepSimulation()
    p.disconnect()
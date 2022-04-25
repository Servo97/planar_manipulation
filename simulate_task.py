import pybullet as p
import pybullet_tools.utils as pb_utils
import pybullet_data
import numpy as np
import time
import robot_helper as helper
from scipy.spatial.transform import Rotation as R
############ GLOBAL VARIABLES #################
BOARD_DIMS = np.array((0.381, 0.304))
FIXED_ROTATION = [1, 0, 0, 0]
MOVABLE_JOINT_NUMBERS = range(7)
VIEWMATRIX = p.computeViewMatrix(
    cameraEyePosition=[0, 0, 0.6],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 1, 0])

PROJECTIONMATRIX = p.computeProjectionMatrixFOV(
    fov=45,
    aspect=helper.WIDTH / helper.HEIGHT,
    nearVal=0.02,
    farVal=1)

# p.resetSimulation(pb_utils.CLIENT)
pb_utils.connect(use_gui=True)
# p.setRealTimeSimulation(1)
pb_utils.add_data_path()
p.setGravity(0, 0, -9.81)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,1)
p.setTimeStep(1/1000, physicsClientId=pb_utils.CLIENT)
p.setPhysicsEngineParameter(
    solverResidualThreshold=0, physicsClientId=pb_utils.CLIENT)
pb_utils.set_camera(90, -89, 1)
# colors = ['red', 'blue', 'orange', 'yellow', 'green', 'light_green']
# colors = ['green', 'orange']
colors = ['green']

objects = p.loadURDF("plane.urdf", basePosition=[
                     0.000000, 0.000000, 0.000000], baseOrientation=[0.000000, 0.000000, 0.000000, 1.000000])
with pb_utils.LockRenderer():
    franka = p.loadURDF('./assets/franka_description/robots/franka_panda.urdf', basePosition=[-0.4, 0, 0.000000], baseOrientation=[
                        0.000000, 0.000000, 0.000000, 1.000000], useFixedBase=True, globalScaling=1)
    pb_utils.set_dynamics(franka, 8, linearDamping=0, lateralFriction=1)
    pb_utils.set_dynamics(franka, 9, linearDamping=0, lateralFriction=1)
    cutting_board = p.loadURDF("./URDFs/Chopping Board/urdf/Chopping Board.urdf", basePosition=[
                     0.000000, 0.000000, 0.025], baseOrientation=[0.000000, 0.000000, 0.000000, 1.000000], useFixedBase=True)
    xs, ys = np.random.choice(np.linspace(-0.33/2, 0.33/2, 2000), 100,
                              replace=False), np.random.choice(np.linspace(-0.25/2, 0.25/2, 2000), 100, replace=False)
    for i in range(70):
        x, y, z = xs[i], ys[i], 0.055
        color = np.random.choice(colors)
        objects = [p.loadURDF(
            f"./URDFs/disc_{color}.urdf", x, y, z, 0, 0, 0, 1)]
helper.set_robot_to_reasonable_position(franka)
BLACK = (0.4, 0.4, 0.4, 1)
for i in range(-1, 10):
    if i % 2 == 0:
        pb_utils.set_color(franka, link=i, color=BLACK)
    else:
        # color_robot_part(my_robot, i,RED)
        pass

# knife = p.loadURDF("./URDFs/knife.urdf", -0.1, -0.1, 0.05, 0.000000, 0.000000, 0.000000, 1.000000)


def go_to_position(robot, pos, rot = FIXED_ROTATION):
    print(pos, rot)
    desired_joints = helper.inverse_kinematics(robot, pos, rot)
    helper.control_joints(robot, MOVABLE_JOINT_NUMBERS, desired_joints, velocity_scale=1)

def initialize_robot_arm(robot, board):
    board_pos = helper.get_obj_com_position(board)
    board_pos[2] += 0.1
    board_pos[:2] -= BOARD_DIMS/2
    go_to_position(robot, board_pos)

def sweep_over_chopping_board(robot, x, y, theta, l, z = 0.02):
    theta = theta/180*np.pi
    r = R.from_euler('y', theta)
    rotation = r.as_quat()
    # rotation = rotation[1:]+rotation[:1]
    print(rotation)
    go_to_position(robot, [x,y,0.1], FIXED_ROTATION)
    time.sleep(0.5)
    go_to_position(robot, [x,y,z], FIXED_ROTATION)
    time.sleep(0.5)
    go_to_position(robot, [x+np.cos(theta)*l, y+np.sin(theta)*l, z], FIXED_ROTATION)
    time.sleep(0.5)
    go_to_position(robot, [x+np.cos(theta)*l, y+np.sin(theta)*l, 0.1], FIXED_ROTATION)

def get_outta_the_way(robot, board):
    board_pos = helper.get_obj_com_position(board)
    board_pos[2] += 0.1
    board_pos[:2] -= BOARD_DIMS/2
    go_to_position(robot, board_pos)


amount_to_move = 0.05  # 5cm
# FIXED_ROTATION = (0, 0, 0, 1)
MOVABLE_JOINT_NUMBERS = [0, 1, 2, 3, 4, 5, 6]
pos = helper.get_gripper_position(franka)
while True:
    initialize_robot_arm(franka, cutting_board)
    # Initialize Click Image
    imgs = p.getCameraImage(width=helper.WIDTH,height=helper.HEIGHT,viewMatrix=VIEWMATRIX,projectionMatrix=PROJECTIONMATRIX, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    helper.plot_images(imgs)
    # Sample possible starting point and orientation of gripper
    # Move gripper by random distance l
    x, y, theta, l = -0.01, -0.01, 90, 0.15
    sweep_over_chopping_board(franka, x, y, theta, l, z = 0.02)

    # Take arm out of sight and click picture
    get_outta_the_way(franka, cutting_board)
    imgs = p.getCameraImage(width=helper.WIDTH,height=helper.HEIGHT,viewMatrix=VIEWMATRIX,projectionMatrix=PROJECTIONMATRIX, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    helper.plot_images(imgs)

    p.resetSimulation(pb_utils.CLIENT)
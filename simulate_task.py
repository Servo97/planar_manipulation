import pybullet as p
import pybullet_tools.utils as pb_utils
import pybullet_data
import numpy as np
import time
import robot_helper as helper
############ GLOBAL VARIABLES #################
BOARD_DIMS = np.array((0.381, 0.304))
FIXED_ROTATION = [1, 0, 0, 0]
MOVABLE_JOINT_NUMBERS = range(7)

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


def go_to_position(robot, pos):
    desired_joints = helper.inverse_kinematics(robot, pos, FIXED_ROTATION)
    helper.control_joints(robot, MOVABLE_JOINT_NUMBERS, desired_joints, velocity_scale=1)


def sweep_over_chopping_board(robot, board):
    board_pos = helper.get_obj_com_position(board)
    board_pos[2] += 0.02
    board_pos[:2] -= BOARD_DIMS/2
    go_to_position(robot, board_pos)


    

sweep_over_chopping_board(franka, cutting_board)


amount_to_move = 0.05  # 5cm
FIXED_ROTATION = (0, 0, 0, 1)
MOVABLE_JOINT_NUMBERS = [0, 1, 2, 3, 4, 5, 6]
pos = helper.get_gripper_position(franka)
while True:
    imgs = p.getCameraImage(width=helper.WIDTH,height=helper.HEIGHT,viewMatrix=viewMatrix,projectionMatrix=projectionMatrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    # key_pressed = helper.wait_and_get_pressed_key()
    # if key_pressed == "w":
    #     print("Moving backward")
    #     # TODO move robot 5cm backward (in the negative x direction)
    #     pos[0] -= amount_to_move
    # elif key_pressed == "s":
    #     print("Moving forward")
    #     # TODO move robot 5cm forward (in the positive x direction)
    #     pos[0] += amount_to_move
    # elif key_pressed == "d":
    #     print("Moving robot to the right")
    #     # TODO move robot 5cm to the right (positive y direction)
    #     pos[1] += amount_to_move
    # elif key_pressed == "a":
    #     print("Moving robot to the left")
    #     # TODO move robot 5cm to the left (negative y direction)
    #     pos[1] -= amount_to_move
    # elif key_pressed == "q":
    #     print("Moving robot up")
    #     # TODO move robot 5cm up (positive z direction)
    #     pos[2] += amount_to_move
    # elif key_pressed == "e":
    #     print("Moving robot down")
    #     # TODO move robot 5cm down (negative z direction)
    #     pos[2] -= amount_to_move
    # elif key_pressed == "o":
    #     helper.plot_images(imgs)

    # desired_joints = helper.inverse_kinematics(franka, pos, FIXED_ROTATION)
    # helper.control_joints(franka, [0, 1, 2, 3, 4, 5, 6], desired_joints)

    while True:
        p.stepSimulation(pb_utils.CLIENT)
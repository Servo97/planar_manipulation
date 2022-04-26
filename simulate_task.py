import pybullet as p
import pybullet_tools.utils as pb_utils
import pybullet_data
import numpy as np
import time
import robot_helper as helper
from scipy.spatial.transform import Rotation as R
import torch
import matplotlib.pyplot as plt
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
N_PARTICLES = 150

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
    franka = p.loadURDF('./assets/franka_description/robots/franka_panda.urdf', basePosition=[-0.5, 0, 0.000000], baseOrientation=[
                        0.000000, 0.000000, 0.000000, 1.000000], useFixedBase=True, globalScaling=1)
    pb_utils.set_dynamics(franka, 8, linearDamping=0, lateralFriction=1)
    pb_utils.set_dynamics(franka, 9, linearDamping=0, lateralFriction=1)
    cutting_board = p.loadURDF("./URDFs/Chopping Board/urdf/Chopping Board.urdf", basePosition=[
                     0.000000, 0.000000, 0.025], baseOrientation=[0.000000, 0.000000, 0.000000, 1.000000], useFixedBase=True)
    xs, ys = np.random.choice(np.linspace(-0.28/2, 0.38/2, 2000), N_PARTICLES,
                              replace=False), np.random.choice(np.linspace(-0.25/2, 0.25/2, 2000), N_PARTICLES, replace=False)
    for i in range(N_PARTICLES):
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

# def sweep_over_chopping_board(robot, x, y, theta, l, z = 0.0535):
def sweep_over_chopping_board(robot, y,x, theta, y1, x1, z = 0.0535):
    go_to_position(robot, [x,y,0.1], FIXED_ROTATION)
    time.sleep(0.5)
    ori = rotate_gripper(robot, [x,y,0.1], theta+np.pi/2)
    time.sleep(0.5)
    go_to_position(robot, [x,y,z], ori)
    time.sleep(0.5)
    go_to_position(robot, [x1, y1, z], ori)
    time.sleep(0.5)
    go_to_position(robot, [x1, y1, 0.2], ori)
    time.sleep(0.5)

def get_outta_the_way(robot, board):
    board_pos = helper.get_obj_com_position(board)
    board_pos[2] += 0.1
    board_pos[:2] -= BOARD_DIMS/2
    go_to_position(robot, board_pos)

def rotate_gripper(robot, pos, theta):
    ori = helper.get_quat(theta)
    go_to_position(robot, pos, ori)
    return ori
    
def get_params():
    x = np.random.uniform(-BOARD_DIMS[0]/2, BOARD_DIMS[0]/2)
    y = np.random.uniform(-BOARD_DIMS[1]/2, BOARD_DIMS[1]/2)
    theta = np.random.normal(0, 30)
    l = np.random.normal(0,0.1)
    return (x,y,theta,l)

amount_to_move = 0.05  # 5cm
# FIXED_ROTATION = (0, 0, 0, 1)
MOVABLE_JOINT_NUMBERS = [0, 1, 2, 3, 4, 5, 6]
pos, ori = helper.get_gripper_position(franka)



# while True:
if __name__=="__main__":
    time.sleep(0.5)
    # rotate_gripper(franka, 30)
    initialize_robot_arm(franka, cutting_board)
    # Initialize Click Image
    imgs = p.getCameraImage(width=helper.WIDTH,height=helper.HEIGHT,viewMatrix=VIEWMATRIX,projectionMatrix=PROJECTIONMATRIX, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    # helper.plot_images(imgs, 'gray_before')
    
    """ Import Switched Dynamics """
    iteration = 0
    best_params = []
    torch.set_grad_enabled(False)
    dynamics = helper.ObjectCentricTransport(torch.Tensor(helper.get_board(imgs, iteration)))

    # dynamics.board = dynamics.board.T.to(dynamics.device)
    curr_lyp_score = dynamics.lyapunov_function(dynamics.board)
    # print("HAKUNA")
    while curr_lyp_score > 0:
        iteration += 1
        best_board = None
        best_lyp_score = curr_lyp_score
        # print("HAKUNA1")
        # This is exhaustive search -> Needs to be replaced with BO for faster performance
        for x in np.linspace(-BOARD_DIMS[0]/2, BOARD_DIMS[0]/2,20):
            for y in np.linspace(-BOARD_DIMS[1]/2, BOARD_DIMS[1]/2,20):
                for theta in [0., np.pi/2, np.pi, -np.pi/2]:
                # for theta in [0.]:
                    # for move_distance in [-0.095, 0.095]: # np.linspace(2,32,5):
                    for move_distance in [10]: # np.linspace(2,32,5):
                        x1, y1 = helper.mtr_to_pix(x,y)
                        board, lyp_score = dynamics.step(x1,y1, theta, 10, dynamics.board)
                        if lyp_score < curr_lyp_score and lyp_score < best_lyp_score:
                            print(x1, y1)
                            best_board = board
                            best_lyp_score = lyp_score
                            best_params_temp = [x1, y1, theta, move_distance]
                            best_params = [x,y, theta, x+0.095*np.cos(theta), y+0.095*np.sin(theta)]
        print(best_lyp_score, curr_lyp_score)
        if best_lyp_score >= curr_lyp_score or iteration >= 40:
            break
        
        fig, ax = plt.subplots(1,2)
        dynamics.board = 1.0 * (torch.rand(dynamics.board_shape[0], dynamics.board_shape[1]) > 0.01).to(dynamics.device)
        bb, _ = dynamics.step(*best_params_temp, dynamics.board)
        ax[0].imshow(dynamics.board.cpu().numpy())
        ax[1].imshow(bb.cpu().numpy())
        plt.show()
        # initialize_robot_arm(franka, cutting_board)
        # time.sleep(0.5)
        print("HAKUNA2", best_params)
        sweep_over_chopping_board(franka, *best_params, z = 0.052)
        # Take arm out of sight and click picture
        get_outta_the_way(franka, cutting_board)
        imgs = p.getCameraImage(width=helper.WIDTH,height=helper.HEIGHT,viewMatrix=VIEWMATRIX,projectionMatrix=PROJECTIONMATRIX, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        dynamics.board = torch.Tensor(helper.get_board(imgs, iteration))
        dynamics.board = dynamics.board.T.to(dynamics.device)
        # This is for creating GIF
        # rend.append((255*best_board.cpu().detach().numpy()).astype(np.uint8))
        curr_lyp_score = best_lyp_score
        print("Step #{}: ".format(iteration), best_lyp_score)
    # print("HAKUNA3")
    # Sample possible starting point and orientation of gripper
    # Move gripper by random distance l
    # sweep_over_chopping_board(franka, *get_params(), z = 0.0535)

    # Take arm out of sight and click picture
    get_outta_the_way(franka, cutting_board)
    imgs = p.getCameraImage(width=helper.WIDTH,height=helper.HEIGHT,viewMatrix=VIEWMATRIX,projectionMatrix=PROJECTIONMATRIX, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    helper.plot_images(imgs, 'gray_after')

    # p.resetSimulation(pb_utils.CLIENT)
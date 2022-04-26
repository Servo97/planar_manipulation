import pybullet as p
import pybullet_tools.utils as pb_utils
import pybullet_data
import numpy as np
import time
from collections import namedtuple
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics
import matplotlib.pyplot as plt
import cv2
from glob import glob
from scipy.spatial.transform import Rotation as R
import torch
import copy

WIDTH = 225
HEIGHT = 225
KERNEL = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))


def make_robot(robot_filename):
    pb_utils.connect(use_gui=True)
    pb_utils.add_data_path()
    pb_utils.load_pybullet("plane.urdf")
    # pb_utils.set_real_time(False)
    p.setTimeStep(1/500, physicsClientId=pb_utils.CLIENT)
    p.setPhysicsEngineParameter(solverResidualThreshold=0, physicsClientId=pb_utils.CLIENT)
    with pb_utils.LockRenderer():
        # robot_idx = pb_utils.load_pybullet(robot_filename, fixed_base=True,basePosition=[-0.4, 0, 0.000000], baseOrientation=[0.000000, 0.000000, 0.000000, 1.000000])
        robot_idx = pb_utils.load_pybullet(robot_filename, fixed_base=True)
    pb_utils.set_camera(80, -30, 2) #reasonable view for most things
    set_robot_to_reasonable_position(robot_idx)
    #pb_utils.set_dynamics(robot_idx, 9, linearDamping=0, angularDamping=0, lateralFriction=1)
    pb_utils.set_dynamics(robot_idx, 8, linearDamping=0, lateralFriction=1)
    pb_utils.set_dynamics(robot_idx, 9, linearDamping=0,lateralFriction=1)
    return robot_idx

def set_robot_to_reasonable_position(my_robot):
    reasonable_joint_numbers = list(range(0,7))
    reasonable_joint_positions = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]
    pb_utils.set_joint_positions(my_robot, reasonable_joint_numbers, reasonable_joint_positions)

def inverse_kinematics(object_index, position, rotation):
    state = p.saveState()
    offset = 0.1
    position_up = (position[0], position[1], position[2]+offset)
    goal_ee_pose = (position_up, rotation)
    tool_link = 7
    IKFastInfo = namedtuple('IKFastInfo', ['module_name', 'base_link', 'ee_link', 'free_joints'])
    info = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='panda_link0', ee_link='panda_link7',
                      free_joints=['panda_joint6'])
    ik_joints = get_ik_joints(object_index, info, tool_link)
    pb_kwargs = {"pos_tolerance": 1e-3, "ori_tolerance": 3.14*1e-3, "max_attempts": 5,
                 "max_time": 500000000, "fixed_joints": []}
    if True: #with pb_utils.LockRenderer():
        conf = next(either_inverse_kinematics(object_index, info, tool_link, goal_ee_pose, use_pybullet=True, **pb_kwargs),None)
    p.restoreState(state)
    if conf is None:
        print("Error Position Is Out Of Franka's Workspace")
        # sys.exit()
        return None
    else:
        return [np.degrees(a) for a in conf]

def get_quat(theta):
    # theta = theta/180*np.pi
    r = R.from_euler('x', theta)
    rotation = list(r.as_quat())
    rotation[:] = rotation[-1:]+rotation[:-1]
    return rotation

def wait_simulate_for_duration(duration):
    dt = pb_utils.get_time_step()
    for i in range(int(np.ceil(duration / dt))):
        before = time.time()
        pb_utils.step_simulation()
        after = time.time()
        if after - before < dt:
            time.sleep(dt - (after - before))

def control_joint_positions(body, joints, positions, velocities=None, interpolate=10, time_to_run=0.5, verbose=False, **kwargs):
    if interpolate is not None:
        current_positions = pb_utils.get_joint_positions(body, joints)
        waypoints = np.linspace(current_positions, positions, num=interpolate)[1:]
        if verbose:
            print(f"current = {current_positions}, target = {positions}, waypoints = {waypoints}")
    else:
        waypoints = [positions]

    for pt in waypoints:
        if verbose:
            print(pt)
        pb_utils.control_joints(body, joints, pt, **kwargs)
        wait_simulate_for_duration(time_to_run / len(waypoints))

def control_joints(body, joints, positions, velocities=None, interpolate=10, **kwargs):
    control_joint_positions(body, joints, [np.radians(p) for p in positions], velocities, interpolate=interpolate, **kwargs)

def get_obj_com_position(object, sim_id = pb_utils.CLIENT):
    return np.array(pb_utils.get_com_pose(object, -1)[0])

def get_object_position(object, sim_id = pb_utils.CLIENT):
    return np.array(pb_utils.get_link_pose(object, -1)[0])

def get_gripper_position(robot):
    tool_link = 7
    return pb_utils.get_link_pose(robot, tool_link)


def wait_and_get_pressed_key():
    while True:
        keys = p.getKeyboardEvents()
        for (key, value) in keys.items():
            print("Value", value)
            if value&p.KEY_WAS_TRIGGERED:
                key_pressed = chr(key)
                return key_pressed

VIEWMATRIX = p.computeViewMatrix(
    cameraEyePosition=[0, 0, 0.6],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 1, 0])

PROJECTIONMATRIX = p.computeProjectionMatrixFOV(
    fov=45,
    aspect=WIDTH / HEIGHT,
    nearVal=0.02,
    farVal=1)
def plot_images(img, folder):
    img = np.reshape(img[2], (WIDTH, HEIGHT, 4))
    files = glob(f'./data/{folder}/*.jpg')
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img = img[35:185, ]
    cv2.imwrite(f'./data/{folder}/img_{len(files)}.jpg', img)
    # plt.imshow(img, cmap='gray', vmin=0, vmax=1)
    # plt.show()

def get_board(img, iteration):
    img = np.reshape(img[2], (WIDTH, HEIGHT, 4))
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img = img[35:185, ]
    cv2.imwrite(f'./data/output/img_{iteration}.jpg', img)
    img = cv2.resize(img, (96,64), interpolation = cv2.INTER_AREA)
    img = img[:,10:]
    _, out = cv2.threshold(img,110,255,cv2.THRESH_BINARY_INV)
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, KERNEL)
    return out

def mtr_to_pix(x,y):
    x = int((x + 0.304)*105.26315)
    y = int((y + 0.381)*112.86089)
    return x,y

class ObjectCentricTransport:

    def __init__(self, start_board = None):
        num_particles = 400 # approx

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.knife_half_length = 16
        if start_board is None:
            self.board_shape = np.array([64,96])
            board_size = self.board_shape[0] * self.board_shape[1]
            self.board = 1.0 * (torch.rand(self.board_shape[0], self.board_shape[1]) > 0.5*2*(board_size - num_particles)/board_size)
            self.board = self.board.to(self.device)
        else:
            self.board = start_board.to(self.device)
            self.board_shape = np.array([start_board.shape[0], start_board.shape[1]])
            print(self.board_shape)


    def step(self, x, y, theta, move_distance, curr_board):
        board = copy.deepcopy(curr_board)
        coords = torch.nonzero(board).to(self.device)
        R = torch.Tensor([[-np.sin(theta),-np.cos(theta)],[np.cos(theta),-np.sin(theta)]]).to(self.device)
        transformed_coords = coords.float() @ R
        apply_at = torch.Tensor([[x,y]]).to(self.device) @ R

        indices_of_interest = torch.logical_and((apply_at[0,0] + move_distance) > transformed_coords[:,0], transformed_coords[:,0]> apply_at[0,0])
        indices_of_interest = torch.logical_and(indices_of_interest, ((apply_at[0,1]+self.knife_half_length) > transformed_coords[:,1]))
        indices_of_interest = torch.logical_and(indices_of_interest, (transformed_coords[:,1]> (apply_at[0,1] - self.knife_half_length)))
        
        to_move = transformed_coords[indices_of_interest]
        to_zero = coords[indices_of_interest]
        board[to_zero[:,0], to_zero[:,1]] = 0.0

        # Adding Chi-Square noise, i.e. simply sum of 2 squared Gaussian random variables
        # TODO - Tune the variance of the gaussian to fit data from PyBullet
        to_move[:,0] = apply_at[0,0] + move_distance + (torch.randn_like(to_move[:,0])**2 + torch.randn_like(to_move[:,0])**2)/(2*5)
        to_move = (to_move@ R.T).round().long()

        indices_of_interest = torch.logical_and(to_move[:,0] >= 0, to_move[:,1] >= 0)
        indices_of_interest = torch.logical_and(indices_of_interest, to_move[:,0] < self.board_shape[0])
        indices_of_interest = torch.logical_and(indices_of_interest, to_move[:,1] < self.board_shape[1])
        to_move = to_move[indices_of_interest]
        # to_move = torch.maximum(to_move, torch.zeros_like(to_move).to(self.device))
        # to_move = torch.minimum(to_move, torch.Tensor([[board.shape[0]-1, board.shape[1]-1]]).repeat((to_move.shape[0],1)).to(self.device))
        # to_move = to_move.round().long()

        # occupied = to_move[board[to_move[:,0], to_move[:,1]] == 1.0]
        # board[to_move[:,0], to_move[:,1]][board[to_move[:,0], to_move[:,1]] == 0.0] = 1.0
        # for x,y in occupied:
        #     self.board_recursion(x, y, board)
        board[to_move[:,0], to_move[:,1]] = 1.0
        return board, self.lyapunov_function(board)
    
    def board_recursion(self, x,y, board):
        move = [[-1,-1], [-1,1], [1,-1], [1,1], [-1, 0], [0,-1], [1,0], [0,1]]
        if board[x,y] == 0.0:
            board[x,y] = 1.0
            return
        else:
            row = np.random.choice(8, 1)
            new_x, new_y = move[row[0]]
            self.board_recursion( min(x+new_x, board.shape[0]-1) , min(y+new_y, board.shape[1]-1), board)
    
    def lyapunov_function(self, board, target_set = "square", set_size = 10):
        assert target_set in ["circle", "square"]
        object_locs = torch.nonzero(board)
        vec_values = board[object_locs[:,0], object_locs[:,1]]
        if target_set == "circle":
            vec_distances = torch.clamp(torch.norm((object_locs - self.board_shape/2).float(), dim = 1) - set_size, min=0)
        elif target_set == "square":
            x_min, x_max = (self.board_shape[0]-set_size)/2, (self.board_shape[0]+set_size)/2
            y_min, y_max = (self.board_shape[1]-set_size)/2, (self.board_shape[1]+set_size)/2

            vec_distances = torch.logical_and(object_locs[:,0] < x_min, object_locs[:,1] < y_min) * torch.norm((object_locs - torch.Tensor([x_min, y_min]).to(self.device)).float(), dim = 1)
            vec_distances += torch.logical_and(object_locs[:,0] < x_min, object_locs[:,1] > y_max) * torch.norm((object_locs - torch.Tensor([x_min, y_max]).to(self.device)).float(), dim = 1)
            vec_distances += torch.logical_and(object_locs[:,0] > x_max, object_locs[:,1] > y_max) * torch.norm((object_locs - torch.Tensor([x_max, y_max]).to(self.device)).float(), dim = 1)
            vec_distances += torch.logical_and(object_locs[:,0] > x_max, object_locs[:,1] < y_min) * torch.norm((object_locs - torch.Tensor([x_max, y_min]).to(self.device)).float(), dim = 1)

            vec_distances += torch.logical_and(torch.logical_and(object_locs[:,0] >= x_min, object_locs[:,0] <= x_max), object_locs[:,1] >= y_max) * (object_locs[:,1] - y_max)
            vec_distances += torch.logical_and(torch.logical_and(object_locs[:,0] >= x_min, object_locs[:,0] <= x_max), object_locs[:,1] <= y_min) * (y_min - object_locs[:,1])
            vec_distances += torch.logical_and(torch.logical_and(object_locs[:,1] >= y_min, object_locs[:,1] <= y_max), object_locs[:,0] >= x_max) * (object_locs[:,0] - x_max)
            vec_distances += torch.logical_and(torch.logical_and(object_locs[:,1] >= y_min, object_locs[:,1] <= y_max), object_locs[:,0] <= x_min) * (x_min - object_locs[:,0])
            
        else:
            raise NotImplementedError
        
        # # For visualizing the distances that contribute to the lyapunov function
        # viz = torch.zeros_like(board)
        # viz[object_locs[:,0], object_locs[:,1]] = vec_distances
        # X, Y = np.meshgrid(range(self.board_shape[0]), range(self.board_shape[1]))
        # hf = plt.figure()
        # ha = hf.add_subplot(111, projection='3d')
        # ha.plot_surface(X, Y, viz.cpu().numpy())
        # plt.show()
        
        return (vec_distances @ vec_values).item()
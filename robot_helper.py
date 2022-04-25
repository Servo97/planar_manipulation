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

WIDTH = 225
HEIGHT = 225


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

def wait_simulate_for_duration(duration):
    dt = pb_utils.get_time_step()
    for i in range(int(np.ceil(duration / dt))):
        before = time.time()
        pb_utils.step_simulation()
        after = time.time()
        if after - before < dt:
            time.sleep(dt - (after - before))

def control_joint_positions(body, joints, positions, velocities=None, interpolate=10, time_to_run=1, verbose=False, **kwargs):
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


def get_object_position(object, sim_id):
    return np.array(pb_utils.get_link_pose(object, -1, sim_id)[0])

def get_gripper_position(robot):
    tool_link = 7
    return list(pb_utils.get_link_pose(robot, tool_link)[0])


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
def plot_images(img):
    img = np.reshape(img[2], (WIDTH, HEIGHT, 4))
    files = glob('./data/gray/*.jpg')
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img = img[35:185, ]
    cv2.imwrite(f'./data/gray/img_{len(files)}.jpg', img)
    # plt.imshow(img, cmap='gray', vmin=0, vmax=1)
    # plt.show()
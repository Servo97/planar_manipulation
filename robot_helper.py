import pybullet as p
import pybullet_tools.utils as pb_utils
import pybullet_data
import numpy as np
import time
from collections import namedtuple
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics

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
        sys.exit()
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


def get_object_position(object):
    return np.array(pb_utils.get_link_pose(object, -1)[0])
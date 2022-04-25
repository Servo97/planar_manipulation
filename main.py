import time
import pybullet as p
import pybullet_data
import numpy as np
import franka_control as fc
import robot_helper as rh
import env

# Global Vars
FIXED_ROTATION = (1, 0, 0, 0)
MOVABLE_JOINT_NUMBERS = [0,1,2,3,4,5,6]
SAMPLING_RATE = 1e-3  # 1000Hz sampling rate
# colors = ['red', 'blue', 'orange', 'yellow', 'green', 'light_green']
# colors = ['green', 'orange']
colors = ['green']

def connect_pb(use_gui=False):
    sim_id = p.connect(p.GUI)
    if use_gui:
        # p.COV_ENABLE_PLANAR_REFLECTION
        # set_preview(False)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, False, physicsClientId=sim_id) # TODO: does this matter?
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, False, physicsClientId=sim_id) # mouse moves meshes
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, False, physicsClientId=sim_id)
    p.setTimeStep(SAMPLING_RATE, physicsClientId=sim_id)
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(1, 90, -89, np.zeros(3), physicsClientId=sim_id)
    return sim_id


def go_to_position(franka, pos):
    desired_joints = franka.calculate_inverse_kinematics(pos, FIXED_ROTATION)
    franka.set_target_positions(desired_joints)

def main():
    sim_id = connect_pb(use_gui=True)
    franka = fc.Franka(sim_id)
    # for i in range(9):
    #     print(franka.get_joint_info(i))
    pos, orientation = franka.get_gripper_position()

    environment = env.Environment(franka.franka, franka.sim_id)
    board_pos = environment.sweep_over_chopping_board()
    # go_to_position(franka, board_pos)
    franka.control_joint_positions(board_pos)
    while True:
        p.stepSimulation(franka.sim_id)
    time.sleep(2)
    # while True:

    #     print(pos)
    #     desired_joints = franka.calculate_inverse_kinematics(pos, FIXED_ROTATION)
    #     # desired_joints = rh.inverse_kinematics(franka.franka, pos, FIXED_ROTATION)
    #     if desired_joints is not None:
    #         # rh.control_joints(franka.franka, [0, 1, 2, 3, 4, 5, 6], desired_joints)
    #         franka.set_target_positions(desired_joints)
    #     p.stepSimulation(franka.sim_id)
        # time.sleep(SAMPLING_RATE)

    # p.disconnect()
    print("Simulation end")

if __name__ == "__main__":
    main()














# franka = rh.make_robot('./assets/franka_description/robots/franka_panda.urdf')
# rh.set_robot_to_reasonable_position(franka)


# with pb_utils.LockRenderer():
#     pb_utils.set_camera(90, -89, 1)
#     cutting_board = p.loadURDF("./URDFs/Chopping Board/urdf/Chopping Board.urdf", 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)
#     xs, ys = np.random.choice(np.linspace(-0.33/2, 0.33/2, 2000), 100, replace=False), np.random.choice(np.linspace(-0.25/2, 0.25/2, 2000), 100, replace=False)
#     for i in range(70):
#         x, y, z = xs[i], ys[i], 0.055
#         color = np.random.choice(colors)
#         objects = [p.loadURDF(
#             f"./URDFs/disc_{color}.urdf", x, y, z, 0, 0, 0, 1)]

# amount_to_move = 0.05 #5cm
# FIXED_ROTATION = (1,0,0,0)
# MOVABLE_JOINT_NUMBERS = [0,1,2,3,4,5,6]
# pos = rh.get_gripper_position(franka)
# while True:
    # imgs = p.getCameraImage(width=rh.WIDTH,height=rh.HEIGHT,viewMatrix=rh.VIEWMATRIX,projectionMatrix=rh.PROJECTIONMATRIX, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    # key_pressed = rh.wait_and_get_pressed_key()
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
    #     rh.plot_images(imgs)
    # elif key_pressed=="0":
    #     break

#     desired_joints = rh.inverse_kinematics(franka,pos, FIXED_ROTATION)
#     rh.control_joints(franka, [0,1,2,3,4,5,6], desired_joints, interpolate=30)

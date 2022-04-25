import os
import time
import numpy as np
import pybullet as p
from collections import defaultdict, deque, namedtuple

FIXED_ROTATION = (1, 0, 0, 0)
MOVABLE_JOINT_NUMBERS = [0,1,2,3,4,5,6]
SAMPLING_RATE = 5e-3  # 1000Hz sampling rate
class Franka:
    def __init__(self, sim_id):
        self.sim_id = sim_id
        self.franka = p.loadURDF('./assets/franka_description/robots/franka_panda.urdf', useFixedBase=True, basePosition=[-0.4, 0, 0.000000], globalScaling=1)
        # p.changeDynamics(bodyUniqueId=self.franka, linkIndex=0, maxJointVelocity=150 * (np.pi / 180))
        # p.changeDynamics(bodyUniqueId=self.franka, linkIndex=1, maxJointVelocity=150 * (np.pi / 180))
        # p.changeDynamics(bodyUniqueId=self.franka, linkIndex=2, maxJointVelocity=150 * (np.pi / 180))
        # p.changeDynamics(bodyUniqueId=self.franka, linkIndex=3, maxJointVelocity=150 * (np.pi / 180))
        # p.changeDynamics(bodyUniqueId=self.franka, linkIndex=4, maxJointVelocity=180 * (np.pi / 180))
        # p.changeDynamics(bodyUniqueId=self.franka, linkIndex=5, maxJointVelocity=180 * (np.pi / 180))
        # p.changeDynamics(bodyUniqueId=self.franka, linkIndex=6, maxJointVelocity=180 * (np.pi / 180))
        
        p.changeDynamics(self.franka, 8, physicsClientId=self.sim_id, linearDamping=0, lateralFriction=1)
        p.changeDynamics(self.franka, 9, physicsClientId=self.sim_id, linearDamping=0,lateralFriction=1)

        # Set DOF according to the fact that either gripper is supplied or not and create often used joint list
        self.dof = p.getNumJoints(self.franka) - 2
        self.joints = [0,1,2,3,4,5,6]
        # Reset Robot
        self.LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                     'localInertialFramePosition', 'localInertialFrameOrientation',
                                     'worldLinkFramePosition', 'worldLinkFrameOrientation'])
        self.reset_state()
        for i in range(-1,10):
            if i%2==0:
                p.changeVisualShape(self.franka, i, shapeIndex=-1, rgbaColor=(0.4, 0.4, 0.4, 1),physicsClientId=self.sim_id)
            else:
                pass

    
    def reset_state(self):
        """"""
        reasonable_joint_positions = [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]
        for j in range(self.dof):
            p.resetJointState(self.franka, j, targetValue=reasonable_joint_positions[j])
        p.setJointMotorControlArray(bodyUniqueId=self.franka,
                                    jointIndices=self.joints,
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=[0. for _ in self.joints])

    def get_joint_info(self, j):
        return p.getJointInfo(self.franka, j)

    def get_base_position_and_orientation(self):
        return p.getBasePositionAndOrientation(self.franka)

    def get_gripper_position(self):
        link_state = self.LinkState(*p.getLinkState(self.franka, 7, physicsClientId=self.sim_id))
        return link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation

        
    def get_joint_position_and_velocity(self):
        joint_states = p.getJointStates(self.franka, self.joints)
        joint_pos = [state[0] for state in joint_states]
        joint_vel = [state[1] for state in joint_states]
        return joint_pos, joint_vel

    def calculate_inverse_kinematics(self, position, orientation):
        return p.calculateInverseKinematics(self.franka, 7, position, orientation)

    # def calculate_inverse_dynamics(self, pos, vel, desired_acc):
    #     """"""
    #     assert len(pos) == len(vel) and len(vel) == len(desired_acc)
    #     vector_length = len(pos)

    #     # If robot set up with gripper, set those positions, velocities and desired accelerations to 0
    #     if self.dof == 9 and vector_length != 9:
    #         pos = pos + [0., 0.]
    #         vel = vel + [0., 0.]
    #         desired_acc = desired_acc + [0., 0.]

    #     simulated_torque = list(p.calculateInverseDynamics(self.franka, pos, vel, desired_acc))

    #     # Remove unnecessary simulated torques for gripper if robot set up with gripper
    #     if self.dof == 9 and vector_length != 9:
    #         simulated_torque = simulated_torque[:7]
    #     return simulated_torque

    def wait_simulate_for_duration(self, duration):
        dt = p.getPhysicsEngineParameters(physicsClientId=self.sim_id)['fixedTimeStep']
        for i in range(int(np.ceil(duration / dt))):
            before = time.time()
            p.stepSimulation(self.sim_id)
            after = time.time()
            if after - before < dt:
                time.sleep(dt - (after - before))

    def control_joint_positions(self, positions, interpolate=10, time_to_run=1,verbose=False):
        desired_joints = self.calculate_inverse_kinematics(positions, FIXED_ROTATION)
        if interpolate is not None:
            current_positions, _ = self.get_joint_position_and_velocity()
            print(current_positions)
            print(desired_joints)
            waypoints = np.linspace(current_positions, desired_joints, num=interpolate)[1:]
            if verbose:
                print(f"current = {current_positions}, target = {desired_joints}, waypoints = {waypoints}")
        else:
            waypoints = [desired_joints]

        for pt in waypoints:
            if verbose:
                print(pt)
            self.set_target_positions(pt)
            self.wait_simulate_for_duration(time_to_run / len(waypoints))


    def set_target_positions(self, desired_pos):
        """"""
        # If robot set up with gripper, set those positions to 0
        if self.dof == 9:
            desired_pos = desired_pos + [0., 0.]
        p.setJointMotorControlArray(bodyUniqueId=self.franka,
                                    jointIndices=self.joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=desired_pos,
                                    targetVelocities=[0.0] * len(desired_pos),
                                    physicsClientId=self.sim_id)

    def set_torques(self, desired_torque):
        """"""
        p.setJointMotorControlArray(bodyUniqueId=self.franka,
                                    jointIndices=self.joints,
                                    controlMode=p.TORQUE_CONTROL,
                                    forces=desired_torque,
                                    physicsClientId=self.sim_id)
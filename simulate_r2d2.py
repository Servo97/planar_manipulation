import pybullet as p
import pybullet_tools.utils as pb_utils
import pybullet_data
import numpy as np
import time

pb_utils.connect(use_gui=True)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setTimeStep(1/500, physicsClientId=pb_utils.CLIENT)
p.setPhysicsEngineParameter(solverResidualThreshold=0, physicsClientId=pb_utils.CLIENT)

colors = ['red', 'blue', 'orange', 'yellow', 'green', 'light_green']

objects = p.loadURDF("plane.urdf", 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)
xs, ys = np.random.choice(np.linspace(0,3,500), 100, replace=False),np.random.choice(np.linspace(0,3,500), 100, replace=False)
for i in range(100):
    x, y, z = xs[i], ys[i], 0
    color = np.random.choice(colors)
    objects = [p.loadURDF(f"./URDFs/disc_{color}.urdf", x, y, z, 0, 0, 0, 1)]

knife = p.loadURDF("./URDFs/knife.urdf", -0.1, -0.1, 0, 0.000000, 0.000000, 0.000000, 1.000000)





p.setGravity(0.000000, 0.000000, 0.000000)
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)
ref_time = time.time()
running_time = 60  # seconds
while (time.time() < ref_time + running_time):
    p.setGravity(0, 0, -9.81)
    # p.stepSimulation()

p.disconnect()

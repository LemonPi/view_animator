from view_animator import animate_view_in_background
from view_animator.pybullet_animator import PybulletOrbiter
import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# animate an orbit in the background
orbiter = PybulletOrbiter(update_period=0.01, dist=0.5, pitch=-48)
animate_view_in_background(orbiter)

p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "demo.mp4")

# rest is from https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/manyspheres.py

planeId = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
# populate it with some objects
p.loadURDF("plane.urdf", useMaximalCoordinates=True)
p.loadURDF("tray/traybox.urdf", useMaximalCoordinates=True)

p.setPhysicsEngineParameter(numSolverIterations=10)
p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
for i in range(10):
    for j in range(10):
        for k in range(10):
            ob = p.loadURDF("sphere_1cm.urdf", [0.02 * i, 0.02 * j, 0.2 + 0.02 * k],
                            useMaximalCoordinates=True)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)

p.setRealTimeSimulation(1)
while True:
    time.sleep(0.001)

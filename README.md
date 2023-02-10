## View Animator
This package allows you to programmatically change the camera view for a variety of environments 
(for example pybullet simulation, and RViz visualization).

## Installation
TODO after packaging for PYPI

## Usage
The core of it is an animator object and its `update()` method. You can either `update()` the animator
manually, or automatically in a separate daemon thread.

Example automatic orbiting animation

```python
from view_animator import animate_view_in_background
from view_animator.pybullet_animator import PybulletOrbitter
import pybullet as p

p.connect(p.GUI)
# other simulator configurations

# animate an orbit in the background at a distance of 0.5 away from the origin
orbiter = PybulletOrbitter(update_period=0.01, dist=0.5, target=(0, 0, 0))
animate_view_in_background(orbiter)
# set up objects

p.setRealTimeSimulation(1)
while True:
    # do sim computations
    time.sleep(0.001)
```
![dO08ur.gif](https://imgpile.com/images/dO08ur.gif)

Manual updating of an RViz orbiter

```python
from view_animator.rviz_animator import RVizOrbiter
import rospy

orbiter = RVizOrbiter(period=5, world_frame="world")
rate = rospy.Rate(1 / orbiter.update_period)
while not rospy.is_shutdown():
    # visualization and other processing
    orbiter.update()
    rate.sleep()
```
## Animators
While you can subclass `view_animator.base_animator.ViewAnimator` directly to suit your specific environment
and desired animation, there are a few predefined animators listed below. Note that you do not have to
have the environment installed to install this package.

### Pybullet
Install pybullet with
```pip install pybullet```
- `view_animator.pybullet_animator.PybulletOrbitter` for orbitting around a position with fixed pitch at a constant rate

### RViz
Requires the `rviz_animated_view_controller` plugin that can be installed with
```shell 
sudo apt install ros-<ros-distro>-rviz-animated-view-controller
```
e.g.
```shell
sudo apt install ros-noetic-rviz-animated-view-controller
```

- `view_animator.rviz_animator.RVizOrbitter` for orbitting around a position with fixed pitch at a constant rate

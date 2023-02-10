from view_animator.base_animator import Orbiter
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import Point, Vector3
from math import cos, pi, sin
import rospy


class RVizOrbiter(Orbiter):
    """Orbits around a target position in RViz. Note that angles are expected in radians."""
    def __init__(self, *args, world_frame="world", **kwargs):
        self.pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size=1)
        self.world_frame = world_frame
        super().__init__(*args, **kwargs)

    def generate_transform(self, dt):
        cp = CameraPlacement()
        dt = dt + self.offset_yaw * 2 * pi / self.period

        p = Point(self.dist * cos(2 * pi * dt / self.period) + self.target[0],
                  self.dist * sin(2 * pi * dt / self.period) + self.target[1],
                  self.target[2] + self.dist * sin(self.pitch))
        cp.eye.point = p
        cp.eye.header.frame_id = self.world_frame

        f = Point(*self.target)
        cp.focus.point = f
        cp.focus.header.frame_id = self.world_frame

        up = Vector3(sin(self.pitch), 0, 1 - sin(self.pitch))
        cp.up.vector = up
        cp.up.header.frame_id = self.world_frame

        cp.time_from_start = rospy.Duration(self.update_period)
        return cp

    def update_transform(self, cp):
        self.pub.publish(cp)

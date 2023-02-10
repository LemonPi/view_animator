from view_animator.base_animator import Orbiter
import math
import pybullet as p


class PybulletOrbiter(Orbiter):
    """Orbits around a target position in a pybullet environment. Note that angles are expected in degrees."""

    def generate_transform(self, dt):
        # pybullet accepts angles in degrees
        yaw = dt / (2 * math.pi) * 360
        if not self.ccw:
            yaw *= -1
        return yaw + self.offset_yaw

    def update_transform(self, yaw):
        p.resetDebugVisualizerCamera(self.dist, yaw, self.pitch, self.target)

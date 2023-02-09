from view_animator.base_animator import ViewAnimator
import math
import pybullet as p


class PybulletOrbitter(ViewAnimator):
    """Orbits around a target position in a pybullet environment."""

    def __init__(self, pitch=-48, offset_yaw=0, dist=0.8, target=(0, 0, 0), ccw=True, **kwargs):
        self.pitch = pitch
        self.offset_yaw = offset_yaw
        self.dist = dist
        self.target = target
        self.ccw = ccw
        super().__init__(**kwargs)

    def generate_transform(self, dt):
        # pybullet accepts angles in degrees
        yaw = dt / (2 * math.pi) * 360
        if not self.ccw:
            yaw *= -1
        return yaw + self.offset_yaw

    def update_transform(self, T):
        yaw = T
        p.resetDebugVisualizerCamera(self.dist, yaw, self.pitch, self.target)

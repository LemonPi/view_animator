from timeit import default_timer as timer
import abc
import time
import threading


class ViewAnimator:
    """Manages animation state agnostic to the environment to animate and the animation itself.
    Update should be called regularly, while pause and unpause can be used to temporary pause/play the animation.
    The animation is auto-played the first time update() is called. After reset, calling update() will restart."""

    def __init__(self, duration=None, period=5, update_period=0.01):
        """
        :param duration: None or number of seconds to play the animation; None will play it indefinitely
        :param period: number of seconds per animation loop (dt = 1 at end of period sent to generate_transform)
        :param update_period: minimum number of seconds per animation update; update does not happen automatically
        """
        self.duration = duration
        self.period = period
        self.update_period = update_period

        self.start_time = None
        self.pause_time = None
        self.last_update_time = None
        self.stopped = False
        self.paused_time_total = 0

    def reset(self):
        self.start_time = None
        self.pause_time = None
        self.last_update_time = None
        self.paused_time_total = 0

    def pause(self):
        self.pause_time = timer()

    def unpause(self):
        self.paused_time_total += timer() - self.pause_time
        self.pause_time = None

    def update(self):
        """Update the animation and potentially animate the next frame. Note that this does not have to be called
        in high frequency, but any frequency below self.update_frequency may lead to choppy animations."""
        if self.stopped or self.pause_time is not None:
            return
        if self.start_time is None:
            self.start_time = timer()
            return
        # based on the time elapsed, adjust camera transform
        dt = timer() - self.start_time - self.paused_time_total
        if self.last_update_time is not None and dt < self.last_update_time + self.update_period:
            return
        if self.duration is not None and dt > self.duration:
            return
        # scale with respect to period
        # assumes the parametric function for generating transforms is normalized so [0,1] is one "cycle"
        params = self.generate_transform(dt / self.period)
        self.update_transform(params)
        self.last_update_time = dt

    @abc.abstractmethod
    def generate_transform(self, dt):
        """Defines the view/camera transform (such as orbiting around a point) given dt scaled to [0,1].
        The transform parameters returned will be given as is to update_transform."""

    @abc.abstractmethod
    def update_transform(self, T):
        """Environment specific update to change the view. Input is the output of generate_transform."""


class Orbiter(ViewAnimator):
    """Orbits around a target position."""

    def __init__(self, pitch=0, offset_yaw=0, dist=0.8, target=(0, 0, 0), ccw=True, **kwargs):
        self.pitch = pitch
        self.offset_yaw = offset_yaw
        self.dist = dist
        self.target = target
        self.ccw = ccw
        super().__init__(**kwargs)


def animate_view_in_background(animator: ViewAnimator):
    """Create a daemon thread to do the animation in the background and return the daemon.
    Stop the animation by setting animator.stopped = True."""

    def animate():
        while True:
            if animator.stopped:
                break
            animator.update()
            time.sleep(animator.update_period)

    daemon = threading.Thread(target=animate, daemon=True, name='background_view_animator')
    daemon.start()

    return daemon

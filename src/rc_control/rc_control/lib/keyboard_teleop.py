"""WASD key state for keyboard teleop: steer and throttle in [-1, 1], with ramps."""

from dataclasses import dataclass


@dataclass
class KeyboardDrive:
    steer_ramp_rate: float      # [1/s]
    throttle_ramp_rate: float   # [1/s]
    release_timeout: float      # [s] no key repeat for this long means released
    steer: float = 0.0
    steer_target: float = 0.0
    throttle: float = 0.0
    throttle_target: float = 0.0
    _steer_seen: float = 0.0
    _throttle_seen: float = 0.0

    def key_pressed(self, key: str, now: float) -> None:
        if key == 'w':
            self.throttle_target = 1.0
            self._throttle_seen = now
        elif key == 's':
            self.throttle_target = -1.0
            self._throttle_seen = now
        elif key == 'a':
            self.steer_target = 1.0
            self._steer_seen = now
        elif key == 'd':
            self.steer_target = -1.0
            self._steer_seen = now
        elif key == ' ':
            self.steer = self.steer_target = 0.0
            self.throttle = self.throttle_target = 0.0
            self._steer_seen = self._throttle_seen = now

    def tick(self, now: float, dt: float) -> tuple[float, float]:
        """Steer and throttle after dt. Throttle cuts on release; steering ramps back."""
        if now - self._throttle_seen > self.release_timeout:
            self.throttle = self.throttle_target = 0.0
        if now - self._steer_seen > self.release_timeout:
            self.steer_target = 0.0

        self.steer = _ramp(self.steer, self.steer_target, self.steer_ramp_rate * dt)
        self.throttle = _ramp(self.throttle, self.throttle_target, self.throttle_ramp_rate * dt)
        return self.steer, self.throttle


def _ramp(current: float, target: float, step: float) -> float:
    """Move `current` toward `target` by at most `step`."""
    return max(current - step, min(current + step, target))

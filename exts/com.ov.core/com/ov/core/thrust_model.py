import math
from dataclasses import dataclass
from .orbit_math import Vec3, v_mul

def _quat_rotate(q: tuple, v: Vec3) -> Vec3:
    w, x, y, z = q
    t = (
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    )
    return (
        v[0] + w * t[0] + y * t[2] - z * t[1],
        v[1] + w * t[1] + z * t[0] - x * t[2],
        v[2] + w * t[2] + x * t[1] - y * t[0],
    )
@dataclass
class ThrustVectorModel:
    max_thrust_N: float
    mass_kg: float
    max_gimbal_rad: float

    throttle: float = 0.0
    gimbal_pitch: float = 0.0
    gimbal_yaw: float = 0.0

    def clamp_gimbal(self):
        lim = self.max_gimbal_rad
        self.gimbal_pitch = max(-lim, min(lim, self.gimbal_pitch))
        self.gimbal_yaw   = max(-lim, min(lim, self.gimbal_yaw))

    def _nozzle_body(self) -> Vec3:
        dp, dy = self.gimbal_pitch, self.gimbal_yaw
        return (
            math.sin(dp) * math.cos(dy),
            math.sin(dp) * math.sin(dy),
            -math.cos(dp),
        )

    def delta_v(self, attitude_quat: tuple, dt: float) -> Vec3:
        F = self.throttle * self.max_thrust_N
        n = self._nozzle_body()
        # thrust opposes nozzle direction, rotated to inertial frame
        F_body = (-F * n[0], -F * n[1], -F * n[2])
        F_inertial = _quat_rotate(attitude_quat, F_body)
        return v_mul(dt / self.mass_kg, F_inertial)
import math
from dataclasses import dataclass
from typing import Tuple

Vec3 = Tuple[float, float, float]


def v_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def v_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def v_mul(s: float, a: Vec3) -> Vec3:
    return (s * a[0], s * a[1], s * a[2])


def v_norm(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def rot_z(a: float, v: Vec3) -> Vec3:
    c, s = math.cos(a), math.sin(a)
    x, y, z = v
    return (c * x - s * y, s * x + c * y, z)


def rot_x(a: float, v: Vec3) -> Vec3:
    c, s = math.cos(a), math.sin(a)
    x, y, z = v
    return (x, c * y - s * z, s * y + c * z)


def rv_to_coe(mu: float, r: Vec3, v: Vec3):
    """State vectors -> classical orbital elements (a, e, inc, raan, argp, nu). Angles in radians."""
    import math
    rmag = v_norm(r)
    vmag = v_norm(v)

    # specific orbital energy -> semi-major axis
    energy = 0.5 * vmag*vmag - mu / rmag
    a = -mu / (2.0 * energy)

    # eccentricity vector
    rv_dot = r[0]*v[0] + r[1]*v[1] + r[2]*v[2]
    e_vec = (
        (vmag*vmag - mu/rmag) * r[0]/mu - rv_dot * v[0]/mu,
        (vmag*vmag - mu/rmag) * r[1]/mu - rv_dot * v[1]/mu,
        (vmag*vmag - mu/rmag) * r[2]/mu - rv_dot * v[2]/mu,
    )
    e = v_norm(e_vec)

    # angular momentum vector h = r x v
    hx = r[1]*v[2] - r[2]*v[1]
    hy = r[2]*v[0] - r[0]*v[2]
    hz = r[0]*v[1] - r[1]*v[0]
    hmag = math.sqrt(hx*hx + hy*hy + hz*hz)

    # inclination
    inc = math.acos(max(-1.0, min(1.0, hz / hmag)))

    # node vector n = z x h = (-hy, hx, 0)
    nx, ny = -hy, hx
    nmag = math.sqrt(nx*nx + ny*ny)

    # RAAN
    if nmag > 1e-10:
        raan = math.acos(max(-1.0, min(1.0, nx / nmag)))
        if ny < 0:
            raan = 2*math.pi - raan
    else:
        raan = 0.0

    # argument of perigee
    if nmag > 1e-10 and e > 1e-10:
        n_dot_e = nx*e_vec[0] + ny*e_vec[1]
        argp = math.acos(max(-1.0, min(1.0, n_dot_e / (nmag * e))))
        if e_vec[2] < 0:
            argp = 2*math.pi - argp
    else:
        argp = 0.0

    # true anomaly
    if e > 1e-10:
        e_dot_r = e_vec[0]*r[0] + e_vec[1]*r[1] + e_vec[2]*r[2]
        nu = math.acos(max(-1.0, min(1.0, e_dot_r / (e * rmag))))
        if rv_dot < 0:
            nu = 2*math.pi - nu
    else:
        nu = 0.0

    return a, e, inc, raan, argp, nu

def coe_to_rv(mu: float, a: float, e: float, inc_rad: float, raan_rad: float, argp_rad: float, nu_rad: float):
    """Classical orbital elements -> (r,v) in world/inertial frame. Angles in radians."""
    if a <= 0:
        raise ValueError("a must be > 0")
    if e < 0:
        raise ValueError("e must be >= 0")
    if e >= 1.0:
        raise ValueError("e must be < 1 for bound ellipse")

    p = a * (1.0 - e * e)
    rmag = p / (1.0 + e * math.cos(nu_rad))

    r_pqw = (rmag * math.cos(nu_rad), rmag * math.sin(nu_rad), 0.0)

    fac = math.sqrt(mu / p)
    v_pqw = (-fac * math.sin(nu_rad), fac * (e + math.cos(nu_rad)), 0.0)

    # PQW -> world: Rz(raan) * Rx(i) * Rz(argp)
    r_world = rot_z(raan_rad, rot_x(inc_rad, rot_z(argp_rad, r_pqw)))
    v_world = rot_z(raan_rad, rot_x(inc_rad, rot_z(argp_rad, v_pqw)))

    return r_world, v_world


def circular_orbit_ic(mu: float, radius: float, plane: str = "xy"):
    """Convenience circular initial conditions about origin."""
    v = math.sqrt(mu / radius)
    plane = plane.lower()
    if plane == "xy":
        return (radius, 0.0, 0.0), (0.0, v, 0.0)
    if plane == "xz":
        return (radius, 0.0, 0.0), (0.0, 0.0, v)
    if plane == "yz":
        return (0.0, radius, 0.0), (0.0, 0.0, v)
    raise ValueError("plane must be xy, xz, or yz")


@dataclass
class TwoBodyRK4:
    mu: float
    center: Vec3 = (0.0, 0.0, 0.0)

    def accel_gravity(self, r: Vec3) -> Vec3:
        d = v_sub(r, self.center)
        rmag = v_norm(d)
        if rmag < 1e-12:
            return (0.0, 0.0, 0.0)
        return v_mul(-self.mu / (rmag ** 3), d)

    def rk4_step(self, r: Vec3, v: Vec3, dt: float, a_cmd: Vec3 = (0.0, 0.0, 0.0)):
        def add(a, b): return v_add(a, b)
        def mul(s, a): return v_mul(s, a)

        def accel_total(rr: Vec3) -> Vec3:
            g = self.accel_gravity(rr)
            return (g[0] + a_cmd[0], g[1] + a_cmd[1], g[2] + a_cmd[2])

        a1 = accel_total(r)
        k1r, k1v = v, a1

        r2 = add(r, mul(0.5 * dt, k1r))
        v2 = add(v, mul(0.5 * dt, k1v))
        a2 = accel_total(r2)
        k2r, k2v = v2, a2

        r3 = add(r, mul(0.5 * dt, k2r))
        v3 = add(v, mul(0.5 * dt, k2v))
        a3 = accel_total(r3)
        k3r, k3v = v3, a3

        r4 = add(r, mul(dt, k3r))
        v4 = add(v, mul(dt, k3v))
        a4 = accel_total(r4)
        k4r, k4v = v4, a4

        r_next = add(r, mul(dt / 6.0, add(add(k1r, mul(2.0, k2r)), add(mul(2.0, k3r), k4r))))
        v_next = add(v, mul(dt / 6.0, add(add(k1v, mul(2.0, k2v)), add(mul(2.0, k3v), k4v))))
        return r_next, v_next


@dataclass
class FixedStepClock:
    dt_sim: float
    accum: float = 0.0

    def steps(self, dt_frame: float) -> int:
        self.accum += dt_frame
        n = int(self.accum // self.dt_sim)
        if n > 0:
            self.accum -= n * self.dt_sim
        return n
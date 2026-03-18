import math
from dataclasses import dataclass
from .orbit_math import Vec3, v_add, v_mul, v_norm

@dataclass
class CWIntegrator:
    n: float  

    def cw_accel(self, x, y, z, xd, yd, zd, a_cmd=(0.0, 0.0, 0.0)):
        ax = 3*self.n*self.n*x + 2*self.n*yd + a_cmd[0]
        ay = -2*self.n*xd                     + a_cmd[1]
        az = -self.n*self.n*z                 + a_cmd[2]
        return ax, ay, az

    def rk4_step(self, r: Vec3, v: Vec3, dt: float, a_cmd: Vec3 = (0.0, 0.0, 0.0)):
        x,  y,  z  = r
        xd, yd, zd = v

        def deriv(r_, v_):
            x_, y_, z_ = r_
            xd_, yd_, zd_ = v_
            ax, ay, az = self.cw_accel(x_, y_, z_, xd_, yd_, zd_, a_cmd)
            return v_, (ax, ay, az)

        dr1, dv1 = deriv(r, v)
        r2 = v_add(r, v_mul(0.5*dt, dr1)); v2 = v_add(v, v_mul(0.5*dt, dv1))
        dr2, dv2 = deriv(r2, v2)
        r3 = v_add(r, v_mul(0.5*dt, dr2)); v3 = v_add(v, v_mul(0.5*dt, dv2))
        dr3, dv3 = deriv(r3, v3)
        r4 = v_add(r, v_mul(dt, dr3));     v4 = v_add(v, v_mul(dt, dv3))
        dr4, dv4 = deriv(r4, v4)

        r_next = v_add(r, v_mul(dt/6.0, v_add(v_add(dr1, v_mul(2.0, dr2)),
                                               v_add(v_mul(2.0, dr3), dr4))))
        v_next = v_add(v, v_mul(dt/6.0, v_add(v_add(dv1, v_mul(2.0, dv2)),
                                               v_add(v_mul(2.0, dv3), dv4))))
        return r_next, v_next


def cw_initial_conditions(n: float, rho: float):
    return (rho, 0.0, 0.0), (0.0, -2.0*n*rho, 0.0)


def lvlh_to_inertial(r_lvlh: Vec3, ref_r: Vec3, ref_v: Vec3) -> Vec3:
    rmag = v_norm(ref_r)
    if rmag < 1e-12:
        return r_lvlh

    # unit radial (x_lvlh)
    x_hat = v_mul(1.0/rmag, ref_r)

    # h = ref_r x ref_v  (normal to orbital plane)
    hx = ref_r[1]*ref_v[2] - ref_r[2]*ref_v[1]
    hy = ref_r[2]*ref_v[0] - ref_r[0]*ref_v[2]
    hz = ref_r[0]*ref_v[1] - ref_r[1]*ref_v[0]
    hmag = math.sqrt(hx*hx + hy*hy + hz*hz)
    if hmag < 1e-12:
        return r_lvlh
    z_hat = (hx/hmag, hy/hmag, hz/hmag)

    # y_hat = z_hat x x_hat  (completes right-hand frame, along-track)
    y_hat = (z_hat[1]*x_hat[2] - z_hat[2]*x_hat[1],
             z_hat[2]*x_hat[0] - z_hat[0]*x_hat[2],
             z_hat[0]*x_hat[1] - z_hat[1]*x_hat[0])

    # rotate: r_inertial = x_lvlh*x_hat + y_lvlh*y_hat + z_lvlh*z_hat
    xl, yl, zl = r_lvlh
    return (xl*x_hat[0] + yl*y_hat[0] + zl*z_hat[0],
            xl*x_hat[1] + yl*y_hat[1] + zl*z_hat[1],
            xl*x_hat[2] + yl*y_hat[2] + zl*z_hat[2])
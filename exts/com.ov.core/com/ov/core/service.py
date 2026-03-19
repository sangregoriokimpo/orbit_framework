from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, List

from .orbit_math import Vec3, TwoBodyRK4, FixedStepClock, circular_orbit_ic, coe_to_rv, v_add, v_mul, v_norm, v_sub
from .CWIntegrator import cw_initial_conditions, CWIntegrator
from .orbit_math import lvlh_frame, inertial_to_lvlh, lvlh_to_inertial, FixedStepClock, TwoBodyRK4

import math
import omni.usd
from .usd_persistence import read_body_from_prim, scan_stage_for_bodies
@dataclass
class OrbitBody:
    prim_path: str
    attractor_path: str
    mu: float
    dt_sim: float

    r: Vec3
    v: Vec3

    control_mode: str = "free"
    target_offset: Vec3 = (0.0, 0.0, 0.0)
    kp: float = 0.0
    kd: float = 0.0
    a_max: float = 0.0
    thrust: Vec3 = (0.0, 0.0, 0.0)
    enabled: bool = True

    cw_ref_path: str = ""

    _clock: FixedStepClock = field(default_factory=lambda: FixedStepClock(1/120))
    _dyn: TwoBodyRK4 = field(default_factory=lambda: TwoBodyRK4(1.0))


class OrbitService:
    def __init__(self):
        self._bodies: Dict[str, OrbitBody] = {}
        self._stage = None
        self._step_counters: Dict[str,int] ={}

    def add_body_circular(self, prim_path, attractor_path, mu, dt_sim, radius, plane="xy"):
        stage = omni.usd.get_context().get_stage()
        if stage:
            kwargs = read_body_from_prim(stage, prim_path) 
            if kwargs:
                return self.add_body_from_kwargs(**kwargs)
                
        r0, v0 = circular_orbit_ic(mu, radius, plane=plane)
        body = OrbitBody(prim_path=prim_path, attractor_path=attractor_path,
                         mu=float(mu), dt_sim=float(dt_sim), r=r0, v=v0)
        body._clock = FixedStepClock(dt_sim=float(dt_sim))
        body._dyn = TwoBodyRK4(mu=float(mu), center=(0.0, 0.0, 0.0))
        #self._bodies[prim_path] = body
        self._step_counters[prim_path] = 0
        # self._write(body)
        self._bodies[prim_path] = body
        self._write(body)
        return body
    
    def add_body_from_kwargs(self, **kwargs) -> "OrbitBody":
        body = OrbitBody(**kwargs)
        body._clock = FixedStepClock(dt_sim=float(kwargs["dt_sim"]))
        body._dyn = TwoBodyRK4(mu=float(kwargs["mu"]),center =(0.0,0.0,0.0))
        self._bodies[kwargs["prim_path"]] = body
        return body
    


    def add_body_elements(self, prim_path, attractor_path, mu, dt_sim,
                          a, e, inc_deg, raan_deg, argp_deg, nu_deg):
        r0, v0 = coe_to_rv(float(mu), float(a), float(e),
                            math.radians(float(inc_deg)), math.radians(float(raan_deg)),
                            math.radians(float(argp_deg)), math.radians(float(nu_deg)))
        body = OrbitBody(prim_path=prim_path, attractor_path=attractor_path,
                         mu=float(mu), dt_sim=float(dt_sim), r=r0, v=v0)
        body._clock = FixedStepClock(dt_sim=float(dt_sim))
        body._dyn = TwoBodyRK4(mu=float(mu), center=(0.0, 0.0, 0.0))
        self._step_counters[prim_path] = 0 
        self._write(body)                    
        self._bodies[prim_path] = body
        return body

    def remove_body(self, prim_path):
        self._bodies.pop(prim_path, None)
        self._step_counters.pop(prim_path,None)
        if self._stage is not None:
            try:
                from .usd_persistence import erase_body_from_prim
                erase_body_from_prim(self._stage,prim_path)
            except Exception as e:
                print(f"[OrbitService] erase error for {prim_path}: {e}")

    def list_bodies(self) -> List[str]:
        return list(self._bodies.keys())

    def get_body(self, prim_path) -> Optional[OrbitBody]:
        return self._bodies.get(prim_path)

    def apply_impulse(self, prim_path, dv):
        b = self._bodies.get(prim_path)
        if b:
            b.v = (b.v[0] + dv[0], b.v[1] + dv[1], b.v[2] + dv[2])
            self._write(b)

    def set_dock(self, prim_path, offset):
        b = self._bodies.get(prim_path)
        if b:
            b._pre_dock_r = b.r
            b._pre_dock_v = b.v
            b.control_mode = "dock"
            b.target_offset = offset
            self._write(b)

    def clear_dock(self, prim_path):
        b = self._bodies.get(prim_path)
        if b and b.control_mode == "dock":
            b.control_mode = "free"
            b.r = getattr(b, "_pre_dock_r", b.r)
            b.v = getattr(b, "_pre_dock_v", b.v)
            self._write(b)

    def set_pd_hold(self, prim_path, target_offset, kp, kd, a_max=0.0):
        b = self._bodies.get(prim_path)
        if b:
            b.control_mode = "pd"
            b.target_offset = target_offset
            b.kp = float(kp)
            b.kd = float(kd)
            b.a_max = float(a_max)
            self._write(b)

    def clear_pd(self, prim_path):
        b = self._bodies.get(prim_path)
        if b and b.control_mode == "pd":
            b.control_mode = "free"
            self._write(b)

    def step_body(self, prim_path, dt_frame):
        b = self._bodies.get(prim_path)
        if not b or not b.enabled:
            return
        n = b._clock.steps(dt_frame)
        if n <= 0:
            return
        counter = self._step_counters.get(prim_path, 0)
        for _ in range(n):
            if b.control_mode == "dock":
                b.r = b.target_offset
                b.v = (0.0, 0.0, 0.0)
                counter += 1
                continue
            if b.control_mode == "cw":
                cw = getattr(b,"_cw",None)
                if cw is None:
                    continue
                b.r , b.v = cw.rk4_step(b.r,b.v,b.dt_sim,a_cmd=b.thrust)
                b._orbit_dirty = True
                continue

            a_cmd = b.thrust  

            if b.control_mode == "pd":
                ex = b.target_offset[0] - b.r[0]
                ey = b.target_offset[1] - b.r[1]
                ez = b.target_offset[2] - b.r[2]
                ax = b.kp * ex + b.kd * (0.0 - b.v[0])
                ay = b.kp * ey + b.kd * (0.0 - b.v[1])
                az = b.kp * ez + b.kd * (0.0 - b.v[2])
                if b.a_max and b.a_max > 0.0:
                    amag = math.sqrt(ax*ax + ay*ay + az*az)
                    if amag > b.a_max:
                        s = b.a_max / amag
                        ax, ay, az = ax*s, ay*s, az*s
                a_cmd = (a_cmd[0]+ax, a_cmd[1]+ay, a_cmd[2]+az)

            # b.r, b.v = b._dyn.rk4_step(b.r, b.v, b.dt_sim, a_cmd=a_cmd)
            # counter += 1
            b.r, b.v = b._dyn.rk4_step(b.r, b.v, b.dt_sim, a_cmd=a_cmd)
            b._orbit_dirty = True
            # b._orbit_dirty = True
            # if a_cmd != (0.0,0.0,0.0):
            #     b._orbit_dirty = True
            # elif counter % 120 == 0:
            #     b._orbit_dirty = True
            # counter += 1
            # if counter % 30 == 0:
            #     self._write(b)

        self._step_counters[prim_path] = counter
        self._write(b)

    # def set_teleop(self, prim_path: str):
    #     b = self._bodies.get(prim_path)
    #     if b:
    #         b.control_mode = "teleop"
    #         b.thrust = (0.0, 0.0, 0.0)

    def set_thrust(self, prim_path: str, thrust: Vec3):
        b = self._bodies.get(prim_path)
        if b:
            b.thrust = thrust

    def clear_thrust(self, prim_path: str):
        b = self._bodies.get(prim_path)
        if b:
            b.thrust = (0.0, 0.0, 0.0)
    
    # def reset(self):
    #     self._bodies.clear()

    def set_stage(self, stage):
        self._stage = stage

    def clear_stage(self):
        self._stage = None

    def reset(self):
        self._bodies.clear()
        self._step_counters.clear()

    def restore_from_stage(self, stage) -> int:
        self._stage = stage
        self._bodies.clear()
        self._step_counters.clear()
        count = 0
        for data in scan_stage_for_bodies(stage):
            body = OrbitBody(
                prim_path=data["prim_path"], attractor_path=data["attractor_path"],
                mu=data["mu"], dt_sim=data["dt_sim"],
                r=data["r"], v=data["v"],
                control_mode=data["control_mode"], target_offset=data["target_offset"],
                kp=data["kp"], kd=data["kd"], a_max=data["a_max"], enabled=data["enabled"],
                cw_ref_path=data.get("cw_ref_path","")
            )
            body._clock = FixedStepClock(dt_sim=data["dt_sim"])
            body._dyn = TwoBodyRK4(mu=data["mu"], center=(0.0, 0.0, 0.0))
            if body.control_mode == "cw" and body.cw_ref_path:
                ref = self._bodies.get(body.cw_ref_path)
                if ref is not None:
                    a_ref = v_norm(ref.r)
                    n = math.sqrt(body.mu / a_ref**3)
                    body._cw = CWIntegrator(n=n)
            self._bodies[data["prim_path"]] = body
            self._step_counters[data["prim_path"]] = 0
            count += 1
        for body in self._bodies.values():
            if body.control_mode == "cw" and body.cw_ref_path:
                if not getattr(body,"_cw",None):
                    ref = self._bodies.get(body.cw_ref_path)
                    if ref is not None:
                        a_ref = v_norm(ref.r)
                        n = math.sqrt(body.mu / a_ref**3)
                        body._cw = CWIntegrator(n=n)
                    else:
                        print(f"[OrbitService] WARNING: CW body {body.prim_path} ref {body.cw_ref_path} not found after restore")
        print(f"[OrbitService] Restored {count} body/bodies from stage")
        return count

    def _write(self, body: OrbitBody):
        if self._stage is None:
            return
        try:
            from .usd_persistence import write_body_to_prim
            write_body_to_prim(self._stage, body)
        except Exception as e:
            print(f"[OrbitService] write error for {body.prim_path}: {e}")

    def add_body_cw(self, prim_path: str, attractor_path: str, ref_path: str, mu: float,
                    dt_sim: float, rho: float):
        ref = self._bodies.get(ref_path)
        if ref is None:
            raise ValueError(f"Reference body {ref_path} not registered")
        a_ref = v_norm(ref.r)
        n = math.sqrt(mu / a_ref**3)
        r0 , v0 = cw_initial_conditions(n, rho)
        body = OrbitBody(prim_path=prim_path,
                         attractor_path=attractor_path,
                         mu=float(mu),
                         dt_sim=float(dt_sim),
                         r=r0,
                         v=v0,
                         control_mode="cw",
                         cw_ref_path=ref_path)
        body._clock = FixedStepClock(dt_sim=float(dt_sim))
        body._cw = CWIntegrator(n=n)
        self._step_counters[prim_path] = 0
        self._bodies[prim_path] = body
        self._write(body)
        return body
    
    def _step_cw(self,body: OrbitBody, dt: float):
        ref = self._bodies.get(body.attractor_path)
        if ref is None:
            body.r , body.v = body._dyn.rk4_step(body.r,body.v)
            return
        R_hat, S_hat,W_hat , omega_vec = lvlh_frame(ref.r,ref.v)
        dr_1, dv_1 = inertial_to_lvlh(body.r,body.v,R_hat,S_hat,W_hat,omega_vec)
        dr_1, dv_1 = body._cw.rk4_step(dr_1,dv_1,dt)
        body.r, body.v = lvlh_to_inertial(dr_1, dv_1,R_hat,S_hat,W_hat,omega_vec)


    # def get_orbit_service() -> OrbitService:
    #     global _SERVICE
    #     if _SERVICE is None:
    #         _SERVICE = OrbitService()
    #     return _SERVICE


_SERVICE: Optional[OrbitService] = None

def get_orbit_service() -> OrbitService:
    global _SERVICE
    if _SERVICE is None:
        _SERVICE = OrbitService()
    return _SERVICE
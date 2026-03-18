"""
usd_persistence.py  –  com.ov.core
====================================
Reads and writes OrbitBody state as custom attributes in the ``orbit:``
namespace on each body prim. State is saved inside the .usd file and
restored automatically on reload.

Attributes written to each body prim
--------------------------------------
orbit:attractor_path   string
orbit:mu               double
orbit:dt_sim           double
orbit:r                double3   (relative position to attractor)
orbit:v                double3   (relative velocity)
orbit:control_mode     string    "free" | "dock" | "pd"
orbit:target_offset    double3
orbit:kp               double
orbit:kd               double
orbit:a_max            double
orbit:enabled          bool
"""

from __future__ import annotations
from typing import Optional, TYPE_CHECKING

from pxr import Usd, Sdf, Gf

if TYPE_CHECKING:
    from .service import OrbitBody

# ── attribute name constants ──────────────────────────────────────────────────

ATTR_ATTRACTOR = "orbit:attractor_path"
ATTR_MU        = "orbit:mu"
ATTR_DT_SIM    = "orbit:dt_sim"
ATTR_R         = "orbit:r"
ATTR_V         = "orbit:v"
ATTR_MODE      = "orbit:control_mode"
ATTR_TARGET    = "orbit:target_offset"
ATTR_KP        = "orbit:kp"
ATTR_KD        = "orbit:kd"
ATTR_AMAX      = "orbit:a_max"
ATTR_ENABLED   = "orbit:enabled"
ATTR_CW_REF = "orbit:cw_ref_path"

_ALL_ATTRS = (ATTR_ATTRACTOR, ATTR_MU, ATTR_DT_SIM, ATTR_R, ATTR_V,
              ATTR_MODE, ATTR_TARGET, ATTR_KP, ATTR_KD, ATTR_AMAX, ATTR_ENABLED,ATTR_CW_REF)

# ── helpers ───────────────────────────────────────────────────────────────────

def _get_or_create(prim: Usd.Prim, name: str, sdf_type: Sdf.ValueTypeName) -> Usd.Attribute:
    attr = prim.GetAttribute(name)
    if attr and attr.IsValid():
        return attr
    return prim.CreateAttribute(name, sdf_type, custom=True)


def _v3(v) -> Gf.Vec3d:
    return Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]))


def _t3(v) -> tuple:
    return (float(v[0]), float(v[1]), float(v[2]))

# ── write ─────────────────────────────────────────────────────────────────────

def write_body_to_prim(stage: Usd.Stage, body: "OrbitBody") -> bool:
    """
    Persist the full OrbitBody state onto its USD prim.
    Returns False if the prim doesn't exist yet (caller can retry).
    All writes are batched in a single Sdf.ChangeBlock.
    """
    prim = stage.GetPrimAtPath(body.prim_path)
    if not prim or not prim.IsValid():
        return False

    with Sdf.ChangeBlock():
        _get_or_create(prim, ATTR_ATTRACTOR, Sdf.ValueTypeNames.String ).Set(body.attractor_path)
        _get_or_create(prim, ATTR_MU,        Sdf.ValueTypeNames.Double ).Set(float(body.mu))
        _get_or_create(prim, ATTR_DT_SIM,    Sdf.ValueTypeNames.Double ).Set(float(body.dt_sim))
        _get_or_create(prim, ATTR_R,         Sdf.ValueTypeNames.Double3).Set(_v3(body.r))
        _get_or_create(prim, ATTR_V,         Sdf.ValueTypeNames.Double3).Set(_v3(body.v))
        _get_or_create(prim, ATTR_MODE,      Sdf.ValueTypeNames.String ).Set(body.control_mode)
        _get_or_create(prim, ATTR_TARGET,    Sdf.ValueTypeNames.Double3).Set(_v3(body.target_offset))
        _get_or_create(prim, ATTR_KP,        Sdf.ValueTypeNames.Double ).Set(float(body.kp))
        _get_or_create(prim, ATTR_KD,        Sdf.ValueTypeNames.Double ).Set(float(body.kd))
        _get_or_create(prim, ATTR_AMAX,      Sdf.ValueTypeNames.Double ).Set(float(body.a_max))
        _get_or_create(prim, ATTR_ENABLED,   Sdf.ValueTypeNames.Bool   ).Set(bool(body.enabled))
        _get_or_create(prim, ATTR_CW_REF, Sdf.ValueTypeNames.String).Set(body.cw_ref_path)

    return True


def erase_body_from_prim(stage: Usd.Stage, prim_path: str):
    """Remove all orbit: attributes from a prim (called when a body is removed)."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return
    for name in _ALL_ATTRS:
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            prim.RemoveProperty(name)

# ── read ──────────────────────────────────────────────────────────────────────

def read_body_from_prim(stage: Usd.Stage, prim_path: str) -> Optional[dict]:
    """
    Return a dict of kwargs for reconstructing an OrbitBody,
    or None if the prim carries no orbit state.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None

    # The presence of orbit:attractor_path is the marker
    attractor_attr = prim.GetAttribute(ATTR_ATTRACTOR)
    if not attractor_attr or not attractor_attr.IsValid() or attractor_attr.Get() is None:
        return None

    def _get(name, default):
        a = prim.GetAttribute(name)
        v = a.Get() if (a and a.IsValid()) else None
        return v if v is not None else default

    return dict(
        prim_path      = prim_path,
        attractor_path = str(_get(ATTR_ATTRACTOR, "")),
        mu             = float(_get(ATTR_MU,       1.0)),
        dt_sim         = float(_get(ATTR_DT_SIM,   1/120)),
        r              = _t3(_get(ATTR_R,      Gf.Vec3d(0, 0, 0))),
        v              = _t3(_get(ATTR_V,      Gf.Vec3d(0, 0, 0))),
        control_mode   = str(_get(ATTR_MODE,   "free")),
        target_offset  = _t3(_get(ATTR_TARGET, Gf.Vec3d(0, 0, 0))),
        kp             = float(_get(ATTR_KP,   0.0)),
        kd             = float(_get(ATTR_KD,   0.0)),
        a_max          = float(_get(ATTR_AMAX, 0.0)),
        enabled        = bool(_get(ATTR_ENABLED, True)),
        cw_ref_path = str(_get(ATTR_CW_REF, "")),
    )


def scan_stage_for_bodies(stage: Usd.Stage) -> list:
    """
    Walk every prim in the stage and collect orbit body data dicts.
    Skips /OrbitViz prims (those are visualizer curves, not bodies).
    """
    results = []
    if stage is None:
        return results
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith("/OrbitViz"):
            continue
        data = read_body_from_prim(stage, path)
        if data:
            results.append(data)
    return results
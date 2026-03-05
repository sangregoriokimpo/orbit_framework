import omni.ext
import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdGeom, Gf

from .service import get_orbit_service


class OrbitCoreExtension(omni.ext.IExt):
    """
    Runs the orbit simulation and applies transforms to prims.
    Other extensions call into OrbitService to add bodies / apply controls.
    """

    def on_startup(self, ext_id: str):
        self._app = omni.kit.app.get_app()
        self._usd = omni.usd.get_context()
        self._timeline = omni.timeline.get_timeline_interface()
        self._svc = get_orbit_service()
        self._update_sub = self._app.get_update_event_stream().create_subscription_to_pop(self._on_update)
        self._timeline.play()
        print("[OrbitCore] started")

    def on_shutdown(self):
        if self._update_sub:
            self._update_sub.unsubscribe()
            self._update_sub = None
        print("[OrbitCore] shutdown")

    def _get_translate_op(self, stage, path: str):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return None
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                return op
        return xf.AddTranslateOp()

    def _get_world_pos(self, prim):
        m = omni.usd.get_world_transform_matrix(prim)
        p = m.ExtractTranslation()
        return (float(p[0]), float(p[1]), float(p[2]))

    def _on_update(self, e):
        if not self._timeline.is_playing():
            return
        stage = self._usd.get_stage()
        if stage is None:
            return

        dt_frame = float(e.payload.get("dt", 0.0))
        if dt_frame <= 0.0:
            return

        for prim_path in self._svc.list_bodies():
            body = self._svc.get_body(prim_path)
            if not body or not body.enabled:
                continue

            self._svc.step_body(prim_path, dt_frame)

            bprim = stage.GetPrimAtPath(body.prim_path)
            aprim = stage.GetPrimAtPath(body.attractor_path)
            if not bprim.IsValid() or not aprim.IsValid():
                continue

            ax, ay, az = self._get_world_pos(aprim)
            wx, wy, wz = ax + body.r[0], ay + body.r[1], az + body.r[2]

            top = self._get_translate_op(stage, body.prim_path)
            if top is None:
                continue
            try:
                top.Set(Gf.Vec3d(wx, wy, wz))
            except Exception:
                pass
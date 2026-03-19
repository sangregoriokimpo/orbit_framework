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
        self._write_every_n = 30
        self._frame_count   = 0
        stage = self._usd.get_stage()
        if stage is not None:
            self._svc.restore_from_stage(stage)
        self._update_sub = self._app.get_update_event_stream().create_subscription_to_pop(self._on_update)
        self._stage_event_sub = self._usd.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name = "OrbitCore.StageEvent"
        )
        self._timeline.play()
        print("[OrbitCore] started")
        # self._restore_from_stage()
        # self._stage_sub = omni.usd.get_context().get_stage_event_stream()\
        # .create_subscription_to_pop(self._on_stage_event)

    

    def on_shutdown(self):
        if self._update_sub:
            self._update_sub.unsubscribe()
            # self._update_sub = None
        if self._stage_event_sub:
            self._stage_event_sub.unsubscribe()
            self._stage_event_sub = None
        self._svc.clear_stage()
        print("[OrbitCore] shutdown")

    def _restore_from_stage(self):
        stage = self._usd.get_stage()
        if stage is None:
            return
        self._svc.restore_from_stage(stage)

    # def _on_stage_event(self,e):
    #     t = e.type
    #     if t == int(StageEventType.OPENED):
    #         stage = self._usd.get_stage()
    #         if stage is not None:
    #             self._svc.restore_from_stage(stage)
    #     elif t == int(StageEventType.CLOSED):
    #         self._svc.clear_stage()
    #         self._svc.reset()
        # if e.type in (int(StageEventType.OPENED),
        #               int(StageEventType.CLOSED),
        #               int (StageEventType.SAVED)):
        #     self._svc.reset()
        #     print("[OrbitCore] Stage event - bodies cleared")


    def _get_translate_op(self, stage, path: str):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return None
        xf = UsdGeom.Xformable(prim)
        for op in xf.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                return op
        # return xf.AddTranslateOp()
        try:
            return xf.AddTranslateOp()
        except Exception as ex:
            print(f"[OrbitCore] Could not add translate op on {path} : {ex}")
            return None

    def _get_world_pos(self, prim):
        m = omni.usd.get_world_transform_matrix(prim)
        p = m.ExtractTranslation()
        return (float(p[0]), float(p[1]), float(p[2]))
    
    # def restore_from_stage(self):
    #     stage = self._usd.get_stage()
    #     if stage is None:
    #         return
    #     for kwargs in discover_bodies(stage):
    #         if kwargs["prim_path"] not in self._svc.list_bodies():
    #             self._svc.add_body_from_kwargs(**kwargs)
    #             print(f"[OrbitCore] restored {kwargs['prim_path']} from USD")

    def _on_stage_event(self, e):
        if e.type == int(omni.usd.StageEventType.OPENED):
            self._restore_from_stage()




    def _on_update(self, e):
            if not self._timeline.is_playing():
                return
            
            stage = self._usd.get_stage()
            if stage is None:
                return
            
            dt_frame = float(e.payload.get("dt",0.0))
            if dt_frame <= 0.0:
                return
            self._frame_count += 1
            do_write = (self._frame_count % self._write_every_n == 0)

        # try:
        #     if not self._timeline.is_playing():
        #         return
        #     stage = self._usd.get_stage()
        #     if stage is None:
        #         return
        #     if self._svc._stage is None:
        #         self._svc.set_stage(stage)
        #         for p in self._svc.list_bodies():
        #             b = self._svc.get_body(p)
        #             if b:
        #                 self._svc._write(b)

        #     dt_frame = float(e.payload.get("dt", 0.0))
        #     if dt_frame <= 0.0:
        #         return

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
                wx = ax + body.r[0]
                wy = ay + body.r[1]
                wz = az + body.r[2]
                # if body.control_mode == "cw" and body.cw_ref_path:
                #     ref = self._svc.get_body(body.cw_ref_path)
                #     if ref is not None:
                #         r_inertial = lvlh_to_inertial(body.r,ref.r,ref.v)
                #         wx = ax + ref.r[0] + r_inertial[0]
                #         wy = ay + ref.r[1] + r_inertial[1]
                #         wz = az + ref.r[2] + r_inertial[2]
                #     else:
                #         wx,wy,wz = ax + body.r[0], ay + body.r[1], az + body.r[2]
                # else:
                #     wx, wy, wz = ax + body.r[0], ay + body.r[1], az + body.r[2]   

                # wx, wy, wz = ax + body.r[0], ay + body.r[1], az + body.r[2]

                top = self._get_translate_op(stage, body.prim_path)
                if top is None:
                    continue
                try:
                    top.Set(Gf.Vec3d(wx, wy, wz))
                except Exception:
                    pass
        # except Exception as ex:
        #     print(f"[OrbitCore] Update error (transient?): {ex}")
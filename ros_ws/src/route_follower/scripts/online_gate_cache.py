"""Keep accepted course anchors across detector dropouts and tracker ID changes.

方案 13/14 修复:
  - existing gate 坐标更新累计超过阈值时 changed=True (否则 planner 永远用旧 profile);
  - 小抖动只做 EMA, 达到阈值才触发 rebuild;
  - 新门必须连续 N 帧稳定才 commit (map_commit_stable_frames);
  - 静态门附近的视觉观测不再一律丢弃, 而是关联到静态门作为 measurement update,
    累计稳定且变化超阈值时输出 correction。
"""

import math

DEFAULTS = dict(stable_frames=3, xy_threshold=0.15, z_threshold=0.10,
                s_threshold=0.20, static_assoc_s=8.0, static_assoc_xy=5.0,
                static_correction_max=1.5)


def _finite(v, default=float("nan")):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


class OnlineGateCache:
    def __init__(self, stable_frames=None, xy_threshold=None, z_threshold=None,
                 s_threshold=None, static_assoc_s=None, static_assoc_xy=None,
                 static_correction_max=None):
        self.gates = []
        self.by_track_id = {}
        self.next_id = 100000
        self.static_corrections = {}
        self._static_pending = {}
        self._pending = []
        self._published = {}
        self.stable_frames = int(stable_frames if stable_frames is not None
                                 else DEFAULTS["stable_frames"])
        self.xy_threshold = float(xy_threshold if xy_threshold is not None
                                  else DEFAULTS["xy_threshold"])
        self.z_threshold = float(z_threshold if z_threshold is not None
                                 else DEFAULTS["z_threshold"])
        self.s_threshold = float(s_threshold if s_threshold is not None
                                 else DEFAULTS["s_threshold"])
        self.static_assoc_s = float(static_assoc_s if static_assoc_s is not None
                                    else DEFAULTS["static_assoc_s"])
        self.static_assoc_xy = float(static_assoc_xy if static_assoc_xy is not None
                                     else DEFAULTS["static_assoc_xy"])
        self.static_correction_max = float(static_correction_max if static_correction_max is not None
                                           else DEFAULTS["static_correction_max"])

    # ---------- lookups ----------
    def correction_for(self, gate_id):
        return self.static_corrections.get(gate_id)

    def _match(self, g):
        old = self.by_track_id.get(g.get("id"))
        if old is not None and any(old is gate for gate in self.gates):
            return old
        matches = [gate for gate in self.gates if abs(gate["s"] - g["s"]) < 5.0
                   and math.hypot(gate["x"] - g["x"], gate["y"] - g["y"]) < 5.0]
        if matches:
            return min(matches, key=lambda gate: abs(gate["s"] - g["s"]))
        return None

    def _match_pending(self, g):
        old = self.by_track_id.get(g.get("id"))
        if old is not None and any(old is p["obs"] for p in self._pending):
            return old
        matches = [p for p in self._pending
                   if abs(p["obs"]["s"] - g["s"]) < 5.0
                   and math.hypot(p["obs"]["x"] - g["x"], p["obs"]["y"] - g["y"]) < 5.0]
        if matches:
            return min(matches, key=lambda p: abs(p["obs"]["s"] - g["s"]))["obs"]
        return None

    def _match_static(self, g, static_gates):
        if not static_gates:
            return None
        candidates = [sg for sg in static_gates
                      if _finite(sg.get("s")) == _finite(sg.get("s"))
                      and abs(g["s"] - float(sg["s"])) < self.static_assoc_s
                      and math.hypot(g["x"] - float(sg.get("x", 0.0)),
                                     g["y"] - float(sg.get("y", 0.0))) < self.static_assoc_xy]
        if not candidates:
            return None
        return min(candidates, key=lambda sg: abs(g["s"] - float(sg["s"])))

    # ---------- ingest ----------
    def ingest(self, observations, static_s, s_now, now, project, completed,
               static_gates=None):
        changed = False
        for observation in observations:
            g = dict(observation)
            obs_track_id = g.get("id")
            if not self._trusted(g, now):
                continue
            g["s"] = float(project(g))
            sg = self._match_static(g, static_gates)
            if sg is not None:
                if sg.get("id") not in completed and sg["s"] > s_now + 5.:
                    changed |= self._update_static(g, sg)
                continue
            if not static_gates and any(abs(g["s"] - s) < self.static_assoc_s
                                        for s in static_s):
                continue
            old = self._match(g)
            if old is not None:
                if old["id"] in completed or old["s"] < s_now + 5. \
                        or g.get("last_seen", 0) <= old["last_seen"]:
                    continue
                published = self._published.get(old["id"], dict(old))
                for key in ("x", "y", "z", "s"):
                    g[key] = .7 * old[key] + .3 * g[key]
                g["id"] = old["id"]
                if obs_track_id is not None:
                    self.by_track_id[obs_track_id] = old
                old.update(g)
                deltas = {k: abs(old[k]-published[k]) for k in ("x","y","z","s")}
                if deltas["x"] > self.xy_threshold or deltas["y"] > self.xy_threshold \
                        or deltas["z"] > self.z_threshold or deltas["s"] > self.s_threshold:
                    changed = True
                continue
            if g["s"] <= s_now + 2.0:
                continue
            pending = self._match_pending(g)
            if pending is not None:
                if g.get("last_seen",0) <= pending.get("last_seen",0):
                    continue
                pending.update(g)
                pending["_stable"] = pending.get("_stable", 1) + 1
                if obs_track_id is not None:
                    self.by_track_id[obs_track_id] = pending
                if pending["_stable"] >= self.stable_frames:
                    self._commit(pending)
                    changed = True
                continue
            g["_stable"] = 1
            self._pending.append({"obs": g})
            if obs_track_id is not None:
                self.by_track_id[obs_track_id] = g
            if self.stable_frames <= 1:
                self._commit(g)
                changed = True
        if changed:
            self._published = {g["id"]: dict(g) for g in self.gates}
        return changed

    def _commit(self, g):
        g.pop("_stable", None)
        g["track_id"] = g.get("id")
        g["id"] = self.next_id
        self.next_id += 1
        self.gates.append(g)
        self._pending = [p for p in self._pending if p["obs"] is not g]
        for tid, ref in list(self.by_track_id.items()):
            if ref is g:
                self.by_track_id[tid] = g

    # ---------- helpers ----------
    def _trusted(self, g, now):
        if not g.get("valid", True) or not g.get("hard_anchor", False):
            return False
        if not all(math.isfinite(_finite(g.get(k))) for k in ("x", "y", "z")):
            return False
        last_seen = _finite(g.get("last_seen"), 0.0)
        if now - last_seen > 1.0 or last_seen > now + .2:
            return False
        if g.get("support", 0) < 5 or max(_finite(g.get("sigma_x"), 9), _finite(g.get("sigma_y"), 9)) > 1.2 \
                or _finite(g.get("sigma_z"), 9) > .6:
            return False
        # Dense stereo is usable only after repeated, stable, confident observations.
        if not g.get("geometry_valid", False) and (g.get("support", 0) < 8
                                                   or (g.get("confidence") or 0) < .65):
            return False
        return True

    def _update_static(self, g, sg):
        """静态门附近的观测关联为 measurement update (方案 14)。"""
        gate_id = sg.get("id")
        if gate_id is None:
            return False
        dx = g["x"] - float(sg.get("x", 0.0))
        dy = g["y"] - float(sg.get("y", 0.0))
        dz = g["z"] - float(sg.get("z", 0.0))
        d = math.hypot(dx, dy)
        scale = 1.0
        if d > self.static_correction_max and d > 1e-6:
            scale = self.static_correction_max / d
        dz = max(-self.static_correction_max, min(self.static_correction_max, dz))
        pending = self._static_pending.get(gate_id)
        if pending is None:
            pending = {"count": 0, "dx": 0.0, "dy": 0.0, "dz": 0.0}
            self._static_pending[gate_id] = pending
        if g.get("last_seen",0) <= pending.get("last_seen",-1):
            return False
        pending["last_seen"] = g.get("last_seen",0)
        pending["count"] += 1
        pending["dx"] = 0.7 * pending["dx"] + 0.3 * dx * scale
        pending["dy"] = 0.7 * pending["dy"] + 0.3 * dy * scale
        pending["dz"] = 0.7 * pending["dz"] + 0.3 * dz
        if pending["count"] < self.stable_frames:
            return False
        committed = self.static_corrections.get(gate_id)
        new_corr = {"dx": pending["dx"], "dy": pending["dy"], "dz": pending["dz"]}
        if committed is None:
            self.static_corrections[gate_id] = new_corr
            return True
        changed = (abs(new_corr["dx"] - committed["dx"]) > self.xy_threshold
                   or abs(new_corr["dy"] - committed["dy"]) > self.xy_threshold
                   or abs(new_corr["dz"] - committed["dz"]) > self.z_threshold)
        if changed:
            self.static_corrections[gate_id] = new_corr
        return changed

    def reset(self):
        self.__init__(stable_frames=self.stable_frames,
                      xy_threshold=self.xy_threshold, z_threshold=self.z_threshold,
                      s_threshold=self.s_threshold, static_assoc_s=self.static_assoc_s,
                      static_assoc_xy=self.static_assoc_xy,
                      static_correction_max=self.static_correction_max)

"""Keep accepted course anchors across detector dropouts and tracker ID changes."""
import math


class OnlineGateCache:
    def __init__(self):
        self.gates = []
        self.by_track_id = {}
        self.next_id = 100000

    def _match(self, g):
        old = self.by_track_id.get(g.get("id"))
        if old is not None:
            return old
        matches = [old for old in self.gates if abs(old["s"] - g["s"]) < 5.0
                   and math.hypot(old["x"] - g["x"], old["y"] - g["y"]) < 5.0]
        if matches:
            return min(matches, key=lambda old: abs(old["s"] - g["s"]))
        return None

    def ingest(self, observations, static_s, s_now, now, project, completed):
        changed = False
        for observation in observations:
            g = dict(observation)
            obs_track_id = g.get("id")
            if not g.get("valid", True) or not g.get("hard_anchor", False):
                continue
            if not all(math.isfinite(float(g.get(k, float("nan")))) for k in ("x", "y", "z")):
                continue
            if now - g.get("last_seen", 0) > 1.0 or g.get("last_seen", 0) > now + .2:
                continue
            if g.get("support", 0) < 5 or max(g.get("sigma_x", 9), g.get("sigma_y", 9)) > 1.2 or g.get("sigma_z", 9) > .6:
                continue
            # Dense stereo is usable only after repeated, stable, confident observations.
            if not g.get("geometry_valid", False) and (g.get("support", 0) < 8 or (g.get("confidence") or 0) < .65):
                continue
            g["s"] = float(project(g))
            # 静态门附近的视觉锚点多为同一门的不同测量, 不重复入账。
            if any(abs(g["s"] - s) < 8.0 for s in static_s):
                continue
            old = self._match(g)
            if old is not None:
                if old["id"] in completed or old["s"] < s_now or g.get("last_seen", 0) <= old["last_seen"]:
                    continue
                for key in ("x", "y", "z", "s"):
                    g[key] = .7 * old[key] + .3 * g[key]
                g["id"] = old["id"]
                if obs_track_id is not None:
                    self.by_track_id[obs_track_id] = old
                old.update(g)
            else:
                if g["s"] <= s_now + 2.0:
                    continue
                g["track_id"] = obs_track_id
                g["id"] = self.next_id
                self.next_id += 1
                self.gates.append(g)
                if obs_track_id is not None:
                    self.by_track_id[obs_track_id] = g
            changed = True
        return changed

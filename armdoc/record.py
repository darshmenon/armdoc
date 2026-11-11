"""JSONL recorder for armdoc runs (ROS-free). One JSON object per line:
   {"type": "meta", ...} first, then {"type": "joints", "t", "pos"} and {"type": "findings", "t", "findings", "ctrl"}."""
import json
import time


class Recorder:
    def __init__(self, path, ns="", limits=None, joint_hz=10.0):
        self.f = open(path, "w", buffering=1)
        self.min_dt = 1.0 / joint_hz
        self.last_joint_t = -1e9
        self.has_limits = bool(limits)
        self.write({"type": "meta", "version": 1, "started": time.strftime("%Y-%m-%dT%H:%M:%S"), "ns": ns,
                    "limits": limits or {}})

    def set_limits(self, limits):
        """Limits arrive after start (from /robot_description): write them as a late meta record."""
        self.has_limits = True
        self.write({"type": "limits", "limits": limits})

    def write(self, rec):
        self.f.write(json.dumps(rec, separators=(",", ":")) + "\n")

    def joints(self, t, pos):
        """Record joint positions, throttled to joint_hz. `pos` is {joint: rad}."""
        if t - self.last_joint_t < self.min_dt or not pos:
            return
        self.last_joint_t = t
        self.write({"type": "joints", "t": round(t, 3), "pos": {k: round(v, 5) for k, v in pos.items() if v == v}})

    def findings(self, t, findings, controllers=None):
        self.write({"type": "findings", "t": round(t, 3),
                    "findings": [{"level": x.level, "code": x.code, "msg": x.msg} for x in findings],
                    "ctrl": {n: st for n, _typ, st in (controllers or [])}})

    def close(self):
        self.f.close()

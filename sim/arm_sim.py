#!/usr/bin/env python3
"""Mic arm geometry, clearance, motor and counterbalance calculator.

Run this BEFORE doing any CAD. Put your measurements in sim/config.json
(write a starter file with --write-config), then:

    python sim/arm_sim.py                  # full report, exit code 1 if any check fails
    python sim/arm_sim.py --sweep          # try many link lengths, list the ones that pass
    python sim/arm_sim.py --plot           # also save sim/out/arm_poses.png (needs matplotlib)

Only the standard library is needed for the report.

Frame (all lengths in mm, origin = shoulder pivot, which sits on the yaw axis):
  x  across the clamped edge, positive = away from the desk you clamp to
     (that edge is at x = -edge_offset)
  y  along the clamped edge
  z  up. The desk top is at z = -shoulder_above_desk
A simple desk is described by edge_offset / front_y / depth. Anything else (an L-shaped
desk, a second desk) is a list of rectangles in desk.boxes, each {"x": [min, max], "y": [min, max]}.

Joint angles (degrees):
  yaw  heading of the arm in the x-y plane (0 = +x, away from desk; 90 = +y, towards the user)
  sh   upper arm angle above horizontal
  el   forearm angle above horizontal. This is an ABSOLUTE angle: in a double-parallelogram
       arm the elbow motor sits at the base and sets the forearm angle directly.
The head stays level (parallelogram), and the mic hangs below the arm tip.
"""

import argparse
import json
import math
import sys
from copy import deepcopy
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONFIG_PATH = HERE / "config.json"
G = 9.81

DEFAULTS = {
    "geometry": {
        "upper_arm": 375,       # shoulder pivot -> elbow pivot (350 can't cover the tracking range)
        "forearm": 375,         # elbow pivot -> head pivot
        "head_offset_r": 30,    # head bracket sticks out this far past the forearm end
        "mic_drop": 120,        # arm tip -> capsule centre (mic hangs below the tip)
        "mic_hang": 220,        # arm tip -> lowest point of mic + shock mount
        "link_radius": 25,      # half-width of tubes/joint housings, used as clearance
        "turret_radius": 120,   # base turret (2 motors + pulleys) always shows above the desk
    },
    "desk": {
        "shoulder_above_desk": 80,  # shoulder pivot height above the desk top
        "edge_offset": 60,          # yaw axis distance outside the desk's side edge
        "thickness": 25,
        "top_height": 730,          # floor -> desk top
        "front_y": 200,             # clamp -> desk front edge, along the side edge
        "depth": 700,               # desk front-to-back depth
        "clearance_over_desk": 80,  # keep everything this high over the desk (keyboard, mug)
        "floor_clearance": 50,
        # "boxes": [{"x": [..], "y": [..]}, ...]  replaces the single desk above when given
    },
    # Extra things the arm must never touch: [{"name": "monitor", "x": [..], "y": [..], "z": [..]}]
    "keepouts": [],
    "user": {
        "mouth": [-500, 350, 340],     # mouth position in the frame above
        "approach": [0, -1, -0.25],    # direction from mouth towards the mic (towards monitor, a bit low)
        "distance": 150,               # mouth -> capsule distance to hold
        "track_range": 100,            # ToF tracking may move the capsule this far either way
        "head_radius": 110,            # keep-out sphere for your head, centred just behind the mouth
        "head_centre_offset": [0, 90, 30],
        # Keep-out for your body, chair and legs, measured from your mouth along the way you face.
        "body": {
            "back": 450,             # chair back is this far behind your mouth
            "front": 150,            # chest / arms in front of your mouth
            "half_width": 300,       # chair arms either side
            "below_mouth": 180,      # top of your shoulders below your mouth
            "legs_forward": 500,     # knees reach this far forward, under the desk
            "legs_half_width": 250,
        },
    },
    # Deploy runs through these in order, then to the taught target; stow runs it backwards.
    "waypoints": {
        "stowed": [90, -90, -180],  # hanging beside the desk, forearm folded back away from the user
        "tuck": [90, -90, -240],    # fold the elbow up first so the mic doesn't swing at the floor
        "clear": [90, 80, 30],      # lift, folded up tight above the turret, before swinging in
    },
    "checks": {
        "stow_below_desk": True,    # False when the arm parks on top of the desk instead
    },
    "limits": {
        "yaw": [0, 180],            # may run past 360 (e.g. [180, 400]) to swing the other way round
        "sh": [-95, 80],
        "el": [-250, 90],
        "fold": [5, 160],           # sh - el: how far the elbow can fold
        "wrist": [0, 180],          # only used when waypoints give a 4th (wrist) angle
        "max_reach_frac": 0.95,     # stay off a fully straight (singular) arm
    },
    "mass_kg": {
        "upper_arm": 0.18,   # tube + joints + push-rod share
        "forearm": 0.15,
        "payload": 0.80,     # C-1 + shock mount + pop filter + cable share + ToF head
    },
    "drive": {
        "ratio": 15,                 # 20T->60T then 16T->80T
        "efficiency": 0.85,
        "motor_holding_nm": 0.40,    # 17HS4401
        "run_current_frac": 0.7,     # fraction of rated current the TMC2209 runs at
        "pullout_factor": 0.5,       # torque left at speed vs holding torque (conservative)
        "friction_nm": 0.2,          # at the joint: belts, bearings, cable drag
        "motor_max_rps": 5.0,        # a 17HS4401 on 24 V stays in quiet StealthChop up to ~5 rev/s
        "steps_per_rev": 200,
        "microsteps": 8,             # the TMC2209 interpolates to 256 internally
        "far_speed": 600,            # mm/s, more than 250 mm from your mouth
        "tip_speed": 150,            # mm/s, within 250 mm of your mouth
        "approach_speed": 75,        # mm/s, last 100 mm before the target (tracking nudges use 50)
        "tip_accel": 400,            # mm/s^2, gentle = quieter
        "encoder_bits": 12,          # AS5600 on the joint output
        "min_margin": 2.0,           # required motor torque margin
    },
    # Optional flip-up wrist (waypoints with a 4th angle): folds the mic up for a compact park.
    "wrist": {
        "ratio": 27,          # NEMA11 stepper + 27:1 planetary gearbox
        "holding_nm": 0.12,   # NEMA11, 45 mm body
        "mic_kg": 0.60,       # what the wrist swings: C-1 + shock mount + pop filter
        "friction_nm": 0.05,
        "balance_frac": 0.85, # small spring on the wrist; under-balanced so the mic still hangs by itself
    },
    "spring": {
        "balance_frac": 0.92,  # slightly under-balanced: power loss -> arm sinks away from you
        "a_mm": 50,            # cord pulley/anchor distance straight above the pivot
        "b_mm": 80,            # cord attachment distance out along the link/crank
    },
}


def deep_merge(base, over):
    out = deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path):
    if not path.exists():
        return deepcopy(DEFAULTS)
    user = json.loads(path.read_text())
    cfg = deep_merge(DEFAULTS, user)
    if "waypoints" in user:  # a route is a whole sequence: yours replaces the default one
        cfg["waypoints"] = user["waypoints"]
    return cfg


# ---------------------------------------------------------------- vectors

def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(a, s):
    return tuple(x * s for x in a)


def dist(a, b):
    return math.dist(a, b)


def unit(a):
    n = math.hypot(*a)
    return scale(a, 1 / n)


def lerp(a, b, t):
    return tuple(x + (y - x) * t for x, y in zip(a, b))


# ---------------------------------------------------------------- kinematics

class Arm:
    def __init__(self, cfg):
        g = cfg["geometry"]
        self.L1, self.L2 = g["upper_arm"], g["forearm"]
        self.head_r = g["head_offset_r"]
        self.mic_drop, self.mic_hang = g["mic_drop"], g["mic_hang"]

    def points(self, pose):
        """Key points (x, y, z) for a joint pose (yaw, sh, el[, wrist]) in degrees.

        The optional wrist flips the mic in the arm's plane: 0 = hanging straight down (live),
        180 = pointing straight up, folded back over the arm (parked).
        """
        yaw, sh, el = (math.radians(a) for a in pose[:3])
        wrist = math.radians(pose[3]) if len(pose) > 3 else 0.0
        er, ez = self.L1 * math.cos(sh), self.L1 * math.sin(sh)
        fr, fz = er + self.L2 * math.cos(el), ez + self.L2 * math.sin(el)
        tr = fr + self.head_r
        dr, dz = -math.sin(wrist), -math.cos(wrist)  # direction from the tip along the mic

        def world(r, z):
            return (r * math.cos(yaw), r * math.sin(yaw), z)

        return {
            "shoulder": (0.0, 0.0, 0.0),
            "elbow": world(er, ez),
            "forearm_end": world(fr, fz),
            "tip": world(tr, fz),
            "capsule": world(tr + dr * self.mic_drop, fz + dz * self.mic_drop),
            "mic_bottom": world(tr + dr * self.mic_hang, fz + dz * self.mic_hang),  # far end of the mic
        }

    def reach_frac(self, tip):
        x, y, z = tip
        return math.hypot(math.hypot(x, y) - self.head_r, z) / (self.L1 + self.L2)

    def ik(self, tip, yaw_min=None):
        """Elbow-up inverse kinematics for an arm-tip position. None if out of reach.

        yaw_min puts the yaw angle in the window [yaw_min, yaw_min + 360).
        """
        x, y, z = tip
        yaw = math.atan2(y, x)
        r = math.hypot(x, y) - self.head_r
        d = math.hypot(r, z)
        if d > self.L1 + self.L2 or d < abs(self.L1 - self.L2) or d == 0:
            return None
        sh = math.atan2(z, r) + math.acos((self.L1**2 + d**2 - self.L2**2) / (2 * self.L1 * d))
        el = math.atan2(z - self.L1 * math.sin(sh), r - self.L1 * math.cos(sh))
        yaw, sh, el = (math.degrees(a) for a in (yaw, sh, el))
        if yaw_min is not None:
            yaw = (yaw - yaw_min) % 360 + yaw_min
        return yaw, sh, el

    def tip_for_capsule(self, capsule):
        return add(capsule, (0, 0, self.mic_drop))


def sample_body(pts, n=12):
    """Points along the upper arm, forearm, head bracket and hanging mic."""
    chain = [("shoulder", "elbow"), ("elbow", "forearm_end"), ("forearm_end", "tip"), ("tip", "mic_bottom")]
    for a, b in chain:
        for i in range(n + 1):
            yield lerp(pts[a], pts[b], i / n)


# ---------------------------------------------------------------- environment

class Desk:
    def __init__(self, cfg):
        d = cfg["desk"]
        self.rad = cfg["geometry"]["link_radius"]
        self.top = -d["shoulder_above_desk"]
        self.under = self.top - d["thickness"]
        self.floor = self.top - d["top_height"]
        self.edge_x = -d["edge_offset"]
        self.front_y = d["front_y"]
        self.back_y = d["front_y"] - d["depth"]
        self.over_clear = d["clearance_over_desk"]
        self.floor_clear = d["floor_clearance"]
        if d.get("boxes"):
            self.boxes = [(tuple(b["x"]), tuple(b["y"])) for b in d["boxes"]]
        else:
            self.boxes = [((-1e5, self.edge_x), (self.back_y, self.front_y))]
        self.keepouts = [(k.get("name", "keep-out"), k["x"], k["y"], k["z"]) for k in cfg.get("keepouts", [])]

    def over_desk(self, x, y):
        r = self.rad
        return any(bx[0] - r < x < bx[1] + r and by[0] - r < y < by[1] + r for bx, by in self.boxes)

    def problem(self, p):
        """Name of whatever the point (with link radius) would hit, or None."""
        x, y, z = p
        r = self.rad
        if z - r < self.floor + self.floor_clear:
            return "floor"
        if self.over_desk(x, y) and not (z - r > self.top + self.over_clear or z + r < self.under):
            return "desk"
        for name, kx, ky, kz in self.keepouts:
            if kx[0] - r < x < kx[1] + r and ky[0] - r < y < ky[1] + r and kz[0] - r < z < kz[1] + r:
                return name
        return None


# ---------------------------------------------------------------- analysis

def within(v, lim):
    return lim[0] <= v <= lim[1]


def limit_problems(pose, lim):
    yaw, sh, el = pose[:3]
    out = []
    joints = [("yaw", yaw), ("sh", sh), ("el", el), ("fold", sh - el)]
    if len(pose) > 3:
        joints.append(("wrist", pose[3]))
    for name, v in joints:
        if not within(v, lim[name]):
            out.append(f"{name}={v:.1f} outside {lim[name]}")
    return out


def gravity_amplitudes(cfg):
    """Peak gravity torque (N.m) on shoulder and elbow drives; actual = amplitude * cos(angle).

    With absolute joint angles the two terms decouple, which is what lets a simple
    zero-free-length spring balance each axis perfectly at every angle.
    The head bracket's constant moment goes into the levelling links, not the motors.
    """
    m, g = cfg["mass_kg"], cfg["geometry"]
    L1, L2 = g["upper_arm"] / 1000, g["forearm"] / 1000
    a_sh = G * (m["upper_arm"] * L1 / 2 + (m["forearm"] + m["payload"]) * L1)
    a_el = G * (m["forearm"] * L2 / 2 + m["payload"] * L2)
    return a_sh, a_el


def inertias(cfg):
    """Worst-case (arm straight out) inertia about each axis, kg.m^2."""
    m, g = cfg["mass_kg"], cfg["geometry"]
    L1, L2 = g["upper_arm"] / 1000, g["forearm"] / 1000
    reach = L1 + L2 + g["head_offset_r"] / 1000
    i_sh = m["upper_arm"] * L1**2 / 3 + m["forearm"] * (L1 + L2 / 2) ** 2 + m["payload"] * reach**2
    i_el = m["forearm"] * L2**2 / 3 + m["payload"] * (L2 + g["head_offset_r"] / 1000) ** 2
    return {"yaw": i_sh, "sh": i_sh, "el": i_el}, reach


def motor_out_torque(cfg):
    d = cfg["drive"]
    return d["motor_holding_nm"] * d["run_current_frac"] * d["pullout_factor"] * d["ratio"] * d["efficiency"]


def path_poses(a, b, n=200):
    return [lerp(a, b, i / n) for i in range(n + 1)]


def move_timing(arm, poses, cfg, mouth, target):
    """Time a joint-space move that stops at both ends.

    Speed along the path is capped by the zone (far / near your mouth / final approach),
    by the motor-speed cap, and by the accel ramps from the start and into the end.
    """
    d = cfg["drive"]
    w_arm = d["motor_max_rps"] * 2 * math.pi / d["ratio"]  # joint rad/s
    w_wrist = d["motor_max_rps"] * 2 * math.pi / cfg["wrist"]["ratio"]
    steps = []  # (path length of the fastest point, zone speed cap, joint changes)
    prev = arm.points(poses[0])
    for p0, p1 in zip(poses, poses[1:]):
        pts = arm.points(p1)
        ds = max(dist(pts["tip"], prev["tip"]), dist(pts["mic_bottom"], prev["mic_bottom"]))
        v = d["far_speed"]
        if min(dist(p, mouth) for p in (pts["tip"], pts["capsule"], pts["mic_bottom"])) < 250:
            v = d["tip_speed"]
        if dist(pts["capsule"], target) < 100:
            v = d["approach_speed"]
        steps.append((ds, v, [abs(math.radians(b - a)) for a, b in zip(p0, p1)]))
        prev = pts
    total = sum(s[0] for s in steps)
    t, peak_rps, s = 0.0, 0.0, 0.0
    for ds, v, dqs in steps:
        mid = s + ds / 2
        v = min(v, math.sqrt(2 * d["tip_accel"] * max(min(mid, total - mid), 1e-3)))
        dq_arm, dq_wrist = max(dqs[:3]), (dqs[3] if len(dqs) > 3 else 0.0)
        dt = max(ds / v, dq_arm / w_arm, dq_wrist / w_wrist, 1e-6)
        peak_rps = max(peak_rps, dq_arm / dt * d["ratio"] / (2 * math.pi))
        t += dt
        s += ds
    return t, peak_rps


def analyse(cfg):
    arm, desk = Arm(cfg), Desk(cfg)
    lim, drv, spr, usr = cfg["limits"], cfg["drive"], cfg["spring"], cfg["user"]
    checks, info = [], {}

    def check(name, ok, detail):
        checks.append((name, bool(ok), detail))

    # --- target and tracking poses
    u = unit(usr["approach"])
    capsule = add(usr["mouth"], scale(u, usr["distance"]))
    targets = {
        "target": capsule,
        "lean_back": add(capsule, scale(u, -usr["track_range"])),  # you sit back, mic follows you
        "lean_in": add(capsule, scale(u, usr["track_range"])),     # you lean in, mic backs off
    }
    poses = {name: tuple(p) for name, p in cfg["waypoints"].items()}
    path = list(cfg["waypoints"]) + ["target"]
    reach_used = 0.0
    unreachable = []
    for name, cap in targets.items():
        tip = arm.tip_for_capsule(cap)
        reach_used = max(reach_used, arm.reach_frac(tip))
        sol = arm.ik(tip, lim["yaw"][0])
        if sol is None:
            unreachable.append(name)
        else:
            poses[name] = sol
    # The firmware drops the mic onto the target from 100 mm above when it can (ABOVE_TARGET_MM).
    if "target" not in unreachable:
        above = arm.ik(add(arm.tip_for_capsule(capsule), (0, 0, 100)), lim["yaw"][0])
        if above and not limit_problems(above, lim):
            poses["above"] = above
            path.insert(-1, "above")
    info["poses"], info["capsule"] = poses, capsule
    # Firmware SET aim=: which way the mic faces (towards your mouth) relative to the arm, seen from above.
    info["aim_deg"] = (math.degrees(math.atan2(-u[1], -u[0])) - poses.get("target", (0,))[0] + 180) % 360 - 180

    check("target reachable", "target" not in unreachable and arm.reach_frac(arm.tip_for_capsule(capsule)) <= lim["max_reach_frac"],
          f"uses {arm.reach_frac(arm.tip_for_capsule(capsule)):.0%} of full reach (limit {lim['max_reach_frac']:.0%})")
    check(f"tracking +/-{usr['track_range']} mm reachable", not unreachable and reach_used <= lim["max_reach_frac"],
          f"worst case uses {reach_used:.0%} of full reach" + (f"; unreachable: {', '.join(unreachable)}" if unreachable else ""))
    if "target" in unreachable:
        return checks, info

    # A wrist angle in any waypoint means every pose has one; the live poses hang the mic (0).
    n = max(len(p) for p in poses.values())
    poses = {k: tuple(p) + (0.0,) * (n - len(p)) for k, p in poses.items()}
    info["poses"] = poses

    # --- joint limits
    bad =[f"{n}: {p}" for n, pose in poses.items() for p in limit_problems(pose, lim)]
    check("joint limits", not bad, "; ".join(bad) if bad else "all poses inside limits")

    # --- collisions along deploy path and tracking range
    segments = list(zip(path, path[1:]))
    if "lean_back" in poses and "lean_in" in poses:
        segments.append(("lean_back", "lean_in"))
    mouth = tuple(usr["mouth"])
    head = add(mouth, usr["head_centre_offset"])

    body = usr.get("body")
    face = unit((u[0], u[1]))  # the way you face, seen from above

    def in_body(p):
        rx, ry = p[0] - mouth[0], p[1] - mouth[1]
        along, side = rx * face[0] + ry * face[1], abs(-rx * face[1] + ry * face[0])
        r, z = desk.rad, p[2]
        if along < -body["back"] - r:
            return None
        if along < body["front"] + r and side < body["half_width"] + r and z - r < mouth[2] - body["below_mouth"]:
            return "your body / chair"
        if along < body["legs_forward"] + r and side < body["legs_half_width"] + r and z - r < desk.top:
            return "your legs"
        return None

    def problem(p, seg):
        if seg[0] != "lean_back":  # you move with the tracking range, so only check the deploy path
            if dist(p, head) < usr["head_radius"] + desk.rad:
                return "your head"
            hit = body and in_body(p)
            if hit:
                return hit
        return desk.problem(p)

    hits = []
    for a, b in segments:
        for pose in path_poses(poses[a], poses[b]):
            hit = next((problem(p, (a, b)) for p in sample_body(arm.points(pose)) if problem(p, (a, b))), None)
            if hit:
                hits.append(f"{a}->{b} hits {hit} near pose ({pose[0]:.0f}, {pose[1]:.0f}, {pose[2]:.0f})")
                break
            lp = limit_problems(pose, lim)
            if lp:
                hits.append(f"{a}->{b} passes outside limits ({lp[0]})")
                break
    check("path clears desk, floor, you and keep-outs", not hits,
          "; ".join(hits) if hits else "deploy path and tracking range are clear")

    # --- stowed pose hidden
    turret = cfg["geometry"]["turret_radius"]
    stow_pts = [p for p in sample_body(arm.points(poses[path[0]])) if math.hypot(*p) > turret]
    highest = max(p[2] for p in stow_pts) + desk.rad
    lowest = min(p[2] for p in stow_pts) - desk.rad
    hidden = highest < desk.top and lowest > desk.floor + desk.floor_clear
    above = highest - desk.top
    info["parked_height_mm"] = above
    if cfg["checks"]["stow_below_desk"]:
        check("stowed pose hidden below desk top", hidden,
              (f"pokes {above:.0f} mm above the desk top" if above > 0 else f"top of the arm is {-above:.0f} mm below the desk top")
              + f" (turret excluded), lowest point {lowest - desk.floor:.0f} mm above the floor")

    # --- torque, springs, motors
    a_sh, a_el = gravity_amplitudes(cfg)
    inert, reach_m = inertias(cfg)
    alpha = drv["tip_accel"] / 1000 / reach_m
    avail = motor_out_torque(cfg)
    resid = 1 - spr["balance_frac"]
    a_m, b_m = spr["a_mm"] / 1000, spr["b_mm"] / 1000
    info["springs"], info["motors"] = {}, {}
    for axis, amp in (("sh", a_sh), ("el", a_el), ("yaw", 0.0)):
        need = resid * amp + drv["friction_nm"] + inert[axis] * alpha
        need_bare = amp + drv["friction_nm"] + inert[axis] * alpha
        margin = avail / need
        info["motors"][axis] = {"need": need, "margin": margin, "margin_no_spring": avail / need_bare}
        check(f"{axis} motor torque margin", margin >= drv["min_margin"],
              f"{margin:.1f}x (needs >= {drv['min_margin']}x): {avail:.2f} N.m available, {need:.2f} N.m needed")
        if amp:
            k = spr["balance_frac"] * amp / (a_m * b_m)  # N/m
            info["springs"][axis] = {
                "amp": amp, "k_n_per_mm": k / 1000,
                "f_min": k * abs(a_m - b_m), "f_max": k * (a_m + b_m),
                "stroke_mm": 2 * min(spr["a_mm"], spr["b_mm"]),
            }

    # --- flip-up wrist: it lifts the mic on a lever of about mic_drop
    if len(poses["target"]) > 3:
        w = cfg["wrist"]
        need = (1 - w["balance_frac"]) * w["mic_kg"] * G * cfg["geometry"]["mic_drop"] / 1000 + w["friction_nm"]
        avail_w = w["holding_nm"] * drv["run_current_frac"] * drv["pullout_factor"] * w["ratio"] * drv["efficiency"]
        info["motors"]["wrist"] = {"need": need, "margin": avail_w / need, "margin_no_spring": avail_w / need}
        check("wrist motor torque margin", avail_w / need >= drv["min_margin"],
              f"{avail_w / need:.1f}x (needs >= {drv['min_margin']}x): {avail_w:.2f} N.m available, {need:.2f} N.m needed")

    # --- timing, resolution, clamp load
    legs = [move_timing(arm, path_poses(poses[a], poses[b]), cfg, mouth, capsule) for a, b in zip(path, path[1:])]
    peak = max(rps for _, rps in legs)
    m = cfg["mass_kg"]
    info["path"] = path
    info["timing"] = {"deploy_s": sum(t for t, _ in legs), "legs": [t for t, _ in legs], "peak_rps": peak,
                      "peak_steps_s": peak * drv["steps_per_rev"] * drv["microsteps"]}
    info["tip_res_mm"] = 2 * math.pi / 2 ** drv["encoder_bits"] * reach_m * 1000
    info["clamp_nm"] = G * (m["upper_arm"] * reach_m * 0.25 + m["forearm"] * reach_m * 0.75 + m["payload"] * reach_m)
    return checks, info


# ---------------------------------------------------------------- output

def fmt(p):
    return "(" + ", ".join(f"{v:7.1f}" for v in p) + ")"


def report(cfg, checks, info):
    g, arm = cfg["geometry"], Arm(cfg)
    print(f"== Mic arm sim ==  links {g['upper_arm']} + {g['forearm']} mm, payload {cfg['mass_kg']['payload']:.2f} kg\n")
    print("Poses               yaw / shoulder / elbow (deg)        arm tip x / y / z (mm)")
    for name, pose in info["poses"].items():
        print(f"  {name:<12} {fmt(pose)}   {fmt(arm.points(pose)['tip'])}")
    print("\nChecks")
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    if "springs" in info:
        s = cfg["spring"]
        print(f"\nCounterbalance  (zero-free-length spring, cord pulley {s['a_mm']} mm above the pivot,"
              f" attached {s['b_mm']} mm out, {s['balance_frac']:.0%} balanced)")
        for axis, sp in info["springs"].items():
            print(f"  {axis:<3} gravity peak {sp['amp']:.2f} N.m -> spring rate {sp['k_n_per_mm']:.2f} N/mm,"
                  f" force {sp['f_min']:.0f}-{sp['f_max']:.0f} N over {sp['stroke_mm']:.0f} mm of cord travel")
        no_spring = info["motors"]["sh"]["margin_no_spring"]
        print(f"  Without springs the shoulder margin would be {no_spring:.1f}x"
              + (" -> the springs are essential." if no_spring < cfg["drive"]["min_margin"] else "."))
        t = info["timing"]
        print("\nMotion")
        legs = ", ".join(f"{a}->{b} {lt:.1f} s" for a, b, lt in zip(info["path"], info["path"][1:], t["legs"]))
        print(f"  deploy path: {legs}")
        print(f"  deploy time ~{t['deploy_s']:.1f} s, peak motor speed {t['peak_rps']:.2f} rev/s"
              f" ({t['peak_steps_s']:.0f} steps/s at {cfg['drive']['microsteps']} microsteps)")
        print(f"  encoder resolution at the tip ~{info['tip_res_mm']:.1f} mm")
        print(f"  firmware tracking direction: send  SET aim={info['aim_deg']:.0f}")
        print(f"  clamp load: up to {info['clamp_nm']:.1f} N.m of tipping moment -> use a clamp rated"
              f" for at least {3 * info['clamp_nm']:.0f} N.m (a monitor-arm clamp is fine)")
    failed = sum(not ok for _, ok, _ in checks)
    print(f"\n{'ALL CHECKS PASS' if not failed else f'{failed} CHECK(S) FAILED'}")
    return failed


def sweep(cfg):
    rows = []
    lengths = range(250, 525, 25)
    for l1 in lengths:
        for l2 in lengths:
            c = deepcopy(cfg)
            c["geometry"]["upper_arm"], c["geometry"]["forearm"] = l1, l2
            checks, info = analyse(c)
            if all(ok for _, ok, _ in checks):
                worst = min(v["margin"] for v in info["motors"].values())
                rows.append((l1 + l2, -worst, l1, l2, worst, info["timing"]["deploy_s"]))
    if not rows:
        print("No link lengths between 250 and 500 mm pass. Check the failing items with a normal run.")
        return 1
    rows.sort()
    print("Link lengths that pass every check (shortest first):")
    print("  upper  forearm  worst motor margin  deploy time")
    for _, _, l1, l2, worst, t in rows[:12]:
        print(f"  {l1:5d}  {l2:7d}  {worst:17.1f}x  {t:9.1f} s")
    return 0


def plot(cfg, info, path):
    import matplotlib.pyplot as plt  # optional dependency

    arm, desk = Arm(cfg), Desk(cfg)
    colours = {"stowed": "tab:gray", "tuck": "tab:brown", "clear": "tab:orange", "above": "tab:olive", "target": "tab:green",
               "lean_back": "tab:blue", "lean_in": "tab:purple"}
    views = [("Top view (x-y)", 0, 1), ("Front view (x-z)", 0, 2), ("Side view (y-z)", 1, 2)]
    fig, axes = plt.subplots(1, 3, figsize=(17, 6))
    for ax, (title, i, j) in zip(axes, views):
        for k, (bx, by) in enumerate(desk.boxes):
            span = {0: (max(bx[0], -1500), bx[1]), 1: by, 2: (desk.under, desk.top)}
            ax.add_patch(plt.Rectangle((span[i][0], span[j][0]), span[i][1] - span[i][0], span[j][1] - span[j][0],
                                       color="burlywood", alpha=0.4, label="desk" if k == 0 else None))
        for name, kx, ky, kz in desk.keepouts:
            span = {0: kx, 1: ky, 2: kz}
            ax.add_patch(plt.Rectangle((span[i][0], span[j][0]), span[i][1] - span[i][0], span[j][1] - span[j][0],
                                       color="tab:red", alpha=0.2))
        if j == 2:
            ax.axhline(desk.floor, color="k", lw=1)
        for a, b in zip(info["path"], info["path"][1:]):
            tips = [arm.points(p)["tip"] for p in path_poses(info["poses"][a], info["poses"][b])]
            ax.plot([t[i] for t in tips], [t[j] for t in tips], ":", color="k", lw=1)
        for name, pose in info["poses"].items():
            pts = arm.points(pose)
            chain = [pts[k] for k in ("shoulder", "elbow", "forearm_end", "tip", "mic_bottom")]
            ax.plot([p[i] for p in chain], [p[j] for p in chain], "-o", ms=3, color=colours.get(name, "k"), label=name)
        mouth = cfg["user"]["mouth"]
        ax.plot(mouth[i], mouth[j], "r*", ms=12, label="mouth")
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
    axes[0].legend(loc="best", fontsize=8)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"\nSaved {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=CONFIG_PATH, help="JSON with your measurements (default sim/config.json)")
    ap.add_argument("--write-config", action="store_true", help="write the default config to --config and exit")
    ap.add_argument("--sweep", action="store_true", help="search link lengths 250-500 mm")
    ap.add_argument("--plot", action="store_true", help="save sim/out/arm_poses.png (needs matplotlib)")
    args = ap.parse_args()

    if args.write_config:
        if args.config.exists():
            sys.exit(f"{args.config} already exists, not overwriting")
        args.config.write_text(json.dumps(DEFAULTS, indent=2) + "\n")
        print(f"Wrote {args.config}; edit it with your measurements")
        return 0

    cfg = load_config(args.config)
    if args.sweep:
        return sweep(cfg)
    checks, info = analyse(cfg)
    failed = report(cfg, checks, info)
    if args.plot:
        try:
            plot(cfg, info, HERE / "out" / "arm_poses.png")
        except ImportError:
            print("\n--plot needs matplotlib: pip install matplotlib")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())

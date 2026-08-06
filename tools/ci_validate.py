#!/usr/bin/env python3
"""CI validation for GarudaNEX: syntax, manifests, and tuned-parameter guards.

Runs without ROS installed. Fails loudly if a navigation parameter drifts back
into a configuration that is known to break the system.
"""
import ast
import glob
import pathlib
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
fails = []


def ok(msg):
    print("  ok   %s" % msg)


def bad(msg):
    print("  FAIL %s" % msg)
    fails.append(msg)


print("== python syntax ==")
pys = sorted(glob.glob(str(ROOT / "src/**/*.py"), recursive=True))
pys += sorted(glob.glob(str(ROOT / "tools/**/*.py"), recursive=True))
pys = [p for p in pys if "px4_msgs" not in p]
for f in pys:
    try:
        ast.parse(pathlib.Path(f).read_text())
    except SyntaxError as e:
        bad("%s: %s" % (pathlib.Path(f).relative_to(ROOT), e))
ok("%d python files parsed" % len(pys))

print("== package manifests ==")
mans = sorted(glob.glob(str(ROOT / "src/*/package.xml")))
for f in mans:
    try:
        r = ET.parse(f).getroot()
        if r.find("name") is None or r.find("version") is None:
            bad("%s missing <name> or <version>" % f)
    except ET.ParseError as e:
        bad("%s: %s" % (f, e))
ok("%d package.xml parsed" % len(mans))

print("== world sdf ==")
worlds = sorted(glob.glob(str(ROOT / "src/garudanex_sim/worlds/*.sdf")))
for f in worlds:
    try:
        ET.parse(f)
    except ET.ParseError as e:
        bad("%s: %s" % (f, e))
if not worlds:
    bad("no world files found")
ok("%d world files parsed" % len(worlds))

print("== tuned navigation parameters ==")
NAV = ROOT / "src/garudanex_navigation/config/nav2_uav.yaml"
try:
    cfg = yaml.safe_load(NAV.read_text())
except Exception as e:
    bad("cannot parse nav2_uav.yaml: %s" % e)
    cfg = None

if cfg:
    def ros(name):
        return cfg.get(name, {}).get(name, {}).get("ros__parameters", {})

    lc = ros("local_costmap")
    gc = ros("global_costmap")
    cs = cfg.get("controller_server", {}).get("ros__parameters", {})
    ps = cfg.get("planner_server", {}).get("ros__parameters", {})
    fp = cs.get("FollowPath", {})
    gb = ps.get("GridBased", {})

    rr = lc.get("robot_radius")
    inf = (lc.get("inflation_layer") or {}).get("inflation_radius")
    if rr is None or inf is None:
        bad("robot_radius / inflation_radius not found in local_costmap")
    elif inf <= rr:
        bad("inflation_radius (%.2f) must exceed robot_radius (%.2f) or MPPI "
            "has no cost gradient and the drone clips walls" % (inf, rr))
    else:
        ok("inflation_radius %.2f > robot_radius %.2f" % (inf, rr))

    au = gb.get("allow_unknown")
    if au is None:
        bad("GridBased.allow_unknown not set")
    elif au is not False:
        bad("GridBased.allow_unknown must be false, else the planner routes "
            "through unmapped space into sealed rooms")
    else:
        ok("GridBased.allow_unknown is false")

    vx = fp.get("vx_max")
    if vx is None:
        bad("FollowPath.vx_max not set")
    elif vx > 1.5:
        bad("FollowPath.vx_max %.2f is above the 1.5 m/s ceiling that caused "
            "wall contact in 1.6 m doorways" % vx)
    else:
        ok("FollowPath.vx_max %.2f within safe ceiling" % vx)

    mm = fp.get("motion_model")
    if mm != "Omni":
        bad("FollowPath.motion_model must be Omni for a multirotor (got %r)" % mm)
    else:
        ok("FollowPath.motion_model is Omni")

    rbf = lc.get("robot_base_frame")
    if rbf != "base_footprint":
        bad("local_costmap.robot_base_frame must be base_footprint (got %r)" % rbf)
    else:
        ok("robot_base_frame is base_footprint")

    for name, r in (("global", gc), ("local", lc)):
        gi = (r.get("inflation_layer") or {}).get("inflation_radius")
        if gi is None:
            bad("%s_costmap inflation_radius missing" % name)

print()
if fails:
    print("VALIDATION FAILED - %d problem(s)" % len(fails))
    for f in fails:
        print("  - %s" % f)
    sys.exit(1)
print("ALL CHECKS PASSED")

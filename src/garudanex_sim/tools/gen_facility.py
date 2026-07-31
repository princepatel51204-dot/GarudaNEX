#!/usr/bin/env python3
"""Generate garudanex_facility.sdf - GPS-denied multi-zone facility.

Built for autonomous frontier exploration:
  60 x 40 m, 14 rooms off a central spine corridor
  perimeter corridor closes a loop      -> SLAM loop closure
  1.6 m doorways                        -> narrow-gap navigation
  two sealed store rooms                -> frontier blacklisting / dead ends
  clutter straddling the 1.5 m slice    -> some obstacles never enter the map
Primitives only: real-time factor ~1.0, which autonomy requires.
"""
import os, math

PX4 = os.path.expanduser('~/PX4-Autopilot')
PKG = os.path.expanduser('~/GarudaNEX/ros2_ws/src/garudanex_sim')
SRC = os.path.join(PX4, 'Tools/simulation/gz/worlds/default.sdf')
OUT = os.path.join(PKG, 'worlds', 'garudanex_facility.sdf')
SVG = os.path.join(PKG, 'docs', 'facility_floorplan.svg')

HX, HY   = 30.0, 20.0     # interior half-extents
WT, WH   = 0.25, 4.0      # wall thickness / height
SPINE    = 2.5            # central corridor half-width
ROOM_Y   = 17.0           # rooms end here, leaving a perimeter corridor
DOOR     = 1.6            # doorway width
CRUISE   = 1.5            # LiDAR slice altitude
CLEAR_R  = 2.0            # keep-clear radius around spawn (corridor is 2.5 m half-width)

B = []
def box(g, n, cx, cy, sx, sy, sz=WH):
    B.append(dict(g=g, n=n, cx=cx, cy=cy, sx=sx, sy=sy, sz=sz))

def wall_x(g, tag, y, x0, x1, gaps):
    """Wall running along X at fixed y, with gaps (list of gap centres)."""
    edges = [x0]
    for c in sorted(gaps):
        edges += [c - DOOR / 2, c + DOOR / 2]
    edges.append(x1)
    for i in range(0, len(edges) - 1, 2):
        a, b = edges[i], edges[i + 1]
        if b - a > 0.05:
            box(g, '%s_%d' % (tag, i), (a + b) / 2, y, b - a, WT)

def wall_y(g, tag, x, y0, y1, gaps):
    """Wall running along Y at fixed x, with gaps (list of gap centres)."""
    edges = [y0]
    for c in sorted(gaps):
        edges += [c - DOOR / 2, c + DOOR / 2]
    edges.append(y1)
    for i in range(0, len(edges) - 1, 2):
        a, b = edges[i], edges[i + 1]
        if b - a > 0.05:
            box(g, '%s_%d' % (tag, i), x, (a + b) / 2, WT, b - a)

# ---- outer shell -----------------------------------------------------------
box('shell', 'north',  0.0,  HY, 2 * HX + WT, WT)
box('shell', 'south',  0.0, -HY, 2 * HX + WT, WT)
box('shell', 'east',    HX, 0.0, WT, 2 * HY + WT)
box('shell', 'west',   -HX, 0.0, WT, 2 * HY + WT)

# ---- spine corridor walls, one doorway per room ---------------------------
CENTRES = [-26.0, -18.0, -10.0, 0.0, 10.0, 18.0, 26.0]
PARTS   = [-22.0, -14.0, -6.0, 6.0, 14.0, 22.0]
wall_x('spine', 'spine_n',  SPINE, -HX, HX, CENTRES)
wall_x('spine', 'spine_s', -SPINE, -HX, HX, CENTRES)

# ---- room partitions; gap near the far end creates the loop ---------------
# two partitions are solid -> the rooms they seal become dead ends
SEALED = {-22.0, 14.0}
for i, px in enumerate(PARTS):
    gaps = [] if px in SEALED else [15.0]
    wall_y('rooms_n', 'pn%d' % i, px,  SPINE, ROOM_Y, gaps)
    wall_y('rooms_s', 'ps%d' % i, px, -ROOM_Y, -SPINE, gaps)

# ---- room back walls, leaving a perimeter corridor -> closes the loop ------
wall_x('rooms_n', 'backn',  ROOM_Y, -HX, HX, [-26.0, 10.0, 26.0])
wall_x('rooms_s', 'backs', -ROOM_Y, -HX, HX, [-26.0, 10.0, 26.0])

# ---- clutter: tall (mappable) and low (invisible to the 2D slice) ---------
tall, low = [], []
for i, cx in enumerate(CENTRES):
    for sgn in (1, -1):
        tall.append((cx + 2.2 * (1 if i % 2 else -1), sgn * (ROOM_Y - 4.0)))
        low.append((cx - 2.0, sgn * (SPINE + 3.5)))
for i, (x, y) in enumerate(tall):
    box('crates', 'crate_%d' % i, x, y, 1.4, 1.4, 2.6)
for i, (x, y) in enumerate(low):
    box('pallets', 'pallet_%d' % i, x, y, 1.2, 1.0, 1.1)
for i, px in enumerate([-18.0, -6.0, 6.0, 18.0]):
    box('columns', 'col_%d' % i, px, 0.0, 0.4, 0.4, WH)

# ---- guard: spawn must be clear -------------------------------------------
bad = []
for b in B:
    if b['g'] == 'shell':
        continue
    dx = max(abs(b['cx']) - b['sx'] / 2, 0.0)
    dy = max(abs(b['cy']) - b['sy'] / 2, 0.0)
    if math.hypot(dx, dy) < CLEAR_R:
        bad.append('%s (%.1f m)' % (b['n'], math.hypot(dx, dy)))
assert not bad, 'inside %.1f m spawn clearance: %s' % (CLEAR_R, bad)

COL = {'shell': '0.55 0.55 0.58', 'spine': '0.50 0.52 0.56',
       'rooms_n': '0.62 0.58 0.52', 'rooms_s': '0.62 0.58 0.52',
       'crates': '0.66 0.48 0.28', 'pallets': '0.72 0.62 0.40',
       'columns': '0.35 0.35 0.38'}

LINK = """    <link name="{n}">
      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
      <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
      <visual name="v">
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material>
      </visual>
    </link>
"""

groups = {}
for b in B:
    groups.setdefault(b['g'], []).append(b)
blob = ''
for g, items in groups.items():
    blob += '\n    <model name="%s">\n    <static>true</static>\n' % g
    for b in items:
        blob += LINK.format(cz=b['sz'] / 2, c=COL.get(g, '0.6 0.6 0.6'), **b)
    blob += '    </model>\n'

sdf = open(SRC).read()
assert '<world name="default">' in sdf, 'unexpected default.sdf'
sdf = sdf.replace('<world name="default">', '<world name="garudanex_facility">', 1)
i = sdf.rfind('</world>')
os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, 'w').write(sdf[:i] + blob + sdf[i:])

# ---- floor plan ------------------------------------------------------------
S, M = 13.0, 40.0
W, H = 2 * HX * S + 2 * M, 2 * HY * S + 2 * M + 60
sx_ = lambda x: (x + HX) * S + M
sy_ = lambda y: (HY - y) * S + M
o = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.0f" height="%.0f" '
     'font-family="Helvetica,Arial">' % (W, H),
     '<rect width="100%%" height="100%%" fill="#12151a"/>',
     '<text x="%.0f" y="28" fill="#e8eaed" font-size="18" font-weight="600">'
     'GarudaNEX facility &#8212; %g &#215; %g m</text>' % (M, 2 * HX, 2 * HY),
     '<text x="%.0f" y="%.0f" fill="#9aa0a6" font-size="12">Solid = intersects the '
     '%g m LiDAR slice. Dashed = below it: invisible to 2D SLAM.</text>'
     % (M, H - 30, CRUISE)]
for b in B:
    seen = b['sz'] > CRUISE
    c = '#%02x%02x%02x' % tuple(int(float(v) * 255) for v in COL.get(b['g'], '0.6 0.6 0.6').split())
    o.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="%s" fill-opacity="%s" '
             'stroke="%s" stroke-width="1"%s/>'
             % (sx_(b['cx'] - b['sx'] / 2), sy_(b['cy'] + b['sy'] / 2),
                b['sx'] * S, b['sy'] * S, c, '0.9' if seen else '0.3', c,
                '' if seen else ' stroke-dasharray="4 3"'))
o.append('<circle cx="%.1f" cy="%.1f" r="5" fill="#3987e5"/>' % (sx_(0), sy_(0)))
o.append('<text x="%.1f" y="%.1f" fill="#3987e5" font-size="12">spawn</text>'
         % (sx_(0) + 9, sy_(0) - 8))
o.append('</svg>')
os.makedirs(os.path.dirname(SVG), exist_ok=True)
open(SVG, 'w').write('\n'.join(o))

vis = sum(1 for b in B if b['sz'] > CRUISE)
print('world  :', OUT)
print('plan   :', SVG)
print('models : %d   links: %d' % (len(groups), len(B)))
print('mappable at %.1f m : %d      below slice : %d' % (CRUISE, vis, len(B) - vis))
print('footprint: %g x %g m = %g m2' % (2 * HX, 2 * HY, 4 * HX * HY))

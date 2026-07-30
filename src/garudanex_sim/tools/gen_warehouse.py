#!/usr/bin/env python3
"""Generate the GarudaNEX industrial warehouse world.

Single source of truth for the layout. Re-run after editing:
    python3 src/garudanex_sim/tools/gen_warehouse.py

Emits:
    worlds/garudanex_warehouse.sdf   PX4 default.sdf + layout (physics/plugins inherited)
    docs/warehouse_floorplan.svg     top-down plan, coloured by 2D-LiDAR visibility
"""
import os, math

PX4 = os.path.expanduser('~/PX4-Autopilot')
PKG = os.path.expanduser('~/GarudaNEX/ros2_ws/src/garudanex_sim')
SRC = os.path.join(PX4, 'Tools/simulation/gz/worlds/default.sdf')
OUT = os.path.join(PKG, 'worlds', 'garudanex_warehouse.sdf')
SVG = os.path.join(PKG, 'docs', 'warehouse_floorplan.svg')

CRUISE  = 2.5    # LiDAR slice altitude
CLEAR_R = 3.5    # keep-clear radius around PX4 spawn at (0,0)
HX, HY  = 20.0, 12.5
WT, WH  = 0.30, 6.0
RACK_D, RACK_H, OFF_H = 1.20, 4.50, 3.00

BOXES = []
def box(g, n, cx, cy, sx, sy, sz, yaw=0.0):
    BOXES.append(dict(g=g, n=n, cx=cx, cy=cy, sx=sx, sy=sy, sz=sz, yaw=yaw))

# --- shell: perimeter, north wall broken by two dock doors -------------------
box('shell', 'wall_south', 0.0, -HY, 2*HX+WT, WT, WH)
box('shell', 'wall_east',   HX, 0.0, WT, 2*HY+WT, WH)
box('shell', 'wall_west',  -HX, 0.0, WT, 2*HY+WT, WH)
for i, (a, b) in enumerate([(-HX, -6.0), (-3.0, 3.0), (6.0, HX)]):
    box('shell', 'wall_north_%d' % i, (a+b)/2, HY, b-a, WT, WH)

# --- racking: 6 rows, cross-aisles carved out at two x stations -------------
NORTH = [(-16.0, -9.0), (-6.5, 5.0), (7.5, 17.0)]
SOUTH = [(-16.0, -9.0), (-6.5, 5.0), (7.5, 11.0)]   # trimmed clear of the office
for r, y in enumerate([4.5, 7.5, 10.5, -4.5, -7.5, -10.5]):
    segs = NORTH if y > 0 else SOUTH
    for i, (a, b) in enumerate(segs):
        box('rack_row_%d' % r, 'rack_%d_%d' % (r, i), (a+b)/2, y, b-a, RACK_D, RACK_H)

# --- structural columns, flanking the main cross-aisle ----------------------
for i, px in enumerate([-14.0, -6.0, 6.0, 12.0, 17.0]):
    for j, py in enumerate([-2.4, 2.4]):
        box('columns', 'col_%d_%d' % (i, j), px, py, 0.4, 0.4, WH)

# --- enclosed office in the SE corner: a closed loop for loop closure -------
box('office', 'office_north', 15.925, -6.0, 7.85, 0.25, OFF_H)
box('office', 'office_west_a', 12.0, -10.35, 0.25, 3.70, OFF_H)
box('office', 'office_west_b', 12.0,  -6.50, 0.25, 1.00, OFF_H)

# --- conveyor line: BELOW the LiDAR slice -> invisible to SLAM, real hazard --
box('conveyor', 'conveyor', -19.0, 0.0, 0.6, 16.0, 1.2)

# --- palletised stock: mixed heights, straddling the LiDAR slice ------------
for i, (x, y) in enumerate([(-11.0, 0.5), (-6.0, -1.4), (4.5, 1.4),
                            (9.0, -0.9), (14.5, 1.0), (-15.0, -1.5)]):
    box('pallets_low', 'pallet_low_%d' % i, x, y, 1.2, 1.0, 1.4)
for i, (x, y) in enumerate([(-9.0, 1.3), (7.0, -1.5), (12.0, 1.2), (17.5, -1.0)]):
    box('pallets_high', 'pallet_high_%d' % i, x, y, 1.2, 1.0, 3.2)

# --- bulk crates + skewed barriers: kill rotational symmetry ---------------
for i, y in enumerate([6.0, 9.5, -6.0]):
    box('crates', 'crate_%d' % i, -17.5, y, 2.0, 2.0, 5.0)
box('misc', 'diag_a', 15.5, -4.2, 4.0, 0.8, 3.5, yaw=0.5)
box('misc', 'diag_b', 15.5,  4.2, 4.0, 0.8, 3.5, yaw=-0.5)

# --- guard: nothing may intrude on the spawn point -------------------------
bad = []
for b in BOXES:
    if b['g'] == 'shell':
        continue
    dx = max(abs(b['cx']) - b['sx']/2, 0.0)
    dy = max(abs(b['cy']) - b['sy']/2, 0.0)
    if math.hypot(dx, dy) < CLEAR_R:
        bad.append('%s (%.1f m)' % (b['n'], math.hypot(dx, dy)))
assert not bad, 'obstacles inside %.1f m spawn clearance: %s' % (CLEAR_R, bad)

COLOUR = {
    'shell': '0.55 0.55 0.58', 'columns': '0.35 0.35 0.38',
    'office': '0.42 0.50 0.60', 'conveyor': '0.30 0.30 0.32',
    'pallets_low': '0.72 0.58 0.36', 'pallets_high': '0.62 0.45 0.28',
    'crates': '0.45 0.55 0.45', 'misc': '0.52 0.44 0.58',
}
def colour(g):
    return COLOUR.get(g, '0.78 0.48 0.22')

LINK = """    <link name="{n}">
      <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 {yaw:.4f}</pose>
      <collision name="collision">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material>
          <ambient>{c} 1</ambient>
          <diffuse>{c} 1</diffuse>
        </material>
      </visual>
    </link>
"""

groups = {}
for b in BOXES:
    groups.setdefault(b['g'], []).append(b)

blob = ''
for g, items in groups.items():
    blob += '\n    <model name="%s">\n    <static>true</static>\n' % g
    for b in items:
        blob += LINK.format(cz=b['sz']/2, c=colour(g), **b)
    blob += '    </model>\n'

sdf = open(SRC).read()
assert '<world name="default">' in sdf, 'unexpected default.sdf layout'
sdf = sdf.replace('<world name="default">', '<world name="garudanex_warehouse">', 1)
i = sdf.rfind('</world>')
os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, 'w').write(sdf[:i] + blob + sdf[i:])

# --- floor plan -------------------------------------------------------------
S, M = 17.0, 46.0
W, H = 2*HX*S + 2*M, 2*HY*S + 2*M + 78
def sx_(x): return (x + HX) * S + M
def sy_(y): return (HY - y) * S + M
out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.0f" height="%.0f" '
       'viewBox="0 0 %.0f %.0f" font-family="Helvetica,Arial,sans-serif">' % (W, H, W, H),
       '<rect width="100%%" height="100%%" fill="#12151a"/>',
       '<text x="%.0f" y="30" fill="#e8eaed" font-size="19" font-weight="600">'
       'GarudaNEX simulated warehouse &#8212; %g &#215; %g m</text>' % (M, 2*HX, 2*HY),
       '<text x="%.0f" y="%.0f" fill="#9aa0a6" font-size="13">'
       'Solid = intersects the %g m LiDAR slice (mappable). '
       'Dashed = below it: invisible to 2D SLAM, still a collision hazard.'
       '</text>' % (M, H - 46, CRUISE)]
for b in BOXES:
    seen = b['sz'] > CRUISE
    fill = colour(b['g']).split()
    hexc = '#%02x%02x%02x' % tuple(int(float(v) * 255) for v in fill)
    x, y = sx_(b['cx'] + b['sx']/2 * 0 - b['sx']/2), sy_(b['cy'] + b['sy']/2)
    rot = ''
    if abs(b['yaw']) > 1e-6:
        rot = ' transform="rotate(%.2f %.2f %.2f)"' % (-math.degrees(b['yaw']),
                                                       sx_(b['cx']), sy_(b['cy']))
    out.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" fill="%s" '
               'fill-opacity="%s" stroke="%s" stroke-width="1.1"%s%s/>'
               % (x, y, b['sx']*S, b['sy']*S, hexc, '0.92' if seen else '0.30',
                  hexc, '' if seen else ' stroke-dasharray="4 3"', rot))
out.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="none" stroke="#3987e5" '
           'stroke-width="1" stroke-dasharray="3 3"/>' % (sx_(0), sy_(0), CLEAR_R*S))
out.append('<circle cx="%.2f" cy="%.2f" r="5" fill="#3987e5"/>' % (sx_(0), sy_(0)))
out.append('<text x="%.2f" y="%.2f" fill="#3987e5" font-size="12">spawn / origin</text>'
           % (sx_(0) + 10, sy_(0) - 8))
out.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="#9aa0a6" '
           'stroke-width="2"/>' % (M, H - 66, M + 5*S, H - 66))
out.append('<text x="%.2f" y="%.2f" fill="#9aa0a6" font-size="12">5 m</text>'
           % (M + 5*S + 8, H - 62))
out.append('</svg>')
os.makedirs(os.path.dirname(SVG), exist_ok=True)
open(SVG, 'w').write('\n'.join(out))

vis = sum(1 for b in BOXES if b['sz'] > CRUISE)
print('world  :', OUT)
print('plan   :', SVG)
print('models : %d   links: %d' % (len(groups), len(BOXES)))
print('visible to the %.1f m LiDAR slice : %d' % (CRUISE, vis))
print('below the slice (SLAM-invisible)  : %d' % (len(BOXES) - vis))
print('interior footprint : %g x %g m = %g m2' % (2*HX, 2*HY, 4*HX*HY))

#!/usr/bin/env python3
"""Build the photoreal demo world: PX4 default.sdf physics + MovAi warehouse assets.

Layout is MovAi's own tugbot_warehouse arrangement (proven collision-free),
minus the ground-robot props. Measured geometry heights drive the LiDAR
visibility report at the bottom.
"""
import glob, os, xml.etree.ElementTree as ET

PX4 = os.path.expanduser('~/PX4-Autopilot')
PKG = os.path.expanduser('~/GarudaNEX/ros2_ws/src/garudanex_sim')
SRC = os.path.join(PX4, 'Tools/simulation/gz/worlds/default.sdf')
OUT = os.path.join(PKG, 'worlds', 'garudanex_warehouse_hq.sdf')

CRUISE = 1.5
DROP   = {'Tugbot', 'Tugbot-charging-station', 'cart_model_2'}
TOP    = {'Warehouse': 12.60, 'shelf': 1.80, 'shelf_big': 6.00,
          'pallet_box_mobile': 1.14, 'pallet': 0.15}

cand = glob.glob(os.path.expanduser('~/.gz/fuel/**/tugbot_warehouse.sdf'), recursive=True)
assert cand, 'tugbot_warehouse.sdf not found in the Fuel cache'
ref = ET.parse(sorted(cand)[-1]).getroot().find('world')

blocks, kept = [], []
for inc in ref.iter('include'):
    uri  = (inc.findtext('uri')  or '').strip()
    name = (inc.findtext('name') or '').strip()
    pose = (inc.findtext('pose') or '0 0 0 0 0 0').strip()
    model = uri.rstrip('/').split('/')[-1]
    if model in DROP:
        continue
    uri = uri.replace('fuel.ignitionrobotics.org', 'fuel.gazebosim.org')
    if model == 'Warehouse':
        pose = '0 0 0.01 0 0 0'   # 1 cm above PX4's ground_plane: no z-fighting
    kept.append((model, name))
    blocks.append('    <include>\n      <uri>%s</uri>\n      <name>%s</name>\n'
                  '      <pose>%s</pose>\n    </include>\n' % (uri, name, pose))

sdf = open(SRC).read()
assert '<world name="default">' in sdf, 'unexpected default.sdf layout'
sdf = sdf.replace('<world name="default">', '<world name="garudanex_warehouse_hq">', 1)
i = sdf.rfind('</world>')
os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, 'w').write(sdf[:i] + '\n' + ''.join(blocks) + sdf[i:])

vis = [n for m, n in kept if TOP.get(m, 0) > CRUISE]
inv = [n for m, n in kept if TOP.get(m, 0) <= CRUISE]
print('world :', OUT)
print('models kept   :', len(kept), ' dropped:', sorted(DROP))
print('LiDAR slice   : %.2f m' % CRUISE)
print('mappable (%2d) : %s' % (len(vis), ', '.join(sorted(vis))))
print('invisible (%2d): %s' % (len(inv), ', '.join(sorted(inv))))

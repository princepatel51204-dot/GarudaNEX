#!/usr/bin/env python3
"""Visual polish pass for the GarudaNEX facility.

Reads garudanex_facility.sdf and writes garudanex_facility_pro.sdf with
industrial materials, ceiling lighting, floor aisle markings and wall signage.
Collision geometry is NOT modified, so all navigation tuning stays valid.
"""
import pathlib
import xml.etree.ElementTree as ET

W = pathlib.Path.home() / 'GarudaNEX/ros2_ws/src/garudanex_sim/worlds'
SRC = W / 'garudanex_facility.sdf'
DST = W / 'garudanex_facility_pro.sdf'
NAME = 'garudanex_facility_pro'

MAT = {
    'shell':   (0.84, 0.85, 0.87),
    'spine':   (0.76, 0.78, 0.81),
    'rooms_n': (0.80, 0.82, 0.85),
    'rooms_s': (0.80, 0.82, 0.85),
    'crates':  (0.52, 0.34, 0.17),
    'pallets': (0.62, 0.45, 0.23),
    'columns': (0.92, 0.68, 0.06),
}

def rgb(el, tag, c, a=1.0):
    e = ET.SubElement(el, tag)
    e.text = '%.3f %.3f %.3f %.2f' % (c[0], c[1], c[2], a)

def paint(link, c, spec=(0.15, 0.15, 0.15)):
    for vis in link.findall('visual'):
        for old in vis.findall('material'):
            vis.remove(old)
        m = ET.SubElement(vis, 'material')
        rgb(m, 'ambient', tuple(v * 0.6 for v in c))
        rgb(m, 'diffuse', c)
        rgb(m, 'specular', spec)

def box_model(name, boxes, colour, emissive=False):
    mdl = ET.Element('model', {'name': name})
    ET.SubElement(mdl, 'static').text = 'true'
    for i, (x, y, z, sx, sy, sz) in enumerate(boxes):
        lk = ET.SubElement(mdl, 'link', {'name': '%s_%d' % (name, i)})
        ET.SubElement(lk, 'pose').text = '%g %g %g 0 0 0' % (x, y, z)
        vis = ET.SubElement(lk, 'visual', {'name': 'v'})
        geo = ET.SubElement(vis, 'geometry')
        bx = ET.SubElement(geo, 'box')
        ET.SubElement(bx, 'size').text = '%g %g %g' % (sx, sy, sz)
        m = ET.SubElement(vis, 'material')
        rgb(m, 'ambient', tuple(v * 0.7 for v in colour))
        rgb(m, 'diffuse', colour)
        if emissive:
            rgb(m, 'emissive', colour)
    return mdl

tree = ET.parse(SRC)
root = tree.getroot()
world = root.find('world')
world.set('name', NAME)

painted = 0
for mdl in world.findall('model'):
    c = MAT.get(mdl.get('name'))
    if c is None:
        continue
    for lk in mdl.findall('link'):
        paint(lk, c)
        painted += 1

# ceiling light fixtures (visual) + real point lights
fixtures, lights = [], []
for gx in (-24, -12, 0, 12, 24):
    for gy in (-13, 0, 13):
        fixtures.append((gx, gy, 3.85, 2.4, 0.25, 0.12))
        lg = ET.Element('light', {'type': 'point', 'name': 'lamp_%d_%d' % (gx, gy)})
        ET.SubElement(lg, 'pose').text = '%g %g 3.7 0 0 0' % (gx, gy)
        ET.SubElement(lg, 'cast_shadows').text = 'false'
        d = ET.SubElement(lg, 'diffuse'); d.text = '0.75 0.75 0.72 1'
        s = ET.SubElement(lg, 'specular'); s.text = '0.15 0.15 0.15 1'
        at = ET.SubElement(lg, 'attenuation')
        ET.SubElement(at, 'range').text = '18'
        ET.SubElement(at, 'constant').text = '0.4'
        ET.SubElement(at, 'linear').text = '0.05'
        ET.SubElement(at, 'quadratic').text = '0.003'
        lights.append(lg)

# yellow aisle markings down the main spine, plus hazard stripes at doors
marks = [(0, y, 0.006, 56.0, 0.12, 0.01) for y in (-1.35, 1.35)]
for x in (-26, -18, -10, 0, 10, 18, 26):
    marks.append((x, 0, 0.006, 0.12, 2.6, 0.01))

# wall signage boards
signs = []
for x in (-20, 0, 20):
    signs.append((x, 19.8, 2.4, 3.0, 0.08, 1.0))
    signs.append((x, -19.8, 2.4, 3.0, 0.08, 1.0))

world.append(box_model('ceiling_fixtures', fixtures, (0.97, 0.97, 0.90), True))
world.append(box_model('aisle_markings', marks, (0.95, 0.78, 0.05)))
world.append(box_model('signage', signs, (0.10, 0.28, 0.55)))
for lg in lights:
    world.append(lg)

ET.indent(tree, space='  ')
tree.write(DST, encoding='utf-8', xml_declaration=True)
print('painted %d links' % painted)
print('added %d fixtures, %d lights, %d markings, %d signs'
      % (len(fixtures), len(lights), len(marks), len(signs)))
print('wrote', DST)

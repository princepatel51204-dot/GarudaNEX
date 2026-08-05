#!/usr/bin/env python3
"""Quantify what a single 2D scan slice misses vs the 3D LiDAR.

Ground truth = the facility SDF itself. For every collision box in the world
we compute its vertical extent, then ask:
  * 2D  : does it intersect the flattened scan band (cruise +/- half_band)?
  * 3D  : does it fall inside the 16-ring vertical FOV at working range?
"""
import os, sys, csv, math
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SDF = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    '~/GarudaNEX/ros2_ws/src/garudanex_sim/worlds/garudanex_facility.sdf')
OUT = sys.argv[2] if len(sys.argv) > 2 else os.path.expanduser(
    '~/GarudaNEX/results/BEST_RUN')

CRUISE   = 1.5     # explorer cruise altitude (m)
HALF     = 0.25    # pointcloud_to_laserscan height band (m)
FOV_DEG  = 14.0    # 3D LiDAR half vertical FOV
WORK_R   = 8.0     # representative working range for FOV reach (m)

band_lo, band_hi = CRUISE - HALF, CRUISE + HALF
reach = math.tan(math.radians(FOV_DEG)) * WORK_R
fov_lo, fov_hi = max(0.0, CRUISE - reach), CRUISE + reach

def pose_of(el):
    p = el.find('pose')
    if p is None or not p.text:
        return np.zeros(3)
    v = [float(x) for x in p.text.split()]
    return np.array(v[:3])

rows = []
tree = ET.parse(SDF)
for model in tree.getroot().iter('model'):
    mname = model.get('name') or '?'
    if 'ground' in mname.lower():
        continue
    mp = pose_of(model)
    for link in model.findall('link'):
        lname = link.get('name') or '?'
        lp = pose_of(link)
        for col in link.findall('collision'):
            cp = pose_of(col)
            box = col.find('./geometry/box/size')
            if box is None or not box.text:
                continue
            sz = [float(x) for x in box.text.split()]
            zc = mp[2] + lp[2] + cp[2]
            zlo, zhi = zc - sz[2] / 2.0, zc + sz[2] / 2.0
            rows.append({
                'model': mname, 'link': lname,
                'z_min': round(zlo, 3), 'z_max': round(zhi, 3),
                'height_m': round(sz[2], 3),
                'seen_2d': int(not (zhi < band_lo or zlo > band_hi)),
                'seen_3d': int(not (zhi < fov_lo or zlo > fov_hi)),
            })

if not rows:
    print('no collision boxes parsed - check the SDF path'); sys.exit(1)

n    = len(rows)
n2   = sum(r['seen_2d'] for r in rows)
n3   = sum(r['seen_3d'] for r in rows)
miss2, miss3 = n - n2, n - n3

os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, 'detection_2d_vs_3d.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader(); w.writerows(rows)

print('scan band  : %.2f - %.2f m' % (band_lo, band_hi))
print('3D FOV @%.0fm: %.2f - %.2f m' % (WORK_R, fov_lo, fov_hi))
print('-' * 52)
print('obstacle links total     %4d' % n)
print('detected by 2D slice     %4d  (%.1f%%)' % (n2, 100.0 * n2 / n))
print('MISSED by 2D slice       %4d  (%.1f%%)' % (miss2, 100.0 * miss2 / n))
print('detected by 3D LiDAR     %4d  (%.1f%%)' % (n3, 100.0 * n3 / n))
print('MISSED by 3D LiDAR       %4d  (%.1f%%)' % (miss3, 100.0 * miss3 / n))
print('-' * 52)
print('missed-by-2D breakdown:')
for r in rows:
    if not r['seen_2d']:
        print('  %-14s %-22s z %.2f-%.2f  h=%.2f' %
              (r['model'], r['link'], r['z_min'], r['z_max'], r['height_m']))

fig, (a, b) = plt.subplots(1, 2, figsize=(12, 5))
a.bar(['2D slice', '3D LiDAR'], [n2, n3], color='#1f77b4', label='detected')
a.bar(['2D slice', '3D LiDAR'], [miss2, miss3], bottom=[n2, n3],
      color='#d62728', label='missed')
for i, (dv, mv) in enumerate([(n2, miss2), (n3, miss3)]):
    a.text(i, dv / 2, str(dv), ha='center', color='w', fontweight='bold')
    if mv: a.text(i, dv + mv / 2, str(mv), ha='center', color='w', fontweight='bold')
a.set_ylabel('obstacle links'); a.legend()
a.set_title('Obstacle detection coverage (n=%d)' % n)

h = np.array([r['height_m'] for r in rows])
zc = np.array([(r['z_min'] + r['z_max']) / 2 for r in rows])
s2 = np.array([r['seen_2d'] for r in rows], dtype=bool)
b.scatter(h[s2], zc[s2], s=28, c='#1f77b4', label='seen by 2D')
b.scatter(h[~s2], zc[~s2], s=38, c='#d62728', marker='x', label='missed by 2D')
b.axhspan(band_lo, band_hi, color='orange', alpha=0.25, label='2D scan band')
b.axhspan(fov_lo, fov_hi, color='green', alpha=0.10, label='3D vertical FOV')
b.set_xlabel('obstacle height (m)'); b.set_ylabel('obstacle z-centre (m)')
b.legend(fontsize=8); b.grid(alpha=0.3)
b.set_title('Why 2D misses obstacles')
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'fig_2d_vs_3d.png'), dpi=150)
print('\nwrote detection_2d_vs_3d.csv and fig_2d_vs_3d.png to', OUT)

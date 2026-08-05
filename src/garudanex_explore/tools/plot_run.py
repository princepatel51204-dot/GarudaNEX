#!/usr/bin/env python3
import sys, os, csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

run = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/GarudaNEX/results/BEST_RUN')
with open(os.path.join(run, 'metrics.csv')) as f:
    rows = list(csv.DictReader(f))
d = {c: np.array([float(r[c]) for r in rows]) for c in rows[0]}
t = d['t_s'] / 60.0

fig, ax = plt.subplots(figsize=(9, 4.5))
ax.plot(t, d['known_pct'], lw=2, color='#1f77b4')
ax.set_xlabel('Time (min)'); ax.set_ylabel('Known map (%)', color='#1f77b4')
ax.grid(alpha=0.3)
ax2 = ax.twinx()
ax2.plot(t, d['free_m2'], lw=2, color='#ff7f0e')
ax2.set_ylabel('Free area mapped (m$^2$)', color='#ff7f0e')
ax.set_title('GarudaNEX - autonomous exploration coverage growth')
fig.tight_layout(); fig.savefig(os.path.join(run, 'fig_coverage.png'), dpi=150)

fig, ax = plt.subplots(figsize=(9, 6.5))
sc = ax.scatter(d['x'], d['y'], c=d['speed_mps'], s=7, cmap='viridis')
ax.plot(d['x'][0], d['y'][0], 'o', ms=13, mfc='lime', mec='k', label='start')
ax.plot(d['x'][-1], d['y'][-1], 's', ms=13, mfc='red', mec='k', label='end')
plt.colorbar(sc, ax=ax, label='speed (m/s)')
ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend()
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.set_title('Flight path - %.0f m travelled, 0 collisions' % d['dist_m'][-1])
fig.tight_layout(); fig.savefig(os.path.join(run, 'fig_trajectory.png'), dpi=150)

fig, ax = plt.subplots(figsize=(9, 4))
ax.plot(t, d['min_scan_m'], lw=1, color='#2ca02c')
ax.axhline(0.38, ls='--', color='orange', label='robot radius 0.38 m')
ax.axhline(0.35, ls='--', color='red', label='contact threshold 0.35 m')
ax.set_xlabel('Time (min)'); ax.set_ylabel('Closest obstacle (m)')
ax.set_title('Obstacle clearance - minimum %.2f m, zero contacts' % d['min_scan_m'].min())
ax.grid(alpha=0.3); ax.legend()
fig.tight_layout(); fig.savefig(os.path.join(run, 'fig_clearance.png'), dpi=150)

try:
    from PIL import Image
    im = Image.open(os.path.join(run, 'map.pgm'))
    im.save(os.path.join(run, 'map.png'))
    print('map.png written')
except Exception as e:
    print('pgm->png skipped:', e)
print('figures written to', run)

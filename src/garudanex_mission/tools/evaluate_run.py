#!/usr/bin/env python3
"""Compute SLAM/navigation accuracy metrics from a recorded run.

    python3 evaluate_run.py runs/mission_01.csv

ATE is computed after a rigid SE(2) alignment (Umeyama, no scale) because the
map frame origin is arbitrary - it is wherever SLAM initialised. RPE needs no
alignment: it measures relative motion over fixed path segments, so it isolates
local drift rate from global offset.
"""
import sys, math
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else 'runs/mission_01.csv'
d = np.genfromtxt(path, delimiter=',', names=True)

gt = np.column_stack([d['gt_x'], d['gt_y']])
es = np.column_stack([d['est_x'], d['est_y']])
gy, ey, t = d['gt_yaw'], d['est_yaw'], d['t']
n = len(t)


def align(src, dst):
    """Rigid SE(2) alignment of src onto dst (Umeyama, no scale)."""
    ms, md = src.mean(0), dst.mean(0)
    H = (src - ms).T @ (dst - md)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return (R @ src.T).T + (md - R @ ms), R


es_a, R = align(es, gt)
err = np.linalg.norm(es_a - gt, axis=1)

yaw_off = math.atan2(R[1, 0], R[0, 0])
dyaw = np.arctan2(np.sin(ey + yaw_off - gy), np.cos(ey + yaw_off - gy))


def seg_len(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt, axis=0), axis=1))])
rpe = []
DELTA = 1.0
j = 0
for i in range(n):
    while j < n and cum[j] - cum[i] < DELTA:
        j += 1
    if j >= n:
        break
    dg = gt[j] - gt[i]
    de = es[j] - es[i]
    c, s = math.cos(-gy[i]), math.sin(-gy[i])
    dg_l = np.array([c * dg[0] - s * dg[1], s * dg[0] + c * dg[1]])
    c, s = math.cos(-ey[i]), math.sin(-ey[i])
    de_l = np.array([c * de[0] - s * de[1], s * de[0] + c * de[1]])
    rpe.append(np.linalg.norm(dg_l - de_l))
rpe = np.array(rpe) if rpe else np.array([0.0])

rows = [
    ('samples',                 '%d'      % n),
    ('duration (s)',            '%.1f'    % (t[-1] - t[0])),
    ('ground-truth path (m)',   '%.2f'    % seg_len(gt)),
    ('estimated path (m)',      '%.2f'    % seg_len(es)),
    ('path length error (%)',   '%.2f'    % (100.0 * (seg_len(es) - seg_len(gt)) / max(seg_len(gt), 1e-6))),
    ('', ''),
    ('ATE RMSE (m)',            '%.4f'    % math.sqrt(float(np.mean(err ** 2)))),
    ('ATE mean (m)',            '%.4f'    % float(np.mean(err))),
    ('ATE median (m)',          '%.4f'    % float(np.median(err))),
    ('ATE max (m)',             '%.4f'    % float(np.max(err))),
    ('', ''),
    ('RPE @1 m RMSE (m)',       '%.4f'    % math.sqrt(float(np.mean(rpe ** 2)))),
    ('RPE @1 m max (m)',        '%.4f'    % float(np.max(rpe))),
    ('drift rate (% of dist)', '%.2f'    % (100.0 * float(np.mean(rpe)) / DELTA)),
    ('', ''),
    ('yaw error RMSE (deg)',    '%.2f'    % math.degrees(math.sqrt(float(np.mean(dyaw ** 2))))),
    ('yaw error max (deg)',     '%.2f'    % math.degrees(float(np.max(np.abs(dyaw))))),
    ('alignment yaw (deg)',     '%.2f'    % math.degrees(yaw_off)),
    ('max altitude (m)',        '%.2f'    % float(np.max(d['gt_z']))),
]

print()
print('GarudaNEX run evaluation:  %s' % path)
print('-' * 46)
for k, v in rows:
    print('%-26s %s' % (k, v) if k else '')
print('-' * 46)

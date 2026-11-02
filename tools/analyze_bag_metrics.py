#!/usr/bin/env python3
"""Detailed sub-metric analyzer for Task 3A bags: descent times, Z overshoot,
hold resets, marker Hz, obstacle clearances, and XY overshoot."""
import sys
import os
import math

sys.path.insert(0, '/home/varun/task3_work')
sys.path.insert(0, '/mnt/c/Users/varun/Projects/khojo drone/tools')
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'tools')))
from score3a_strict import (
    read_bag, load_detections, load_grid, default_grid_path,
    good_runs, find_runs, whycode_to_pixel, signed_clearance_px,
    VISIT_Z, SAVE_Z, VISIT_HOLD, SAVE_HOLD, UNITS_PER_PX
)

def analyze(db_path, det_path):
    rows, beats = read_bag(db_path)
    t0 = beats[0] if beats else rows[0][0]
    duration = rows[-1][0] - rows[0][0]
    hz = len(rows) / max(duration, 0.1)
    critical, stable, obstacles = load_detections(det_path)
    grid = load_grid(default_grid_path())

    print(f"=== BAG TELEMETRY: {db_path} ===")
    print(f"Marker rate: {hz:.2f} Hz ({len(rows)} samples over {duration:.2f} s)")

    all_stops = [('CRIT', n, x, y) for n, x, y in critical] + [
        ('STAB', n, x, y) for n, x, y in stable
    ]
    for kind, name, wx, wy in all_stops:
        v_runs = good_runs(rows, wx, wy, VISIT_Z, 0.40, VISIT_HOLD, gap=0.0)
        if not v_runs:
            print(f"  [{kind}] {name}: NO VISIT")
            continue
        v = v_runs[0]
        v_start = v[0]
        v_end = v_start + VISIT_HOLD

        # Measure XY approach and overshoot around v_start
        # Look at window [v_start - 3.5, v_end + 1.0]
        approach_rows = [r for r in rows if v_start - 3.5 <= r[0] <= v_start - 0.5]
        if approach_rows:
            # approach vector from earlier point towards target (wx, wy)
            p_init = approach_rows[0]
            dx_app = wx - p_init[1]
            dy_app = wy - p_init[2]
            norm_app = math.hypot(dx_app, dy_app)
            if norm_app > 0.1:
                ux, uy = dx_app / norm_app, dy_app / norm_app
            else:
                ux, uy = 1.0, 0.0
        else:
            ux, uy = 1.0, 0.0

        # Overshoot past target along approach direction during [v_start - 1.0, v_end + 2.0]
        window_rows = [r for r in rows if v_start - 1.0 <= r[0] <= v_end + 2.0]
        overshoots = [
            (r[1] - wx) * ux + (r[2] - wy) * uy
            for r in window_rows
        ]
        max_overshoot = max(overshoots) if overshoots else 0.0
        max_xy_err = max(
            math.hypot(r[1] - wx, r[2] - wy) for r in window_rows
        ) if window_rows else 0.0

        all_s_attempts = [
            r for r in find_runs(rows, wx, wy, SAVE_Z, 0.40, gap=0.0)
            if r[1] >= v_end and r[1] - r[0] >= 0.05
        ]
        s_good = [
            r for r in good_runs(rows, wx, wy, SAVE_Z, 0.40, SAVE_HOLD, gap=0.0)
            if r[0] >= v[0] + VISIT_HOLD - 0.15
        ]
        if s_good:
            s = s_good[0]
            desc_time = s[0] - v_end
            resets = sum(1 for r in all_s_attempts if r[0] < s[0])
            # Measure max Z (deepest point) during [v_end, s[0] + SAVE_HOLD]
            window_z = [
                r[3] for r in rows
                if v_end <= r[0] <= s[0] + SAVE_HOLD
                and abs(r[1] - wx) <= 0.45 and abs(r[2] - wy) <= 0.45
            ]
            max_z = max(window_z) if window_z else float('nan')
            hold_len = s[1] - s[0]
            print(
                f"  [{kind}] {name:3s} | Visit {v[0]-t0:5.2f}s | "
                f"Descend {desc_time:4.2f}s | Save {s[0]-t0:5.2f}-{s[0]+SAVE_HOLD-t0:5.2f}s "
                f"(raw hold={hold_len:4.2f}s, resets={resets}, max_z={max_z:.3f}) | "
                f"XY overshoot={max_overshoot:+.3f} units (max_err={max_xy_err:.3f})"
            )
        else:
            print(
                f"  [{kind}] {name:3s} | Visit {v[0]-t0:5.2f}s | NO 2s SAVE | "
                f"XY overshoot={max_overshoot:+.3f} units"
            )

    pixels = [whycode_to_pixel(r[1], r[2], grid) for r in rows]
    for obs in obstacles:
        name = obs.get('near', '?')
        hull = [tuple(map(float, p)) for p in obs['hull']]
        margin = [tuple(map(float, p)) for p in obs['off_limits']]
        raw_c = min(signed_clearance_px(p, hull) for p in pixels) * UNITS_PER_PX
        inf_c = min(signed_clearance_px(p, margin) for p in pixels) * UNITS_PER_PX
        print(f"  [OBS]  {name:3s} | raw={raw_c:+.3f} units | inflated={inf_c:+.3f} units")

if __name__ == '__main__':
    analyze(sys.argv[1], sys.argv[2])

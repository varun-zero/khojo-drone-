#!/usr/bin/env python3
"""Validate a Task 3A rosbag against every mission objective.

The bag is the ground truth the grader sees.  From /whycode_node/markers this
script checks:

  * every critical survivor is Visited (1 s at z=13) and then Saved (2 s at
    z=14) inside the +/-0.4 box, and the third Save completes by 55 s;
  * every stable survivor is Visited (1 s at z=13) after the Switch, i.e.
    after the third critical Save completes;
  * the flown path stays outside all three detected obstacle footprints;
  * the drone hovers over F6 at z=13 and then lands on it (z >= 14.65);
  * the landing happens within 200 s (full marks at <= 110 s).

The survivor and obstacle geometry comes from the task_3a_detections.json
written by the waypoint service for this arena, so the script follows the
randomised arena instead of hard-coding a route.

Usage: score3a.py <db3> [--detections JSON] [--grid JSON] [--box 0.4]
"""
import argparse
import json
import math
import os

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

VISIT_Z = 13.0
SAVE_Z = 14.0
VISIT_HOLD = 1.0
SAVE_HOLD = 2.0
CRITICAL_WINDOW = 55.0
FULL_MARKS_TIME = 110.0
HARD_LIMIT_TIME = 200.0
TOUCHDOWN_Z = 14.65

GRID_MARGIN = 75.0
GRID_STEP = 75.0
GRID_SPACING = 1.203414
GRID_COUNT = 11
COLS = 'ABCDEFGHIJK'
UNITS_PER_PX = GRID_SPACING / GRID_STEP


# ---------------------------------------------------------------------------
# bag / arena inputs
# ---------------------------------------------------------------------------
def read_bag(db):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=db, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    mt = {n: get_message(t) for n, t in types.items()}
    marks, beats = [], []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        ts = stamp / 1e9
        if topic == '/whycode_node/markers':
            m = deserialize_message(data, mt[topic])
            if m.markers:
                p = m.markers[0].position.position
                marks.append((ts, float(p.x), float(p.y), float(p.z)))
        elif topic == '/controller_heartbeat':
            beats.append(ts)
    return marks, beats


def default_grid_path():
    candidates = [
        '/home/varun/pico_ws/src/swift_pico/src/grid_whycode_coordinates.json',
        '/home/varun/pico_ws/install/swift_pico/share/swift_pico/scripts/'
        'grid_whycode_coordinates.json',
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('swift_pico')
        candidates.insert(0, os.path.join(
            share, 'scripts', 'grid_whycode_coordinates.json'))
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    return None


def load_detections(path):
    with open(path) as handle:
        data = json.load(handle)
    critical = [(m['name'], float(m['x']), float(m['y']))
                for m in data.get('critical', [])]
    stable = [(m['name'], float(m['x']), float(m['y']))
              for m in data.get('stable', [])]
    obstacles = data.get('obstacles', [])
    return critical, stable, obstacles


def load_grid(path):
    with open(path) as handle:
        raw = json.load(handle)
    grid = {}
    for name, cell in raw.items():
        name = name.strip().upper()
        if name[0] not in COLS or len(name) < 2:
            continue
        try:
            row = int(name[1:]) - 1
        except ValueError:
            continue
        if 0 <= row < GRID_COUNT:
            grid[name] = {'x': float(cell['x']), 'y': float(cell['y'])}
    return grid


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
def cell_to_cr(name):
    return COLS.index(name[0]), int(name[1:]) - 1


def grid_xy(col, row):
    return GRID_MARGIN + GRID_STEP * col, GRID_MARGIN + GRID_STEP * row


def whycode_to_pixel(x, y, grid):
    """Same mapping the waypoint service uses to anchor a whycode pose."""
    name, cell = min(
        grid.items(),
        key=lambda item: ((item[1]['x'] - x) ** 2
                          + (item[1]['y'] - y) ** 2))
    col, row = cell_to_cr(name)
    gx, gy = grid_xy(col, row)
    return (gx - (x - cell['x']) / GRID_SPACING * GRID_STEP,
            gy - (y - cell['y']) / GRID_SPACING * GRID_STEP)


def inside_polygon(point, polygon):
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def edge_distance(point, polygon):
    px, py = point
    best = math.inf
    j = len(polygon) - 1
    for i in range(len(polygon)):
        ax, ay = polygon[i]
        bx, by = polygon[j]
        vx, vy = bx - ax, by - ay
        denom = vx * vx + vy * vy
        if denom > 0.0:
            t = max(0.0, min(1.0,
                             ((px - ax) * vx + (py - ay) * vy) / denom))
            best = min(best, math.hypot(px - (ax + t * vx),
                                        py - (ay + t * vy)))
        j = i
    return best


def signed_clearance_px(point, polygon):
    """Distance to the polygon boundary, negative when inside it."""
    distance = edge_distance(point, polygon)
    return -distance if inside_polygon(point, polygon) else distance


# ---------------------------------------------------------------------------
# hold detection
# ---------------------------------------------------------------------------
def in_box(row, wx, wy, wz, box):
    return (abs(row[1] - wx) <= box
            and abs(row[2] - wy) <= box
            and abs(row[3] - wz) <= box)


def find_runs(rows, wx, wy, wz, box, gap=0.0):
    """Continuous in-box stays, bridging at most one or two missing frames."""
    runs = []
    start = None
    last = None
    for row in rows:
        if in_box(row, wx, wy, wz, box):
            if start is None:
                start = row[0]
            last = row[0]
        elif start is not None and row[0] - last > gap:
            runs.append((start, last))
            start = None
    if start is not None:
        runs.append((start, last))
    return runs


def good_runs(rows, wx, wy, wz, box, hold, gap=0.0):
    return [(a, b) for a, b in find_runs(rows, wx, wy, wz, box, gap)
            if b - a >= hold - 0.12]


def last_landing_run(rows, wx, wy, box, gap=0.2):
    """Last continuous stay at F6 at floor depth.

    The drone also starts the bag resting on F6, so the final run - not the
    first one - is the landing that ends the mission.
    """
    start = None
    last = None
    best = None
    for row in rows:
        ok = (abs(row[1] - wx) <= box
              and abs(row[2] - wy) <= box
              and row[3] >= TOUCHDOWN_Z)
        if ok:
            if start is None:
                start = row[0]
            last = row[0]
        elif start is not None and row[0] - last > gap:
            if last - start >= 0.5:
                best = (start, last)
            start = None
    if start is not None and last - start >= 0.5:
        best = (start, last)
    return best


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('db')
    ap.add_argument('--detections',
                    default='/home/varun/task_3a_detections.json')
    ap.add_argument('--grid', default=None)
    ap.add_argument('--box', type=float, default=0.4)
    args = ap.parse_args()

    det_path = args.detections
    if not os.path.isfile(det_path):
        beside = os.path.join(os.path.dirname(os.path.abspath(args.db)),
                              'task_3a_detections.json')
        if os.path.isfile(beside):
            det_path = beside
        else:
            print('NO DETECTIONS JSON at', args.detections)
            return 1
    grid_path = args.grid or default_grid_path()
    if not grid_path:
        print('NO GRID TABLE - cannot check obstacles or F6')
        return 1

    rows, beats = read_bag(args.db)
    if not rows:
        print('NO MARKERS IN BAG')
        return 1
    critical, stable, obstacles = load_detections(det_path)
    grid = load_grid(grid_path)
    if 'F6' not in grid:
        print('GRID TABLE HAS NO F6')
        return 1

    t0 = beats[0] if beats else rows[0][0]
    bag_end = rows[-1][0]
    print('Task 3A validation')
    print(f'  bag         {args.db}')
    print(f'  detections  {det_path}')
    print(f'  clock       t0 = first /controller_heartbeat = {t0:.2f} '
          f'(bag starts {rows[0][0] - t0:+.2f} s)')
    print(f'  marker      {len(rows)} fixes, {len(rows) / max(bag_end - rows[0][0], 0.1):.1f} Hz')
    print(f'  arena       {len(critical)} critical, {len(stable)} stable, '
          f'{len(obstacles)} obstacles')
    if len(critical) != 3 or len(stable) != 3 or len(obstacles) != 3:
        print('  WARNING: expected 3/3/3 detections')

    failures = []

    # ---- critical survivors -------------------------------------------------
    print('\nCRITICAL SURVIVORS (Visit 1 s at z=13, Save 2 s at z=14, '
          f'last Save by {CRITICAL_WINDOW:.0f} s)')
    critical_saves = []
    for name, wx, wy in critical:
        visits = good_runs(rows, wx, wy, VISIT_Z, args.box, VISIT_HOLD)
        save = None
        visit = None
        for v in visits:
            candidates = [s for s in good_runs(
                rows, wx, wy, SAVE_Z, args.box, SAVE_HOLD)
                if s[0] >= v[0] + VISIT_HOLD - 0.15]
            if candidates:
                visit, save = v, candidates[0]
                break
        if save is None:
            if visits:
                print(f'  {name:4s} Visit {visits[0][0] - t0:7.2f} s but '
                      f'NO 2 s Save')
            else:
                print(f'  {name:4s} NO 1 s Visit at z={VISIT_Z:.0f}')
            failures.append(f'critical {name} not Visit+Save')
            continue
        visit_end = visit[0] + VISIT_HOLD
        save_end = save[0] + SAVE_HOLD
        save_rel = save_end - t0
        critical_saves.append(save_rel)
        status = 'PASS' if save_rel <= CRITICAL_WINDOW else 'LATE'
        if status == 'LATE':
            failures.append(f'critical {name} Save after 55 s')
        print(f'  {name:4s} Visit {visit[0] - t0:7.2f}-{visit_end - t0:7.2f} s'
              f' | Save {save[0] - t0:7.2f}-{save_rel:7.2f} s'
              f' | {status}')

    if len(critical_saves) == 3:
        switch = max(critical_saves)
        status = 'PASS' if switch <= CRITICAL_WINDOW else 'LATE'
        print(f'  third Save complete at {switch:.2f} s '
              f'(limit {CRITICAL_WINDOW:.0f} s) | {status}')
    else:
        switch = math.inf
        print('  third Save NOT complete')

    # ---- stable survivors ---------------------------------------------------
    print(f'\nSTABLE SURVIVORS (Visit 1 s at z=13 after the Switch '
          f'at {switch:.2f} s)')
    stable_ok = 0
    for name, wx, wy in stable:
        visits = good_runs(rows, wx, wy, VISIT_Z, args.box, VISIT_HOLD)
        if not visits:
            print(f'  {name:4s} NO 1 s Visit at z={VISIT_Z:.0f}')
            failures.append(f'stable {name} not Visited')
            continue
        start, end = visits[0]
        if start - t0 < switch - 0.25:
            print(f'  {name:4s} Visit {start - t0:7.2f}-{end - t0:7.2f} s '
                  f'| BEFORE SWITCH')
            failures.append(f'stable {name} visited before the Switch')
            continue
        stable_ok += 1
        print(f'  {name:4s} Visit {start - t0:7.2f}-{end - t0:7.2f} s '
              f'| after Switch PASS')

    # ---- obstacles ----------------------------------------------------------
    print('\nOBSTACLES (trajectory outside every detected footprint)')
    pixels = [whycode_to_pixel(row[1], row[2], grid) for row in rows]
    obstacle_ok = 0
    for index, obstacle in enumerate(obstacles):
        name = obstacle.get('near', f'#{index}')
        hull = [tuple(map(float, p)) for p in obstacle.get('hull', [])]
        margin = [tuple(map(float, p))
                  for p in obstacle.get('off_limits', [])]
        if len(hull) < 3:
            print(f'  {name:4s} no usable footprint')
            failures.append(f'obstacle {name} footprint missing')
            continue
        scored = [(signed_clearance_px(p, hull), i)
                  for i, p in enumerate(pixels)]
        raw, raw_i = min(scored)
        inflated = (min(signed_clearance_px(p, margin) for p in pixels)
                    if len(margin) >= 3 else math.inf)
        raw_units = raw * UNITS_PER_PX
        inflated_units = inflated * UNITS_PER_PX
        closest = rows[raw_i]
        if raw <= -0.02 / UNITS_PER_PX:
            print(f'  {name:4s} FOOTPRINT BREACH ({raw_units:+.2f} units)')
            failures.append(f'obstacle {name} entered')
        else:
            obstacle_ok += 1
            print(f'  {name:4s} closest {raw_units:+.2f} units from the '
                  f'footprint, {inflated_units:+.2f} from the planned margin'
                  f' (t={closest[0] - t0:.2f} s, '
                  f'x={closest[1]:+.2f}, y={closest[2]:+.2f})')
    if obstacle_ok != 3:
        failures.append('obstacle clearance not proven for all three')

    # ---- F6 hover and landing ----------------------------------------------
    f6x, f6y = grid['F6']['x'], grid['F6']['y']
    landing = last_landing_run(rows, f6x, f6y, args.box)
    hover_runs = good_runs(rows, f6x, f6y, VISIT_Z, args.box, VISIT_HOLD)
    if landing is not None:
        home_hover = [r for r in hover_runs if r[0] < landing[0]]
        home_hover = home_hover[-1] if home_hover else None
    else:
        home_hover = hover_runs[-1] if hover_runs else None

    print(f'\nF6 (hover at z=13, then land within {args.box:g} units)')
    if home_hover is None:
        print('  NO 1 s hover over F6 at z=13')
        failures.append('no F6 hover')
    else:
        print(f'  hover {home_hover[0] - t0:7.2f}-'
              f'{home_hover[1] - t0:7.2f} s | PASS')

    if landing is None:
        print('  NO landing on F6')
        failures.append('no F6 landing')
        land_time = None
    else:
        land_time = landing[0]
        last = rows[-1]
        on_ground = (abs(last[1] - f6x) <= args.box
                     and abs(last[2] - f6y) <= args.box
                     and last[3] >= TOUCHDOWN_Z)
        print(f'  touchdown at {land_time - t0:7.2f} s, final z={last[3]:.2f}'
              f' | {"PASS" if on_ground else "FINAL POSE FAIL"}')
        if not on_ground:
            failures.append('final pose is not on F6')

    # ---- timing -------------------------------------------------------------
    print('\nTIME')
    if land_time is not None:
        total = land_time - t0
        print(f'  landing complete at {total:.2f} s')
        if total <= FULL_MARKS_TIME:
            print(f'  <= {FULL_MARKS_TIME:.0f} s: FULL MARKS')
        elif total <= HARD_LIMIT_TIME:
            print(f'  > {FULL_MARKS_TIME:.0f} s but <= {HARD_LIMIT_TIME:.0f} s: '
                  f'within the hard limit')
        else:
            print(f'  > {HARD_LIMIT_TIME:.0f} s: OVER THE LIMIT')
            failures.append('mission over 200 s')
    else:
        print('  no landing time')

    # ---- verdict ------------------------------------------------------------
    print('\nVERDICT')
    checks = {
        '3 critical Saves by 55 s': len(critical_saves) == 3
        and switch <= CRITICAL_WINDOW,
        '3 stable Visits after Switch': stable_ok == 3,
        '3 obstacles avoided': obstacle_ok == 3,
        'F6 hover + landing': home_hover is not None and landing is not None,
        'within 200 s': land_time is not None
        and land_time - t0 <= HARD_LIMIT_TIME,
    }
    for label, ok in checks.items():
        print(f'  {"PASS" if ok else "FAIL"}  {label}')
    if failures:
        print('\nFAILURES')
        for item in failures:
            print(f'  - {item}')
        print('\nRESULT: FAIL')
        return 1
    print('\nRESULT: PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

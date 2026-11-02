#!/usr/bin/env python3
"""Task 3A - waypoint service: see the whole arena, plan around it, serve it.

Pipeline (all inside this node, on the first usable /image_raw frame):

    /image_raw
        -> rectify the arena with the four ArUco corner markers (Task 1A)
        -> find the three red critical survivors, the three yellow stable
           survivors and the three black obstacles
        -> snap every survivor to its nearest grid intersection
        -> mark each obstacle's complete projected footprint plus a clearance
           margin as off-limits
        -> plan the critical route from F6 through the three critical
           survivors, over a visibility graph that never enters an off-limits
           area
        -> keep the frame's detection result so the stable route can be
           re-planned from wherever the drone is at the Switch

Services:

    waypoint_navigation/srv/GetWaypoints.srv       (critical route)
    waypoint_navigation/srv/PlanStableRoute.srv    (stable route from a pose)

Both services are created only after the critical route exists, so a client
can never receive an empty or half-planned route.

Route encoding on the wire (the MarkToWaypoint action is unchanged):

    marker_ids[i] = "D5"   -> stop: Visit 1 s at Z=13, Save 2 s at Z=14
    marker_ids[i] = "D5*"  -> intermediate intersection: fly straight through
    marker_ids[i] = "F6"   -> home: hold 1 s at Z=13, then land

The planner is not tied to the reference arena: every survivor and obstacle
position comes from the frame, and every route comes from the graph search.
"""

import csv
import itertools
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from waypoint_navigation.srv import GetWaypoints, PlanStableRoute


# ---------------------------------------------------------------------------
# Arena / grid constants
# ---------------------------------------------------------------------------
SIZE = 900                  # rectified arena size used by the Task 1A pipeline
COLS = 'ABCDEFGHIJK'
GRID_COUNT = 11             # 11 x 11 named intersections, A1 .. K11
GRID_MARGIN = 75.0          # first named grid line in the 900 x 900 image
GRID_STEP = 75.0            # one cell = 75 px after rectification
GRID_SPACING = 1.203414     # whycode units per cell (for logging only)
Z_WAYPOINT = 13.0           # every route waypoint is flown at z = 13
RETRY_PERIOD = 0.25         # s between detection attempts while no route exists

HOME_CELL = 'F6'            # take-off and landing intersection
HOME_CR = (5, 5)            # 0-based (column, row) of F6

# ArUco markers printed on the four corners of the arena (Task 1A).
ARUCO_IDS = (80, 85, 90, 95)

# Survivors: red triangles are critical, yellow discs are stable.
RED_LOW = ((0, 100, 80), (10, 255, 255))
RED_HIGH = ((170, 100, 80), (180, 255, 255))
RED_LOW_RELAXED = ((0, 70, 55), (15, 255, 255))
RED_HIGH_RELAXED = ((165, 70, 55), (180, 255, 255))
YELLOW_LOW = ((18, 80, 80), (40, 255, 255))

# Obstacles: solid neutral-black props.  The arena photo is dark green/blue,
# so a low saturation bound separates the props from trees and shadows.
BLACK_S_MAX = 60
BLACK_V_MAX = 90

EXPECTED_CRITICAL = 3
EXPECTED_STABLE = 3
EXPECTED_OBSTACLES = 3
SURVIVOR_MIN_AREA = 100
RED_MAX_AREA = 5000
YELLOW_MAX_AREA = 8000
OBSTACLE_MIN_AREA = 5000
OBSTACLE_MAX_AREA = 60000
SNAP_TOLERANCE = 0.30 * GRID_STEP     # survivor must sit on an intersection
OBSTACLE_CLEARANCE_CELLS = 0.65       # off-limits margin beyond the footprint


# ---------------------------------------------------------------------------
# Grid lookup table
# ---------------------------------------------------------------------------
def _finite_xy(x, y):
    """True when both coordinates are usable numbers."""
    return math.isfinite(x) and math.isfinite(y)


def _parse_grid_csv(path):
    """Read a grid table from CSV, accepting a few common column layouts."""
    with open(path, 'r', newline='', encoding='utf-8-sig') as handle:
        rows = [row for row in csv.reader(handle) if any(cell.strip() for cell in row)]
    if not rows:
        return {}

    header = [cell.strip().lower() for cell in rows[0]]
    name_keys = ('marker_id', 'marker', 'id', 'intersection', 'cell', 'grid', 'name')

    if 'x' in header and 'y' in header:
        xi, yi = header.index('x'), header.index('y')
        try:
            ni = next(i for i, name in enumerate(header) if name in name_keys)
        except StopIteration:
            return {}
        data_rows = rows[1:]
    else:
        ni = next((i for i, cell in enumerate(rows[0])
                   if cell_to_cr(cell.strip()) is not None), None)
        if ni is None:
            return {}
        others = [i for i in range(len(rows[0])) if i != ni]
        if len(others) < 2:
            return {}
        xi, yi = others[0], others[1]
        data_rows = rows

    table = {}
    for row in data_rows:
        if len(row) <= max(ni, xi, yi):
            continue
        name = row[ni].strip().upper()
        cr = cell_to_cr(name)
        if cr is None:
            continue
        name = cell_name(*cr)
        try:
            x = float(row[xi].strip())
            y = float(row[yi].strip())
        except ValueError:
            continue
        if not _finite_xy(x, y):
            continue
        table[name] = {'x': x, 'y': y}
    return table


def _parse_grid_json(path):
    with open(path, 'r', encoding='utf-8') as handle:
        raw = json.load(handle)
    table = {}
    for name, cell in raw.items():
        if not isinstance(cell, dict) or 'x' not in cell or 'y' not in cell:
            continue
        name = str(name).strip().upper()
        cr = cell_to_cr(name)
        if cr is None:
            continue
        name = cell_name(*cr)
        try:
            x = float(cell['x'])
            y = float(cell['y'])
        except (TypeError, ValueError):
            continue
        if not _finite_xy(x, y):
            continue
        table[name] = {'x': x, 'y': y}
    return table


def load_grid():
    """Load the WhyCode coordinate table, CSV first (the task's format)."""
    candidates = []
    try:
        share = get_package_share_directory('swift_pico')
        candidates.extend([
            os.path.join(share, 'scripts', 'grid_whycode_coordinates.csv'),
            os.path.join(share, 'scripts', 'grid_whycode_coordinates.json'),
            os.path.join(share, 'config', 'grid_whycode_coordinates.csv'),
        ])
    except Exception:
        pass

    home = os.path.expanduser('~')
    base = os.path.join(home, 'pico_ws', 'src', 'swift_pico')
    candidates.extend([
        os.path.join(base, 'scripts', 'grid_whycode_coordinates.csv'),
        os.path.join(base, 'scripts', 'grid_whycode_coordinates.json'),
        os.path.join(base, 'src', 'grid_whycode_coordinates.json'),
    ])

    for path in candidates:
        if not os.path.isfile(path):
            continue
        try:
            table = (_parse_grid_csv(path) if path.lower().endswith('.csv')
                     else _parse_grid_json(path))
            if table:
                print(f'[waypoint_service] grid table: {path} '
                      f'({len(table)} intersections)')
                if len(table) < GRID_COUNT * GRID_COUNT:
                    print('[waypoint_service] WARNING: grid table is incomplete '
                          f'({len(table)}/{GRID_COUNT * GRID_COUNT} entries); '
                          'a route through a missing cell will not be planned')
                return table
        except Exception as exc:                                # pragma: no cover
            print(f'[waypoint_service] could not read {path}: {exc}')

    print('[waypoint_service] WARNING: no grid_whycode_coordinates table found')
    return {}


def grid_xy(col, row):
    """Pixel position of intersection (col, row) in the rectified frame."""
    return (GRID_MARGIN + GRID_STEP * col, GRID_MARGIN + GRID_STEP * row)


def cell_name(col, row):
    return f'{COLS[col]}{row + 1}'


def cell_to_cr(name):
    """'D5' -> (3, 4).  Returns None for an invalid name."""
    name = name.strip().upper()
    if len(name) < 2 or name[0] not in COLS:
        return None
    try:
        row = int(name[1:]) - 1
    except ValueError:
        return None
    if not 0 <= row < GRID_COUNT:
        return None
    return COLS.index(name[0]), row


def nearest_cell(px, py):
    """Nearest named intersection and its pixel distance."""
    col = int(math.floor((px - GRID_MARGIN) / GRID_STEP + 0.5))
    row = int(math.floor((py - GRID_MARGIN) / GRID_STEP + 0.5))
    col = max(0, min(GRID_COUNT - 1, col))
    row = max(0, min(GRID_COUNT - 1, row))
    gx, gy = grid_xy(col, row)
    return (col, row), math.hypot(px - gx, py - gy)


# ---------------------------------------------------------------------------
# Task 1A perception pipeline
# ---------------------------------------------------------------------------
_ARUCO_DETECTOR = None


def aruco_detector():
    """Create (once) a detector compatible with old and new OpenCV APIs."""
    global _ARUCO_DETECTOR
    if _ARUCO_DETECTOR is not None:
        return _ARUCO_DETECTOR

    aruco = cv2.aruco
    if hasattr(aruco, 'getPredefinedDictionary'):
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    elif hasattr(aruco, 'Dictionary_get'):
        dictionary = aruco.Dictionary_get(aruco.DICT_4X4_250)
    else:
        raise RuntimeError('no compatible ArUco dictionary API')

    if hasattr(aruco, 'ArucoDetector'):
        _ARUCO_DETECTOR = (
            aruco.ArucoDetector(dictionary, aruco.DetectorParameters()), True)
    elif hasattr(aruco, 'detectMarkers'):
        _ARUCO_DETECTOR = (dictionary, False)
    else:
        raise RuntimeError('no compatible ArUco detection API')
    return _ARUCO_DETECTOR


def detect_aruco(gray):
    detector, modern = aruco_detector()
    if modern:
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, detector)
    if ids is None:
        return {}
    return {int(marker_id): points.reshape(4, 2).astype(np.float32)
            for marker_id, points in zip(ids.flatten(), corners)}


def rectify_arena(bgr, markers):
    """Warp the arena to SIZE x SIZE using the four corner ArUco markers."""
    if all(marker_id in markers for marker_id in ARUCO_IDS):
        # Task 1A's exact corner correspondence: these are the arena-image
        # corners, not the outside corners of the printed markers.
        source = np.array([
            markers[80][0], markers[85][1],
            markers[90][2], markers[95][3],
        ], dtype=np.float32)
    else:
        # Fallback: pick the detected marker nearest to each image corner.
        if len(markers) < 4:
            return None
        centers = np.array([points.mean(axis=0) for points in markers.values()],
                           dtype=np.float32)
        height, width = bgr.shape[:2]
        targets = np.array([[0, 0], [width - 1, 0],
                            [width - 1, height - 1], [0, height - 1]],
                           dtype=np.float32)
        used, chosen = set(), []
        for target in targets:
            for index in np.argsort(np.linalg.norm(centers - target, axis=1)):
                index = int(index)
                if index not in used:
                    used.add(index)
                    chosen.append(centers[index])
                    break
        if len(chosen) < 4:
            return None
        source = np.array(chosen, dtype=np.float32)

    target = np.array([[0, 0], [SIZE - 1, 0],
                       [SIZE - 1, SIZE - 1], [0, SIZE - 1]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(source, target)
    return cv2.warpPerspective(bgr, homography, (SIZE, SIZE))


def colour_mask(arena, ranges):
    """Binary mask for one or more HSV ranges, with light noise cleanup."""
    hsv = cv2.cvtColor(arena, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in ranges:
        mask = cv2.bitwise_or(
            mask,
            cv2.inRange(hsv, np.array(low, np.uint8),
                        np.array(high, np.uint8)))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def black_mask(arena):
    """Solid neutral-black obstacle mask (trees are green, not neutral)."""
    hsv = cv2.cvtColor(arena, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array((0, 0, 0), np.uint8),
                       np.array((180, BLACK_S_MAX, BLACK_V_MAX), np.uint8))
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_coloured_survivors(arena, ranges, area_min, area_max):
    """Find coloured blobs and snap them to their nearest intersection."""
    mask = colour_mask(arena, ranges)
    count, _labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

    found = {}
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if not area_min <= area <= area_max:
            continue
        px, py = float(centroids[index][0]), float(centroids[index][1])
        (col, row), distance = nearest_cell(px, py)
        # A survivor is printed on an intersection.  Anything farther than a
        # third of a cell is texture, not a survivor.
        if distance > SNAP_TOLERANCE:
            continue
        name = cell_name(col, row)
        candidate = {
            'name': name, 'col': col, 'row': row, 'area': area,
            'px': px, 'py': py, 'distance': distance,
        }
        if name not in found or area > found[name]['area']:
            found[name] = candidate
    return list(found.values())


def find_critical_survivors(arena):
    """Three red triangles; retry with relaxed HSV before giving up."""
    found = find_coloured_survivors(
        arena, [RED_LOW, RED_HIGH], SURVIVOR_MIN_AREA, RED_MAX_AREA)
    if len(found) < EXPECTED_CRITICAL:
        relaxed = find_coloured_survivors(
            arena, [RED_LOW_RELAXED, RED_HIGH_RELAXED],
            SURVIVOR_MIN_AREA, RED_MAX_AREA)
        if len(relaxed) > len(found):
            found = relaxed
    return found


def find_stable_survivors(arena):
    return find_coloured_survivors(
        arena, [YELLOW_LOW], SURVIVOR_MIN_AREA, YELLOW_MAX_AREA)


def _inflate_hull(hull, radius_px):
    """Expand a convex hull by radius_px (circular offset via a mask)."""
    mask = np.zeros((SIZE, SIZE), np.uint8)
    cv2.fillConvexPoly(mask, np.asarray(hull, np.int32), 255)
    radius = max(1, int(math.ceil(radius_px)))
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    mask = cv2.dilate(mask, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return np.asarray(hull, np.float32)
    largest = max(contours, key=cv2.contourArea)
    return cv2.convexHull(largest).reshape(-1, 2).astype(np.float32)


def find_obstacles(arena):
    """Find the three solid black props and return their off-limits hulls."""
    mask = black_mask(arena)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    clearance_px = OBSTACLE_CLEARANCE_CELLS * GRID_STEP

    found = []
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if not OBSTACLE_MIN_AREA <= area <= OBSTACLE_MAX_AREA:
            continue

        component = (labels == index).astype(np.uint8)
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        points = np.vstack([contour.reshape(-1, 2) for contour in contours])
        hull = cv2.convexHull(points).reshape(-1, 2)
        off_limits = _inflate_hull(hull, clearance_px)
        px, py = float(centroids[index][0]), float(centroids[index][1])
        (col, row), _distance = nearest_cell(px, py)
        found.append({
            'area': area,
            'px': px, 'py': py,
            'hull': hull.astype(np.float32),
            'off_limits': off_limits,
            'near': cell_name(col, row),
        })

    found.sort(key=lambda item: item['area'], reverse=True)
    return found


# ---------------------------------------------------------------------------
# Geometry and clearance-aware route planning
# ---------------------------------------------------------------------------
def point_in_polygon(polygon, point):
    """True when point is inside or on the boundary of a convex polygon."""
    return cv2.pointPolygonTest(
        np.asarray(polygon, np.float32),
        (float(point[0]), float(point[1])), False) >= 0.0


def inside_any(polygons, point):
    """True when point is inside (or on) any of the off-limits polygons."""
    return any(point_in_polygon(polygon, point) for polygon in polygons)


def _orientation(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0]))


def _segments_cross(a, b, c, d):
    """Strict segment intersection (touching is handled by the margin)."""
    d1 = _orientation(c, d, a)
    d2 = _orientation(c, d, b)
    d3 = _orientation(a, b, c)
    d4 = _orientation(a, b, d)
    return (((d1 > 0.0 and d2 < 0.0) or (d1 < 0.0 and d2 > 0.0))
            and ((d3 > 0.0 and d4 < 0.0) or (d3 < 0.0 and d4 > 0.0)))


class ClearanceChecker:
    """Exact segment/polygon clearance test with the edges pre-vectorised.

    The visibility graph tests every pair of free intersections.  The naive
    Python loop over the polygon edges made that O(pairs * edges) work in
    interpreted code and took ~7 s per graph (measured on the Task 3A
    detections).  The same strict-intersection predicate is evaluated here
    with NumPy over all edges at once, which keeps the geometry identical --
    endpoints on or inside a polygon block, and only strict edge crossings
    block -- while removing the interpreter overhead.
    """

    def __init__(self, polygons):
        self.source = polygons
        self.polygons64 = [np.asarray(polygon, np.float64)
                           for polygon in polygons]
        self.polygons32 = [np.asarray(polygon, np.float32)
                           for polygon in polygons]
        if self.polygons64:
            self.edge_a = np.vstack(self.polygons64)
            self.edge_b = np.vstack([np.roll(polygon, -1, axis=0)
                                     for polygon in self.polygons64])
        else:
            self.edge_a = np.empty((0, 2), np.float64)
            self.edge_b = np.empty((0, 2), np.float64)

    def is_clear(self, p1, p2):
        for polygon in self.polygons32:
            if cv2.pointPolygonTest(polygon, (float(p1[0]), float(p1[1])),
                                    False) >= 0.0:
                return False
            if cv2.pointPolygonTest(polygon, (float(p2[0]), float(p2[1])),
                                    False) >= 0.0:
                return False
        if not len(self.edge_a):
            return True

        a = self.edge_a
        b = self.edge_b
        ax, ay = a[:, 0], a[:, 1]
        bx, by = b[:, 0], b[:, 1]
        p1x, p1y = float(p1[0]), float(p1[1])
        p2x, p2y = float(p2[0]), float(p2[1])

        d1 = (bx - ax) * (p1y - ay) - (by - ay) * (p1x - ax)
        d2 = (bx - ax) * (p2y - ay) - (by - ay) * (p2x - ax)
        d3 = (p2x - p1x) * (ay - p1y) - (p2y - p1y) * (ax - p1x)
        d4 = (p2x - p1x) * (by - p1y) - (p2y - p1y) * (bx - p1x)

        first = ((d1 > 0.0) & (d2 < 0.0)) | ((d1 < 0.0) & (d2 > 0.0))
        second = ((d3 > 0.0) & (d4 < 0.0)) | ((d3 < 0.0) & (d4 > 0.0))
        return not bool(np.any(first & second))


_CLEARANCE_CACHE = {}


def _clearance(polygons):
    """Reuse the prepared checker for an unchanged polygon list."""
    cached = _CLEARANCE_CACHE.get(id(polygons))
    if cached is not None and cached.source is polygons:
        return cached
    if len(_CLEARANCE_CACHE) >= 8:
        _CLEARANCE_CACHE.clear()
    checker = ClearanceChecker(polygons)
    _CLEARANCE_CACHE[id(polygons)] = checker
    return checker


def segment_is_clear(p1, p2, polygons):
    """True when the straight segment stays out of every off-limits area."""
    return _clearance(polygons).is_clear(p1, p2)


def _dijkstra(adjacency, source):
    """Shortest paths on a non-negative weighted graph (small graph)."""
    count = len(adjacency)
    dist = [math.inf] * count
    prev = [-1] * count
    visited = [False] * count
    dist[source] = 0.0

    for _ in range(count):
        best = -1
        best_distance = math.inf
        for index in range(count):
            if not visited[index] and dist[index] < best_distance:
                best = index
                best_distance = dist[index]
        if best < 0:
            break
        visited[best] = True
        for neighbour, cost in adjacency[best]:
            candidate = best_distance + cost
            if candidate < dist[neighbour]:
                dist[neighbour] = candidate
                prev[neighbour] = best
    return dist, prev


def _reconstruct(prev, source, target):
    if source == target:
        return [source]
    path = []
    node = target
    while node != -1 and node != source:
        path.append(node)
        node = prev[node]
    if node != source:
        return []
    path.append(source)
    path.reverse()
    return path


class RouteGraph:
    """Visibility graph over free grid intersections plus an optional start."""

    def __init__(self, grid, polygons, extra_point=None):
        self.grid = grid
        self.polygons = polygons
        self.nodes = []
        self.labels = []
        self.index = {}

        for row in range(GRID_COUNT):
            for col in range(GRID_COUNT):
                point = grid_xy(col, row)
                if inside_any(polygons, point):
                    continue
                name = cell_name(col, row)
                self.index[name] = len(self.nodes)
                self.nodes.append(point)
                self.labels.append(name)

        self.start_index = None
        if extra_point is not None:
            point = (float(extra_point[0]), float(extra_point[1]))
            if not inside_any(polygons, point):
                self.start_index = len(self.nodes)
                self.nodes.append(point)
                self.labels.append('START')

        self.adjacency = [[] for _ in self.nodes]
        clear = _clearance(polygons)
        nodes = self.nodes
        for first in range(len(nodes)):
            p1 = nodes[first]
            for second in range(first + 1, len(nodes)):
                p2 = nodes[second]
                if not clear.is_clear(p1, p2):
                    continue
                cost = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
                self.adjacency[first].append((second, cost))
                self.adjacency[second].append((first, cost))

    def cell_index(self, cell):
        return self.index.get(cell)

    def shortest_paths(self, source):
        return _dijkstra(self.adjacency, source)


def _best_tour(graph, start_index, stop_indices, end_index):
    """Try all 3! visiting orders and return the shortest total path."""
    sources = [start_index] + stop_indices
    if end_index is not None:
        sources.append(end_index)
    distances, predecessors = {}, {}
    for source in set(sources):
        distances[source], predecessors[source] = graph.shortest_paths(source)

    best = None
    for order in itertools.permutations(range(len(stop_indices))):
        sequence = [start_index] + [stop_indices[i] for i in order]
        if end_index is not None:
            sequence.append(end_index)
        total = 0.0
        possible = True
        for first, second in zip(sequence, sequence[1:]):
            leg = distances[first][second]
            if not math.isfinite(leg):
                possible = False
                break
            total += leg
        if not possible:
            continue
        if best is None or total < best[0] - 1e-9:
            best = (total, order, sequence)

    if best is None:
        return None

    total, order, sequence = best
    legs = []
    for first, second in zip(sequence, sequence[1:]):
        path = _reconstruct(predecessors[first], first, second)
        if not path:
            return None
        legs.append(path)
    return {
        'total_px': total,
        'order': order,
        'sequence': sequence,
        'legs': legs,
    }


def _compress_track(track):
    """Drop interior points on a straight run, keeping first, corners, last.

    track is a list of ((x, y), label) pairs.  Unlike compressing a bare list
    of points, this keeps every label aligned with its own point.
    """
    if len(track) <= 2:
        return list(track)
    result = [track[0]]
    for index in range(1, len(track) - 1):
        previous = track[index - 1][0]
        current = track[index][0]
        following = track[index + 1][0]
        cross = ((current[0] - previous[0]) * (following[1] - previous[1])
                 - (current[1] - previous[1]) * (following[0] - previous[0]))
        if abs(cross) > 1e-6:
            result.append(track[index])
    result.append(track[-1])
    return result


def _route_from_legs(graph, tour, stop_cells, end_cell=None):
    """Convert a tour's node paths into route waypoints and marker ids.

    stop_cells is already in visiting order (tour['order'] applied): leg i
    ends at stop_cells[i].  When end_cell is given, the extra final leg ends
    at the home intersection instead of at a survivor.
    """
    waypoints, marker_ids = [], []
    for leg_index, path in enumerate(tour['legs']):
        if leg_index < len(stop_cells):
            target = stop_cells[leg_index]
        elif end_cell is not None:
            target = end_cell
        else:
            return [], []

        track = [(graph.nodes[index], graph.labels[index]) for index in path]
        # Drop the leg's first point: the previous stop (or the drone's
        # actual position at the Switch) is already there.
        track = track[1:]
        if not track:
            track = [(graph.nodes[path[-1]], target)]
        track = _compress_track(track)

        for point, label in track[:-1]:
            if label == 'START' or not label:
                continue
            if marker_ids and marker_ids[-1].rstrip('*') == label:
                continue
            waypoints.append((float(point[0]), float(point[1])))
            marker_ids.append(label + '*')

        # The final point of every leg is the survivor or home stop.
        if not marker_ids or marker_ids[-1].rstrip('*') != target:
            end_point = track[-1][0]
            waypoints.append((float(end_point[0]), float(end_point[1])))
            marker_ids.append(target)
    return waypoints, marker_ids


def _tour_length_cells(graph, tour):
    return tour['total_px'] / GRID_STEP if tour else math.inf


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------
class WayPoints(Node):

    def __init__(self):
        super().__init__('waypoints_service')

        self.critical = []
        self.stable = []
        self.obstacles = []
        self.critical_route = None
        self.route_ready = False
        self.frames_seen = 0
        self.last_attempt = 0.0
        self.grid = load_grid()
        self.bridge = CvBridge()
        self.image_sub = None
        self.srv = None
        self.stable_srv = None
        self.debug_dir = os.path.expanduser('~')
        self.arena = None

        self._subscribe()

    # ------------------------------------------------------------------
    # image subscription (one latched frame)
    # ------------------------------------------------------------------
    def _subscribe(self):
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos)
        self.get_logger().info(
            'subscribed to /image_raw (reliable, transient_local, depth 1)')

    def _drop_subscription(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.get_logger().info(
                'dropped the /image_raw subscription (one frame is enough)')

    def image_callback(self, msg):
        if self.route_ready:
            self._drop_subscription()
            return

        self.frames_seen += 1
        now = time.monotonic()
        if now - self.last_attempt < RETRY_PERIOD:
            return
        self.last_attempt = now

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:                                # pragma: no cover
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        if self.frames_seen == 1:
            self._save_png('task_3a_capture.png', bgr)

        try:
            result = self._detect_and_plan(bgr)
        except Exception as exc:                                # pragma: no cover
            self.get_logger().error(
                f'frame {self.frames_seen}: detection/planning failed: {exc}')
            return

        if result is None:
            if self.frames_seen <= 3 or self.frames_seen % 15 == 0:
                self.get_logger().warn(
                    f'frame {self.frames_seen}: complete arena not found yet; '
                    'retrying')
            return

        self.route_ready = True
        self._drop_subscription()

    # ------------------------------------------------------------------
    # detection + planning
    # ------------------------------------------------------------------
    def _detect_and_plan(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        markers = detect_aruco(gray)
        if len(markers) < 4:
            self.get_logger().warn(
                f'ArUco markers found: {sorted(markers)}; need the four corners')
            return None

        arena = rectify_arena(bgr, markers)
        if arena is None:
            self.get_logger().warn('arena rectification failed')
            return None

        critical = find_critical_survivors(arena)
        stable = find_stable_survivors(arena)
        obstacles = find_obstacles(arena)

        if len(critical) > EXPECTED_CRITICAL:
            critical.sort(key=lambda item: (item['distance'], -item['area']))
            critical = critical[:EXPECTED_CRITICAL]
        if len(stable) > EXPECTED_STABLE:
            stable.sort(key=lambda item: (item['distance'], -item['area']))
            stable = stable[:EXPECTED_STABLE]
        if len(obstacles) > EXPECTED_OBSTACLES:
            obstacles = obstacles[:EXPECTED_OBSTACLES]

        if (len(critical) != EXPECTED_CRITICAL
                or len(stable) != EXPECTED_STABLE
                or len(obstacles) != EXPECTED_OBSTACLES):
            self.get_logger().warn(
                'detected critical={} stable={} obstacles={} (need {}/{}/{}): '
                '{} | {} | {}'.format(
                    len(critical), len(stable), len(obstacles),
                    EXPECTED_CRITICAL, EXPECTED_STABLE, EXPECTED_OBSTACLES,
                    ', '.join(s['name'] for s in critical) or '-',
                    ', '.join(s['name'] for s in stable) or '-',
                    ', '.join(o['near'] for o in obstacles) or '-'))
            return None

        # Look up WhyCode coordinates for every survivor.
        critical_data = self._attach_grid(critical)
        stable_data = self._attach_grid(stable)
        if critical_data is None or stable_data is None:
            return None

        polygons = [obstacle['off_limits'] for obstacle in obstacles]
        graph = RouteGraph(self.grid, polygons)
        start_index = graph.cell_index(HOME_CELL)
        stop_indices = [graph.cell_index(item['name']) for item in critical_data]
        if start_index is None or any(index is None for index in stop_indices):
            self.get_logger().error(
                'critical route endpoints are missing from the free graph '
                '(an obstacle may cover a survivor or F6)')
            return None

        tour = _best_tour(graph, start_index, stop_indices, None)
        if tour is None:
            self.get_logger().error('critical route planning failed')
            return None

        stop_cells = [item['name'] for item in critical_data]
        ordered_stops = [stop_cells[index] for index in tour['order']]
        route_points, marker_ids = _route_from_legs(
            graph, tour, ordered_stops, None)
        route = self._make_route(
            route_points, marker_ids, critical_data, obstacles, tour, graph)
        if route is None or not route['waypoints']:
            self.get_logger().error('critical route could not be converted')
            return None

        self.critical = critical_data
        self.stable = stable_data
        self.obstacles = obstacles
        self.arena = arena
        self.critical_route = route

        self._save_perception_image(arena, critical_data, stable_data, obstacles)
        try:
            self._save_route_image(arena, route, 'critical')
        except Exception as exc:                                # pragma: no cover
            self.get_logger().warn(f'critical route overlay not saved: {exc}')
        self._save_detections()

        self.get_logger().info(
            'critical survivors: ' + ', '.join(
                f"{item['name']}({item['x']:+.2f},{item['y']:+.2f})"
                for item in critical_data))
        self.get_logger().info(
            'stable survivors: ' + ', '.join(
                f"{item['name']}({item['x']:+.2f},{item['y']:+.2f})"
                for item in stable_data))
        self.get_logger().info(
            'obstacles: ' + ', '.join(
                f"{item['near']}(area {item['area']})" for item in obstacles))
        self.get_logger().info(
            'critical visiting order: ' + ' -> '.join(
                stop_cells[index] for index in tour['order']))
        self.get_logger().info(
            'critical route: ' + ' -> '.join(
                [HOME_CELL] + [marker_id.rstrip('*')
                               for marker_id in marker_ids]) +
            f"  ({route['length_steps']:.1f} grid steps, "
            f"~{route['length_units']:.1f} whycode units, "
            f"{len(route['vias'])} pass-through intersections)")
        return route

    def _attach_grid(self, survivors):
        result = []
        for survivor in survivors:
            cell = self.grid.get(survivor['name'])
            if cell is None:
                self.get_logger().error(
                    f"no grid coordinate for {survivor['name']}")
                return None
            result.append({
                **survivor,
                'x': float(cell['x']),
                'y': float(cell['y']),
            })
        return result

    def _make_route(self, points_px, marker_ids, stops, obstacles, tour, graph):
        waypoints, ids, vias = [], [], []
        stop_by_name = {item['name']: item for item in stops}
        for index, ((px, py), marker_id) in enumerate(zip(points_px, marker_ids)):
            cell = marker_id.rstrip('*')
            is_stop = not marker_id.endswith('*')
            if is_stop:
                item = stop_by_name.get(cell)
                if item is not None:
                    x, y = item['x'], item['y']
                else:
                    table_cell = self.grid.get(cell)
                    if table_cell is None:
                        self.get_logger().error(
                            f'no grid coordinate for route cell {cell}')
                        return None
                    x, y = float(table_cell['x']), float(table_cell['y'])
            else:
                table_cell = self.grid.get(cell)
                if table_cell is None:
                    self.get_logger().error(
                        f'no grid coordinate for route cell {cell}')
                    return None
                x, y = float(table_cell['x']), float(table_cell['y'])
                vias.append(cell)
            waypoints.append((x, y, Z_WAYPOINT))
            ids.append(marker_id)

        return {
            'waypoints': waypoints,
            'marker_ids': ids,
            'vias': vias,
            'length_steps': _tour_length_cells(graph, tour),
            'length_units': _tour_length_cells(graph, tour) * GRID_SPACING,
            'order': [stops[index]['name'] for index in tour['order']],
        }

    # ------------------------------------------------------------------
    # stable route service (planned from the Switch position)
    # ------------------------------------------------------------------
    def whycode_to_pixel(self, x, y):
        """Map a WhyCode (x, y) pose to the rectified arena image."""
        if not self.grid:
            return None
        nearest = min(
            self.grid.items(),
            key=lambda item: ((item[1]['x'] - x) ** 2
                              + (item[1]['y'] - y) ** 2))
        name, cell = nearest
        col, row = cell_to_cr(name)
        gx, gy = grid_xy(col, row)
        px = gx - (x - cell['x']) / GRID_SPACING * GRID_STEP
        py = gy - (y - cell['y']) / GRID_SPACING * GRID_STEP
        return px, py

    def plan_stable_route(self, x, y):
        if not self.stable or self.arena is None:
            self.get_logger().error('stable route requested before detection')
            return None

        pixel = self.whycode_to_pixel(x, y)
        if pixel is None:
            self.get_logger().error('no grid table for the stable route')
            return None
        px, py = pixel
        if not (0.0 <= px < SIZE and 0.0 <= py < SIZE):
            self.get_logger().warn(
                f'stable start ({x:+.2f},{y:+.2f}) maps outside the arena '
                f'({px:.1f},{py:.1f}); clamping')
            px = max(0.0, min(SIZE - 1.0, px))
            py = max(0.0, min(SIZE - 1.0, py))

        polygons = [obstacle['off_limits'] for obstacle in self.obstacles]
        start_point = (px, py)
        if inside_any(polygons, start_point):
            # The drone drifted into the inflated margin (or the cancel left
            # it just inside it).  A route cannot start from an off-limits
            # point, so leave from the nearest free intersection and say so.
            free = [(grid_xy(col, row), cell_name(col, row))
                    for row in range(GRID_COUNT)
                    for col in range(GRID_COUNT)
                    if not inside_any(polygons, grid_xy(col, row))]
            if not free:
                self.get_logger().error('no free intersection for the Switch')
                return None
            (sx, sy), name = min(
                free, key=lambda item: math.hypot(item[0][0] - px,
                                                  item[0][1] - py))
            self.get_logger().warn(
                f'stable start ({x:+.2f},{y:+.2f}) is inside an off-limits '
                f'area; leaving from {name}')
            start_point = (sx, sy)

        graph = RouteGraph(self.grid, polygons, extra_point=start_point)
        if graph.start_index is None:
            self.get_logger().error('stable start position could not be used')
            return None

        stop_indices = [graph.cell_index(item['name']) for item in self.stable]
        home_index = graph.cell_index(HOME_CELL)
        if any(index is None for index in stop_indices) or home_index is None:
            self.get_logger().error(
                'stable route endpoints are missing from the free graph')
            return None

        tour = _best_tour(graph, graph.start_index, stop_indices, home_index)
        if tour is None:
            self.get_logger().error('stable route planning failed')
            return None

        stop_cells = [item['name'] for item in self.stable]
        ordered_stops = [stop_cells[index] for index in tour['order']]
        route_points, marker_ids = _route_from_legs(
            graph, tour, ordered_stops, HOME_CELL)
        route = self._make_route(
            route_points, marker_ids, self.stable, self.obstacles, tour, graph)
        if route is None or not route['waypoints']:
            self.get_logger().error('stable route could not be converted')
            return None

        self.get_logger().info(
            'stable request from ({:+.2f},{:+.2f}) -> pixel ({:.1f},{:.1f})'
            .format(x, y, px, py))
        self.get_logger().info(
            'stable visiting order: ' + ' -> '.join(
                stop_cells[index] for index in tour['order']))
        self.get_logger().info(
            'stable route: ' + ' -> '.join(
                [f'START({x:+.2f},{y:+.2f})']
                + [marker_id.rstrip('*') for marker_id in marker_ids]) +
            f"  ({route['length_steps']:.1f} grid steps, "
            f"~{route['length_units']:.1f} whycode units, "
            f"{len(route['vias'])} pass-through intersections)")
        try:
            self._save_route_image(self.arena, route, 'stable')
        except Exception as exc:                                # pragma: no cover
            self.get_logger().warn(f'stable route overlay not saved: {exc}')
        return route

    # ------------------------------------------------------------------
    # debug images / evidence
    # ------------------------------------------------------------------
    def _save_png(self, name, image):
        path = os.path.join(self.debug_dir, name)
        try:
            cv2.imwrite(path, image)
            self.get_logger().info(f'wrote {path}')
        except Exception as exc:                                # pragma: no cover
            self.get_logger().warn(f'could not write {path}: {exc}')

    def _save_perception_image(self, arena, critical, stable, obstacles):
        image = arena.copy()
        for obstacle in obstacles:
            cv2.polylines(image, [np.asarray(obstacle['off_limits'], np.int32)],
                          True, (0, 0, 255), 2)
            cv2.polylines(image, [np.asarray(obstacle['hull'], np.int32)],
                          True, (0, 255, 255), 1)
        for item in critical:
            px, py = int(round(item['px'])), int(round(item['py']))
            cv2.circle(image, (px, py), 22, (0, 0, 255), 2)
            cv2.putText(image, 'C ' + item['name'], (px + 25, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2,
                        cv2.LINE_AA)
        for item in stable:
            px, py = int(round(item['px'])), int(round(item['py']))
            cv2.circle(image, (px, py), 22, (0, 255, 255), 2)
            cv2.putText(image, 'S ' + item['name'], (px + 25, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2,
                        cv2.LINE_AA)
        x, y = grid_xy(*HOME_CR)
        cv2.drawMarker(image, (int(x), int(y)), (255, 255, 0),
                       cv2.MARKER_CROSS, 22, 2)
        cv2.putText(image, 'F6', (int(x) + 12, int(y) + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2,
                    cv2.LINE_AA)
        self._save_png('task_3a_perception.png', image)

    def _save_route_image(self, arena, route, kind):
        image = arena.copy()

        for row in range(GRID_COUNT):
            for col in range(GRID_COUNT):
                x, y = grid_xy(col, row)
                cv2.circle(image, (int(round(x)), int(round(y))), 2,
                           (0, 220, 0), -1)

        for obstacle in self.obstacles:
            cv2.polylines(image, [np.asarray(obstacle['off_limits'], np.int32)],
                          True, (0, 0, 255), 2)

        # Route polyline in image space.
        points = []
        for x, y, _z in route['waypoints']:
            pixel = self.whycode_to_pixel(x, y)
            if pixel is None:
                continue
            points.append([int(round(pixel[0])), int(round(pixel[1]))])
        if len(points) > 1:
            cv2.polylines(image, [np.array(points, np.int32)], False,
                          (0, 165, 255), 2)

        survivors = self.critical if kind == 'critical' else self.stable
        for item in survivors:
            colour = (0, 0, 255) if kind == 'critical' else (0, 255, 255)
            px, py = int(round(item['px'])), int(round(item['py']))
            cv2.circle(image, (px, py), 24, colour, 2)
            cv2.putText(image, item['name'], (px + 28, py - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv2.LINE_AA)

        x, y = grid_xy(*HOME_CR)
        cv2.drawMarker(image, (int(x), int(y)), (255, 255, 0),
                       cv2.MARKER_CROSS, 22, 2)
        cv2.putText(image, 'F6', (int(x) + 14, int(y) + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2,
                    cv2.LINE_AA)

        for index, marker_id in enumerate(route['marker_ids'], start=1):
            if marker_id.endswith('*'):
                continue
            cell = marker_id.rstrip('*')
            cr = cell_to_cr(cell)
            if cr is None:
                continue
            gx, gy = grid_xy(*cr)
            cv2.putText(image, str(index), (int(gx) + 6, int(gy) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1,
                        cv2.LINE_AA)

        self._save_png(f'task_3a_{kind}_route.png', image)

    def _save_detections(self):
        payload = {
            'critical': [
                {key: item[key] for key in ('name', 'col', 'row', 'px', 'py',
                                            'x', 'y', 'area')}
                for item in self.critical
            ],
            'stable': [
                {key: item[key] for key in ('name', 'col', 'row', 'px', 'py',
                                            'x', 'y', 'area')}
                for item in self.stable
            ],
            'obstacles': [
                {
                    'near': item['near'],
                    'area': item['area'],
                    'hull': np.asarray(item['hull']).round(2).tolist(),
                    'off_limits': np.asarray(item['off_limits']).round(2).tolist(),
                }
                for item in self.obstacles
            ],
            'critical_route': {
                'waypoints': self.critical_route['waypoints'],
                'marker_ids': self.critical_route['marker_ids'],
                'order': self.critical_route['order'],
            },
        }
        path = os.path.join(self.debug_dir, 'task_3a_detections.json')
        try:
            with open(path, 'w', encoding='utf-8') as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
            self.get_logger().info(f'wrote {path}')
        except Exception as exc:                                # pragma: no cover
            self.get_logger().warn(f'could not write {path}: {exc}')

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def wait_for_route(self):
        """Spin until the first usable frame has produced a route."""
        self.get_logger().info('waiting for the first usable camera frame...')
        last_report = time.monotonic()
        while rclpy.ok() and not self.route_ready:
            try:
                rclpy.spin_once(self, timeout_sec=0.2)
            except ExternalShutdownException:
                break
            now = time.monotonic()
            if now - last_report >= 5.0:
                last_report = now
                self.get_logger().info(
                    f'still waiting for a complete route '
                    f'(frames seen: {self.frames_seen})')
        if self.route_ready:
            self.get_logger().info(
                'route ready: the waypoint services can now answer requests')
        return self.route_ready

    def create_waypoint_service(self):
        self.srv = self.create_service(
            GetWaypoints, 'waypoints', self.waypoint_callback)
        self.stable_srv = self.create_service(
            PlanStableRoute, 'stable_waypoints', self.stable_callback)
        self.get_logger().info(
            'critical route ready: ' + ', '.join(
                f'{index + 1}={waypoint}{marker_id}'
                for index, (waypoint, marker_id)
                in enumerate(zip(self.critical_route['waypoints'],
                                 self.critical_route['marker_ids']))))
        self.get_logger().info(
            'stable service ready: stable_waypoints')

    def waypoint_callback(self, request, response):
        if not request.get_waypoints:
            self.get_logger().info('Request rejected')
            return response

        if self.critical_route is None:
            self.get_logger().warn('route not ready; refusing to answer')
            return response

        route = self.critical_route
        response.waypoints.poses = [Pose() for _ in route['waypoints']]
        for index, (x, y, z) in enumerate(route['waypoints']):
            response.waypoints.poses[index].position.x = float(x)
            response.waypoints.poses[index].position.y = float(y)
            response.waypoints.poses[index].position.z = float(z)
        response.marker_ids = list(route['marker_ids'])

        self.get_logger().info('Incoming request for Waypoints')
        self.get_logger().info(
            'Returning ' + ' -> '.join(
                f'{marker_id}'
                f'({"stop" if not marker_id.endswith("*") else "via"})'
                for marker_id in route['marker_ids']))
        return response

    def stable_callback(self, request, response):
        started = time.monotonic()
        position = request.start.position
        x, y = float(position.x), float(position.y)
        if not _finite_xy(x, y):
            self.get_logger().error(
                'stable request has a non-finite start position')
            return response
        if self.critical_route is None:
            self.get_logger().warn('stable route not ready; refusing to answer')
            return response

        route = self.plan_stable_route(x, y)
        if route is None:
            self.get_logger().error('stable route could not be planned')
            return response
        self.get_logger().info(
            f'stable route planned in {time.monotonic() - started:.2f} s')

        response.waypoints.poses = [Pose() for _ in route['waypoints']]
        for index, (wx, wy, wz) in enumerate(route['waypoints']):
            response.waypoints.poses[index].position.x = float(wx)
            response.waypoints.poses[index].position.y = float(wy)
            response.waypoints.poses[index].position.z = float(wz)
        response.marker_ids = list(route['marker_ids'])
        self.get_logger().info(
            'Returning stable route with ' + str(len(route['waypoints'])) +
            ' waypoints')
        return response


def main(args=None):
    rclpy.init(args=args)
    waypoints = WayPoints()

    ready = waypoints.wait_for_route()
    if ready:
        waypoints.create_waypoint_service()
    else:
        waypoints.get_logger().error(
            'no route was planned; the services will not be advertised')

    if not rclpy.ok():
        waypoints.destroy_node()
        return

    try:
        rclpy.spin(waypoints)
    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            waypoints.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoints.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

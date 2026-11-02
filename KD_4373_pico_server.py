#!/usr/bin/env python3
"""Task 3A - action server that flies the Swift Pico through the full mission.

The node implements the ROS 2 action 'waypoint_navigation' (action type
waypoint_navigation/action/NavToWaypoint):

    geometry_msgs/Pose waypoint      # goal: the waypoint to fly to (WhyCode)
    bool hover
    ---
    int32 hov_time                   # result: seconds needed for the goal
    ---
    geometry_msgs/PoseStamped current_waypoint   # feedback: live WhyCode pose

Goal handling:

    hover=False  a pass-through grid intersection; succeed as soon as the drone
                 passes close enough, so the client sends the next leg at once.

    hover=True   a survivor stop.  Fly to (x, y, Z=13), hold the Visit inside
                 the box for 1 s, descend to Z=14, then hold the Save inside
                 the box for 2 s.  The timer restarts from zero if the drone
                 leaves the box.

    F6           recognised by its WhyCode coordinates from the course grid
                 table.  Hold at Z=13 for 1 s, then descend, settle on the
                 ground, disarm and report success.

Cancels are accepted: when the client hits the 55 s Switch mid-goal, the drone
stops where it is and holds station until the stable route's first goal
arrives.

CONTROL LAW - a cascaded position/velocity loop (the structure that scored
20/20 in Task 2A, re-tuned here for transits between waypoints):

    position error  ->  target velocity  ->  rc command

    v_des = clamp(K_POS * error, +/- V_MAX)
    rc    = RC_MID + K_RC * (v_des - v) + K_INT * integral(error)

The velocity feedback is not optional: the airframe is nearly ballistic (no
drag) and the attitude lags the rc command by 1-2 s, so a plain proportional
loop rings and never settles.  The velocity is a windowed finite difference
over the WhyCode markers (>= 0.12 s window, then an EMA).

PLANT MODEL - measured in this simulator (see rosbag KD_4373_task_2a):

  * the WhyCode camera hangs at z = 15 m and looks down, so the reported Z is
    the DEPTH below the camera: altitude = 15 - z;
  * rc_pitch above 1500 moves the drone towards +X, rc_roll above 1500 towards
    +Y, rc_throttle above 1500 climbs (Z decreases);
  * rotors_swift_interface maps rc to thrust as
        rc >  1499 : T = 14.896 + (rc - 1500) * 15.104/500  [N]
        rc <= 1499 : T = (1 + (rc - 1500)/500) * 14.896      [N]
    and the drone hovers at rc ~ 1517, i.e. the airframe needs ~15.41 N;
  * rc -> roll/pitch angle = (rc - 1500) * 4.5 deg / 85;
  * tilting costs vertical thrust (T * cos(tilt)), so the throttle has to be
    compensated explicitly, otherwise the altitude sags during a fast transit;
  * on the ground a tilt command drags the drone over and kills the marker
    stream, so the XY authority is ramped in with the altitude.

Published:
    /drone_command        swift_msgs/msg/SwiftMsgs
    /pos_error            error_msg/msg/Error
    /controller_heartbeat std_msgs/msg/Header

Subscribed:
    /whycode_node/markers whycode_interfaces/msg/MarkerArray
"""

import json
import math
import os
import signal
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from error_msg.msg import Error
from std_msgs.msg import Header
from swift_msgs.msg import SwiftMsgs
from whycode_interfaces.msg import MarkerArray

from waypoint_navigation.action import NavToWaypoint


# ===========================================================================
# Airframe, plant model and rc <-> physics mapping
# ===========================================================================
RC_MIN = 1000.0
RC_MAX = 2000.0
RC_MID = 1500.0

CAMERA_HEIGHT = 15.0        # arena.xml: the ceiling camera sits at z = 15 m

THRUST_AT_1500 = 14.896     # N, thrust produced by rc = 1500
THRUST_MAX = 30.0           # N, thrust produced by rc = 2000
THRUST_PER_COUNT = (THRUST_MAX - THRUST_AT_1500) / 500.0     # N per rc

HOVER_THROTTLE = 1518.0     # rc that holds altitude at z ~ 13 (Task 2A fit)
HOVER_THRUST = THRUST_AT_1500 + (HOVER_THROTTLE - RC_MID) * THRUST_PER_COUNT

TILT_RAD_PER_COUNT = math.radians(4.5) / 85.0    # rc -> roll/pitch angle

# ===========================================================================
# Controller tuning
# ===========================================================================
# Horizontal (X -> rc_pitch, Y -> rc_roll)
K_POS_XY = 0.9          # 1/s        position error -> velocity target
V_MAX_XY = 2.20         # m/s        cruise speed limit
K_RC_XY = 140.0         # rc/(m/s)   velocity error -> rc
MAX_DEV_XY = 165.0      # rc         tilt clamp (150 rc = 7.9 deg; more braking authority)
K_INT_XY = 0.4          # rc/(m*s)   small bias trim
INT_LIMIT_XY = 20.0     # rc*s
INT_BAND_XY = 0.6       # only integrate this close to the target

# Vertical (Z -> rc_throttle, inverted: the reported Z is the depth).
# These are the Task 2A values that held z to +/-0.02 for a full minute.
# The Z loop must NOT have a per-tick integral: the plant gain is tiny, so an
# rc-domain integral of this size hunts in a slow ~7 s limit cycle (seen in
# the first Task 2C run, where z swung +/-0.7 and no 3 s hold could complete).
K_POS_Z = 0.60          # 1/s
V_MAX_Z = 0.80          # m/s
K_RC_Z = 105.0           # rc/(m/s)
# Drag makes the pure velocity loop settle ~33% short of the commanded rate
# (the error it needs to hold the throttle offset is the same drag).  A
# feedforward of the setpoint rate removes that steady-state loss without
# feeding any more marker noise into the throttle.
K_FF_Z = 40.0           # rc/(m/s) of commanded vertical rate
MAX_DEV_Z = 120.0       # rc
K_INT_Z = 0.0           # deliberately not wired into the Z output
INT_LIMIT_Z = 0.0       # rc*s
INT_BAND_Z = 0.6        # only integrate this close to the target

# Waypoint acceptance
BOX = 0.37              # WhyCode units, per axis (navigation/feedback only)
# The official grader does not use a per-axis box.  Its hold test is a 3-D
# sphere of radius 0.40 around the survivor centre:
#     sqrt(dx*dx + dy*dy + dz*dz) <= 0.40
# A per-axis 0.37 box is looser than that on the diagonal (a corner is 0.52
# away), which is exactly how a bag can look perfect to the local box scorer
# and still lose a survivor on the portal.  Visit/Save/F6 therefore use the
# real spherical test, at the same radius the grader uses, so the server
# cannot advance until the recorded markers themselves pass.
HOLD_RADIUS = 0.38      # whycode units, 0.02 inside the grader's 0.40 sphere
HOLD_FRESH = 0.15       # s, a stale raw_state must not count toward a hold
VISIT_Z = 13.0          # whycode depth for every Visit / fly-through / F6 hold
SAVE_Z = 14.0           # whycode depth for the Save (1 m lower)
SAVE_CTRL_Z = 14.0      # controller target depth during Save (centres the
                        # sphere; 13.92 biased the hover onto its lower edge)
SAVE_FAST_MARGIN = 1.00 # brake this far above the Save box entry; 1.0 keeps the
                        # proportional loop in charge (no open-loop 0.8 m/s dive)
VISIT_HOLD = 1.10       # seconds continuously inside the sphere for a Visit
SAVE_HOLD = 2.10        # seconds continuously inside the sphere for a Save
HOME_HOLD = 1.10        # seconds over F6 before the landing descent
LAND_Z = 15.30          # setpoint depth for the final descent
# Near the floor the altitude loop reaches equilibrium a few centimetres
# above the touchdown depth (1499 rc holds it 8 cm up), so the last part of
# the landing is flown open loop with a throttle clearly below the hover
# value.  The floor stops the descent and the airframe settles on its skids.
LAND_ENGAGE_ALT = 0.40  # m above the floor where the fixed descent starts
LAND_THROTTLE = 1390    # rc, well below the ground-effect hover throttle
LAND_V_MAX = 0.72       # m/s, slower final descent than the transit climbs
# Ground rest is depth 14.851: the ceiling camera is at 15 m and the marker
# sits 0.149 m above the floor when the airframe's collision box rests on it
# (body box bottom at -0.175 m, marker at -0.026 m).  A touchdown threshold
# must stay BELOW that rest depth, otherwise it can never be reached: the
# floor physically stops the descent at 14.85.
TOUCHDOWN_Z = 14.75     # depth at which the drone is on the floor
# The detector's absolute depth has a run-to-run bias of a few centimetres
# (rest readings have been seen from 14.73 to 15.01), so a fixed depth alone
# can miss a real touchdown.  A landing is also complete when the filtered
# depth stops increasing for a while under the fixed landing throttle: at
# that point the floor, not the controller, is holding the airframe.
TOUCHDOWN_STALL_Z = 14.65    # only trust a stall this close to the floor
TOUCHDOWN_STALL_TIME = 1.5   # s without any downward progress
TOUCHDOWN_STALL_STEP = 0.02  # depth progress that counts as still descending
TOUCHDOWN_HOLD = 0.40    # seconds to settle disarmed on F6 before success
HOME_RADIUS = 0.30      # whycode units, F6 recognition radius
PASS_BOX = 0.48         # whycode units, fly-through (hover=False) acceptance
PASS_DWELL = 0.00       # s inside PASS_BOX before a fly-through is done
GOAL_TIMEOUT = 35.0     # s, give up on a goal that cannot be completed
GOAL_Z_MIN = 0.0        # whycode units, plausible waypoint depth bounds
GOAL_Z_MAX = 25.0       # (z=13 for every Task 3A waypoint)

# Timing / housekeeping
SAMPLE_TIME = 0.033     # 30 Hz control loop
HEARTBEAT_PERIOD = 0.1
VEL_WINDOW = 0.12       # s, minimum window of the finite difference
VEL_ALPHA = 0.4         # EMA smoothing of the velocity estimate
Z_VEL_WINDOW = 0.30     # s, depth-only velocity baseline (range noise is large)
Z_VEL_MIN_SPAN = 0.22   # s, shortest usable depth history
Z_VEL_ALPHA = 0.5       # EMA smoothing of the depth velocity

# Marker outlier gate.  At 30 Hz the marker can only move a few centimetres
# between accepted samples, so a jump of this size is a stray blob, a blurred
# frame, or a foreign marker stream (e.g. a second simulator writing the same
# topics).  Such a sample is ignored and the last good pose is flown; a new
# pose is only adopted once several consecutive detections agree with each
# other, so a genuine re-detection after a dropout is still recovered.
MAX_MARKER_STEP = 0.35  # whycode units between accepted samples
GATE_DT = 0.25          # s, only gate samples this close to the last fix
RESYNC_COUNT = 5        # consistent outlier samples needed to re-sync
RESYNC_STEP = 0.35      # whycode units, agreement between candidates
RESYNC_WINDOW = 0.5     # s, candidates must arrive this close together
# Yaw hold.  The marker supplies the heading and rc_yaw is a rate command with
# a slow (multi-second) first-order response.  Feedback on the measured rate
# therefore cannot oscillate (first-order plant + proportional rate loop stays
# first order), which is what makes a strong damper safe.  The weak heading
# term only removes the slow residual drift; the position loop rotates its
# command into the body frame, so an imperfect heading costs nothing but a
# little speed.
YAW_KP = 0.4            # rc per degree of heading error (slow trim only)
YAW_KD = 12.0           # rc per (degree/s) of yaw rate
YAW_KI = 0.0            # rc per (degree*s) - unused, keeps the trim simple
YAW_INT_LIMIT = 100.0   # degree*s
YAW_MAX = 350.0         # rc, authority limit around rc_yaw = 1500
YAW_RATE_ALPHA = 0.5    # EMA weight of a new rate sample
YAW_RATE_WINDOW = 0.30  # s, baseline of the finite difference (noise filter)
YAW_RATE_MIN_SPAN = 0.12  # s, shortest usable baseline
MARKER_TIMEOUT = 0.5    # s without a marker before the drone levels out
# The detector's range (depth) estimate is much noisier than its in-plane
# position: +/-0.2 units at 30 Hz even while the airframe is level.  The Z
# loop and the Save/landing boxes therefore use a low-passed depth, otherwise
# the altitude loop chases the noise and the 2 s Save hold keeps resetting.
Z_FILTER_ALPHA = 0.12   # EMA weight per accepted marker sample (~0.27 s)
# The in-plane estimate also carries high-frequency jumps (successive samples
# can reverse by 0.1 units, i.e. 2-3 m/s, which the airframe cannot do).  The
# grader scores the raw marker, but the loop must not fly the jitter: keeping
# the raw pose for the hold test and filtering only the control/velocity path
# lets the airframe sit still, which is what makes the raw sphere hold long.
XY_FILTER_ALPHA = 0.50  # EMA weight per accepted marker sample (~0.05 s)
LIFTOFF_ALT = 0.5       # m, altitude at which the full XY authority is used
LIFTOFF_SCALE = 0.15    # XY authority while still on the ground
STATUS_PERIOD = 1.0     # s, status line period


def wrap180(angle):
    """Wrap an angle in degrees to (-180, 180]."""
    return (angle + 180.0) % 360.0 - 180.0


def load_home_xy():
    """F6's WhyCode coordinates, read from the course grid table."""
    candidates = []
    try:
        candidates.append(os.path.join(
            get_package_share_directory('swift_pico'), 'scripts',
            'grid_whycode_coordinates.json'))
    except Exception:
        pass
    candidates.append(os.path.expanduser(
        '~/pico_ws/src/swift_pico/src/grid_whycode_coordinates.json'))

    for path in candidates:
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                home = json.load(handle).get('F6')
            x, y = float(home['x']), float(home['y'])
            if math.isfinite(x) and math.isfinite(y):
                return x, y
        except Exception:
            continue
    # Course-provided fallback: F6 is the fixed take-off/landing point.
    return 0.0068, 0.0068


class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_or_lqr_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        # ---------------- state ----------------
        # Every vector below is ordered [x, y, z] so that the same index
        # always means the same axis.
        self.curr_state = [0.0, 0.0, 0.0]     # filtered pose for control
        self.raw_state = [0.0, 0.0, 0.0]      # unfiltered pose for hold tests
        self.xy_filt = None                   # low-passed marker x/y
        self.z_filt = None                    # low-passed marker depth
        self.gate_pos = None                  # last accepted raw sample (gate)
        self.z_hist = []                      # [(t, z_filt)] for vertical speed
        self.desired_state = [0.0, 0.0, 0.0]  # the waypoint being flown to
        self.alt = 0.0                        # altitude above the arena floor
        self.vel = [0.0, 0.0, 0.0]           # vx, vy, vz (vz is negative when climbing)
        self.prev_pos = None
        self.last_marker_t = None
        self.has_vel = False
        self.have_fix = False
        self.gate_candidate = None
        self.gate_candidate_t = 0.0
        self.gate_hits = 0
        self.gate_rejects = 0

        # Yaw, straight from the WhyCode marker (the marker carries its own
        # orientation) and the setpoint the task boilerplate provides.
        self.yaw = 0.0              # current yaw in degrees (updated from whycode)
        self.yaw_ref = None         # heading held for the flight (deg)
        self.yaw_rate = 0.0         # filtered yaw rate (deg/s)
        self.yaw_integral = 0.0     # heading error integral (deg*s)
        self.yaw_hist = []          # [(t, yaw)] for the rate baseline
        self.desired_yaw = -0.5     # target yaw angle (degrees)

        self.target_set = False
        self.airborne = False
        self.landing = False
        self.landed = False
        self.land_z_ref = None       # depth reference for the touchdown stall
        self.land_z_t = 0.0
        self.home_xy = load_home_xy()
        self.error = [0.0, 0.0, 0.0]          # [x, y, altitude]
        self.integral = [0.0, 0.0, 0.0]
        self.rc = [RC_MID, RC_MID, HOVER_THROTTLE]   # [rc_pitch, rc_roll, rc_throttle]

        self.box_start = None
        self.hold_time = 0.0
        self.last_status = 0.0

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = int(RC_MID)
        self.cmd.rc_pitch = int(RC_MID)
        self.cmd.rc_yaw = int(RC_MID)
        self.cmd.rc_throttle = int(HOVER_THROTTLE)
        self.cmd.rc_aux1 = 1000
        self.cmd.rc_aux2 = 1000
        self.cmd.rc_aux3 = 1000
        self.cmd.rc_aux4 = 1000
        self.cmd.drone_index = 0

        self.pos_error = Error()

        # ---------------- plumbing ----------------
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/controller_heartbeat', 10)

        self.create_subscription(MarkerArray, '/whycode_node/markers',
                                 self.whycode_callback, 1)

        # The action server for the action 'NavToWaypoint' (action name
        # 'waypoint_navigation').
        self.action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group,
        )

        self.publish_heartbeat()
        self.create_timer(HEARTBEAT_PERIOD, self.publish_heartbeat)

        time.sleep(0.5)   # let the heartbeat reach the rosbag before arming
        self.arm()

        self.sample_time = SAMPLE_TIME
        self.timer = self.create_timer(self.sample_time, self.pid,
                                       callback_group=self.pid_or_lqr_callback_group)

        self.get_logger().info(
            'waypoint action server ready (action: waypoint_navigation)')
        self._install_signal_handlers()

    def _install_signal_handlers(self):
        """Leave a stable hover behind when the node is killed."""
        def _bye(_signum, _frame):
            try:
                if self.landed:
                    self.disarm()
                else:
                    self.publish_stable_hover()
            except Exception:
                pass
            sys.exit(0)
        signal.signal(signal.SIGTERM, _bye)
        signal.signal(signal.SIGINT, _bye)

    # ------------------------------------------------------------------
    # housekeeping
    # ------------------------------------------------------------------
    def publish_heartbeat(self):
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = 'pico_controller_active'
        self.heartbeat_pub.publish(hb)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = int(RC_MID)
        self.cmd.rc_yaw = int(RC_MID)
        self.cmd.rc_pitch = int(RC_MID)
        self.cmd.rc_throttle = int(HOVER_THROTTLE)
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def publish_stable_hover(self, cycles=10):
        """Leave a level hover command behind when the node exits."""
        self.cmd.rc_roll = int(RC_MID)
        self.cmd.rc_pitch = int(RC_MID)
        self.cmd.rc_yaw = int(RC_MID)
        self.cmd.rc_throttle = int(HOVER_THROTTLE)
        self.cmd.rc_aux4 = 2000
        for _ in range(cycles):
            try:
                self.command_pub.publish(self.cmd)
            except Exception:                                   # pragma: no cover
                break
            time.sleep(0.05)

    # ------------------------------------------------------------------
    # perception
    # ------------------------------------------------------------------
    def _select_marker(self, markers):
        """Pick the drone's marker and drop non-finite detections.

        Only one WhyCode marker exists (it is mounted on the drone), but the
        detector can briefly report extra blobs while the image is blurred.
        Prefer the detection closest to the last known position, so a stray
        blob cannot teleport the control loop.
        """
        valid = []
        for marker in markers:
            position = marker.position.position
            xyz = (float(position.x), float(position.y), float(position.z))
            if all(math.isfinite(value) for value in xyz):
                valid.append((marker, xyz))
        if not valid:
            return None
        if len(valid) == 1 or self.gate_pos is None:
            return valid[0][0]
        return min(
            valid,
            key=lambda item: sum(
                (item[1][axis] - self.gate_pos[axis]) ** 2
                for axis in range(3)))[0]

    def whycode_callback(self, msg):
        if not msg.markers:
            return

        marker = self._select_marker(msg.markers)
        if marker is None:
            return

        position = marker.position.position
        x = float(position.x)
        y = float(position.y)
        z = float(position.z)
        if not all(math.isfinite(v) for v in (x, y, z)):
            return

        now = time.monotonic()
        raw = [x, y, z]

        # Reject a sample that would teleport the control loop.  The last good
        # pose keeps flying; only a burst of mutually consistent outliers is
        # adopted, which recovers a real re-detection after a dropout.
        if (self.have_fix and self.gate_pos is not None
                and self.last_marker_t is not None
                and now - self.last_marker_t <= GATE_DT):
            jump = max(abs(raw[i] - self.gate_pos[i]) for i in range(3))
            if jump > MAX_MARKER_STEP:
                recent = (
                    self.gate_candidate is not None
                    and now - self.gate_candidate_t <= RESYNC_WINDOW
                    and max(abs(raw[i] - self.gate_candidate[i])
                            for i in range(3)) <= RESYNC_STEP)
                self.gate_hits = self.gate_hits + 1 if recent else 1
                self.gate_candidate = raw
                self.gate_candidate_t = now
                if self.gate_hits < RESYNC_COUNT:
                    self.gate_rejects += 1
                    if self.gate_rejects % 30 == 1:
                        self.get_logger().warn(
                            'marker outlier rejected ({:.2f} units from the '
                            'last fix); holding the last good pose'
                            .format(jump))
                    return
                self.get_logger().warn(
                    'marker re-synced after {} consistent outliers '
                    '({:.2f} units)'.format(self.gate_hits, jump))
                self.has_vel = False
                self.vel = [0.0, 0.0, 0.0]
            else:
                self.gate_candidate = None
                self.gate_hits = 0

        # This sample passed the gate: it is the new raw reference the next
        # sample is compared against.  Keep it separate from prev_pos, which
        # now carries the filtered pose used for velocity.
        self.gate_pos = raw

        # Low-pass the depth and the in-plane position.  The gate above still
        # works on raw samples, and raw_state stays unfiltered because it is
        # the pose the grader scores.  Only the control loop and the velocity
        # estimate use the filtered pose, so marker jitter does not turn into
        # airframe motion (which would shorten the graded sphere hold).
        if self.z_filt is None:
            self.z_filt = z
        else:
            self.z_filt += Z_FILTER_ALPHA * (z - self.z_filt)
        if self.xy_filt is None:
            self.xy_filt = [x, y]
        else:
            self.xy_filt[0] += XY_FILTER_ALPHA * (x - self.xy_filt[0])
            self.xy_filt[1] += XY_FILTER_ALPHA * (y - self.xy_filt[1])
        self.raw_state = [x, y, z]
        pos = [self.xy_filt[0], self.xy_filt[1], self.z_filt]

        # Vertical speed comes from the filtered depth over a longer baseline.
        # The range channel is by far the noisiest marker measurement, and a
        # 0.12 s raw difference turns that noise into +/-0.7 m/s of false
        # vertical speed, which the throttle loop then chases.  Averaging the
        # filtered depth over 0.3 s gives a smooth rate without adding much
        # lag (the plant itself is drag dominated with a ~1 s time constant).
        self.z_hist.append((now, self.z_filt))
        while (len(self.z_hist) > 2
               and now - self.z_hist[0][0] > Z_VEL_WINDOW):
            self.z_hist.pop(0)
        span = now - self.z_hist[0][0]
        if span >= Z_VEL_MIN_SPAN:
            vz = (self.z_filt - self.z_hist[0][1]) / span
            self.vel[2] = Z_VEL_ALPHA * self.vel[2] + (1.0 - Z_VEL_ALPHA) * vz

        if self.have_fix and self.prev_pos is not None and self.last_marker_t is not None:
            dt = now - self.last_marker_t
            if dt >= 0.40:
                # stale feed: drop the velocity estimate
                self.prev_pos = pos
                self.last_marker_t = now
                self.has_vel = False
                self.vel = [0.0, 0.0, 0.0]
            elif dt >= VEL_WINDOW:
                # windowed finite difference on the filtered pose: a raw-sample
                # difference reverses by 2-3 m/s on detector jitter, and the
                # loop would fly that noise instead of holding still.
                jump = max(abs(pos[i] - self.prev_pos[i]) for i in range(3))
                if jump <= 0.55:
                    v_new = [(pos[i] - self.prev_pos[i]) / dt for i in range(2)]
                    for i in range(2):
                        self.vel[i] = VEL_ALPHA * self.vel[i] + (1.0 - VEL_ALPHA) * v_new[i]
                    self.has_vel = True
                self.prev_pos = pos
                self.last_marker_t = now
        else:
            self.prev_pos = pos
            self.last_marker_t = now
            self.has_vel = False
            self.vel = [0.0, 0.0, 0.0]

        self.curr_state = pos
        self.alt = CAMERA_HEIGHT - self.z_filt
        yaw = math.degrees(float(marker.rotation.z))
        if math.isfinite(yaw):
            self.yaw = yaw
            self.yaw_hist.append((now, yaw))
            while (len(self.yaw_hist) > 2
                   and now - self.yaw_hist[0][0] > YAW_RATE_WINDOW):
                self.yaw_hist.pop(0)
            span = now - self.yaw_hist[0][0]
            if span >= YAW_RATE_MIN_SPAN:
                rate = wrap180(yaw - self.yaw_hist[0][1]) / span
                self.yaw_rate += YAW_RATE_ALPHA * (rate - self.yaw_rate)
        self.have_fix = True

        # Until the first goal arrives the drone simply holds its position.
        if not self.target_set:
            self.desired_state = list(pos)

    # ------------------------------------------------------------------
    # control
    # ------------------------------------------------------------------
    def pid(self):
        """The control loop, run on a fixed timer (self.sample_time)."""
        now = time.monotonic()

        if not self.have_fix:
            # no perception yet: stay on the ground, do not arm
            self.publish_command(armed=False)
            return

        if self.landed:
            # Mission complete: stay disarmed on the floor, never bounce back.
            self.disarm()
            self.publish_error()
            return

        if self.last_marker_t is None or (now - self.last_marker_t) > MARKER_TIMEOUT:
            # Marker lost.  Two stage watchdog: first level the tilt and keep
            # the throttle, then (after 1.5 s) fall back to the hover
            # throttle, so the drone never keeps a stale (and possibly large)
            # tilt command while the camera cannot see it.
            lost_t = now - (self.last_marker_t or now)
            self.cmd.rc_roll = int(RC_MID)
            self.cmd.rc_pitch = int(RC_MID)
            self.cmd.rc_yaw = int(RC_MID)
            if lost_t > 1.5:
                self.cmd.rc_throttle = int(HOVER_THROTTLE)
            self.command_pub.publish(self.cmd)
            self.publish_error()          # keep the error stream alive
            self.box_start = None
            self.hold_time = 0.0
            return

        self.update_errors()
        self.update_commands()
        self.publish_command(armed=True)
        self.publish_error()
        self.update_hold_state(now)
        self.log_status(now)

    def update_errors(self):
        """Error = setpoint minus current position, per axis.

        X and Y are used directly.  Z is the depth measured downwards from the
        camera, so the vertical error is expressed as an altitude error
        (positive = the drone is too low and has to climb).
        """
        self.error[0] = self.desired_state[0] - self.curr_state[0]
        self.error[1] = self.desired_state[1] - self.curr_state[1]
        self.error[2] = (CAMERA_HEIGHT - self.desired_state[2]) - self.alt

        # Take-off is over once the drone has climbed clear of the ground.
        # The flag stays set for the rest of the flight, so a later Save
        # descent to Z=14 at 1 m altitude uses the normal altitude loop.
        if not self.airborne and self.curr_state[2] < 14.2:
            self.airborne = True
            # Freeze the heading the drone actually took off with.  The
            # marker measures rotation relative to the arena, so this is
            # ~0 at spawn regardless of the randomised arena rotation.
            self.yaw_ref = self.yaw

    def update_commands(self):
        err_x, err_y, err_h = self.error

        # ---- takeoff (Task 2A phase A) ----------------------------------
        # While the drone is still high above the arena, climb vertically and
        # do not tilt.  A tilt command on/near the ground drags the airframe
        # over and can kill the marker stream; starting the XY transit during
        # the climb also makes the altitude loop overshoot its setpoint.
        if not self.airborne and not self.landing:
            self.rc[0] = RC_MID
            self.rc[1] = RC_MID
            self.rc[2] = 1548.0
            self.cmd.rc_yaw = int(RC_MID)
            return

        # ---- velocity targets ----
        v_des_x = max(-V_MAX_XY, min(V_MAX_XY, K_POS_XY * err_x))
        v_des_y = max(-V_MAX_XY, min(V_MAX_XY, K_POS_XY * err_y))
        v_des_h = max(-V_MAX_Z, min(V_MAX_Z, K_POS_Z * err_h))
        if self.landing:
            v_des_h = max(-LAND_V_MAX, v_des_h)

        # A Save needs a full metre of descent and the proportional law only
        # averages ~0.35 m/s over that distance.  Descend at the maximum rate
        # while the drone is still well above the Save box, then hand back to
        # the loop at SAVE_FAST_MARGIN above the box edge so it can settle
        # smoothly.  The hand-back point is deliberately far above the box,
        # and the filtered depth lags the true depth, so this cannot push the
        # Save overshoot out of the allowed band.
        if (not self.landing
                and abs(self.desired_state[2] - SAVE_Z) < 0.05
                and self.curr_state[2] < SAVE_Z - BOX - SAVE_FAST_MARGIN):
            v_des_h = -V_MAX_Z

        vx = self.vel[0]
        vy = self.vel[1]
        vh = -self.vel[2] if self.has_vel else 0.0      # climbing is positive

        # ---- rc commands ----
        out_x = K_RC_XY * (v_des_x - vx)
        out_y = K_RC_XY * (v_des_y - vy)
        out_h = K_RC_Z * (v_des_h - vh) + K_FF_Z * v_des_h

        # ---- integrators (anti windup: only close to the target and only
        #      while the command is not saturated) ----
        if abs(err_x) < INT_BAND_XY and abs(out_x) < MAX_DEV_XY:
            self.integral[0] = max(-INT_LIMIT_XY,
                                   min(INT_LIMIT_XY, self.integral[0] + err_x))
        if abs(err_y) < INT_BAND_XY and abs(out_y) < MAX_DEV_XY:
            self.integral[1] = max(-INT_LIMIT_XY,
                                   min(INT_LIMIT_XY, self.integral[1] + err_y))
        if abs(err_h) < INT_BAND_Z and abs(out_h) < MAX_DEV_Z:
            self.integral[2] = max(-INT_LIMIT_Z,
                                   min(INT_LIMIT_Z, self.integral[2] + err_h))

        out_x += K_INT_XY * self.integral[0]
        out_y += K_INT_XY * self.integral[1]
        out_h += K_INT_Z * self.integral[2]

        # ---- world-frame command -> body frame ----
        # The marker yaw is the airframe's heading relative to the arena, so
        # the body axes point at (cos yaw, sin yaw) and (-sin yaw, cos yaw) in
        # WhyCode coordinates.  Rotating the world-frame velocity command into
        # that frame keeps the drone flying at the waypoint even while the
        # airframe yaws; at yaw = 0 this is exactly the old fixed mapping.
        theta = math.radians(self.yaw)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        out_bx = out_x * cos_t + out_y * sin_t
        out_by = -out_x * sin_t + out_y * cos_t

        out_bx = max(-MAX_DEV_XY, min(MAX_DEV_XY, out_bx))
        out_by = max(-MAX_DEV_XY, min(MAX_DEV_XY, out_by))
        out_h = max(-MAX_DEV_Z, min(MAX_DEV_Z, out_h))

        # Tilting costs vertical thrust (T * cos(tilt)), so the throttle has to
        # grow by 1/cos(tilt) to keep the altitude.  The extra thrust is
        # converted to rc through the same N/rc slope the interface uses.
        tilt = TILT_RAD_PER_COUNT * max(abs(out_bx), abs(out_by))
        tilt_comp = (HOVER_THRUST / THRUST_PER_COUNT) * (1.0 / math.cos(tilt) - 1.0)

        # On the ground a tilt command drags the drone over and kills the
        # marker stream, so the XY authority is ramped in with the altitude.
        scale = max(LIFTOFF_SCALE, min(1.0, self.alt / LIFTOFF_ALT))

        self.rc[0] = RC_MID + scale * out_bx       # pitch -> body forward
        self.rc[1] = RC_MID + scale * out_by       # roll  -> body left
        self.rc[2] = HOVER_THROTTLE + out_h + tilt_comp

        # Final landing: the ground cushion can hold the airframe above the
        # touchdown depth, so close to the floor use a fixed low throttle and
        # let the floor stop the descent.
        if self.landing and self.alt < LAND_ENGAGE_ALT:
            self.rc[2] = LAND_THROTTLE

        # ---- yaw ----------------------------------------------------------
        # Hold the take-off heading.  The rate term is the main authority: the
        # airframe picks up yaw torque while it brakes out of a fast transit,
        # which previously spun the whole mission.  Heading is not scored, but
        # a spinning camera loses the marker, so the loop keeps it near the
        # reference heading; position control no longer depends on it.
        if self.yaw_ref is None:
            self.yaw_ref = self.yaw
        yaw_err = wrap180(self.yaw_ref - self.yaw)
        self.yaw_integral = max(-YAW_INT_LIMIT, min(YAW_INT_LIMIT,
                              self.yaw_integral + yaw_err * SAMPLE_TIME))
        yaw_out = (YAW_KP * yaw_err
                   + YAW_KI * self.yaw_integral
                   - YAW_KD * self.yaw_rate)
        yaw_out = max(-YAW_MAX, min(YAW_MAX, yaw_out))
        self.cmd.rc_yaw = int(round(RC_MID + yaw_out))

    def publish_command(self, armed):
        self.cmd.rc_pitch = int(round(max(RC_MIN, min(RC_MAX, self.rc[0]))))
        self.cmd.rc_roll = int(round(max(RC_MIN, min(RC_MAX, self.rc[1]))))
        self.cmd.rc_throttle = int(round(max(RC_MIN, min(RC_MAX, self.rc[2]))))
        self.cmd.rc_aux4 = 2000 if armed else 1000
        self.command_pub.publish(self.cmd)

    def publish_error(self):
        # Task convention: error = setpoint minus current position.
        #   pitch_error -> X, roll_error -> Y, throttle_error -> Z (depth).
        self.pos_error.pitch_error = float(self.error[0])
        self.pos_error.roll_error = float(self.error[1])
        self.pos_error.throttle_error = float(-self.error[2])
        self.pos_error.yaw_error = float(self.desired_yaw - self.yaw)
        self.pos_error_pub.publish(self.pos_error)

    def is_drone_in_box(self, drone_pos, box_centre, tolerance):
        """True when the drone is inside the waypoint box on every axis.

        Used for the navigation/feedback status only.  The graded Visit/Save
        holds use is_drone_in_sphere() below, which mirrors the portal test.
        """
        return (all(math.isfinite(value)
                    for value in (*drone_pos, *box_centre))
                and abs(drone_pos[0] - box_centre[0]) <= tolerance
                and abs(drone_pos[1] - box_centre[1]) <= tolerance
                and abs(drone_pos[2] - box_centre[2]) <= tolerance)

    def is_drone_in_sphere(self, drone_pos, centre, radius):
        """True when the raw marker is inside the grader's 3-D sphere.

        The portal checks sqrt(dx^2 + dy^2 + dz^2) <= 0.40 on the recorded
        /whycode_node/markers samples, so a per-axis box is not equivalent:
        a box corner can be 0.52 units from the centre and still pass the
        local box checker.  Counting the same sphere here means a completed
        action is a hold the portal can actually see.
        """
        if not all(math.isfinite(value)
                   for value in (*drone_pos, *centre)):
            return False
        dx = drone_pos[0] - centre[0]
        dy = drone_pos[1] - centre[1]
        dz = drone_pos[2] - centre[2]
        return dx * dx + dy * dy + dz * dz <= radius * radius

    def update_hold_state(self, now):
        """Arrival test: inside the box on every axis, continuously."""
        in_box = self.is_drone_in_box(self.curr_state, self.desired_state, BOX)
        if in_box:
            if self.box_start is None:
                self.box_start = now
            self.hold_time = now - self.box_start
        else:
            self.box_start = None
            self.hold_time = 0.0

    def log_status(self, now):
        if now - self.last_status < STATUS_PERIOD:
            return
        self.last_status = now
        self.get_logger().info(
            'pos=({0:+.2f},{1:+.2f},{2:+.2f}) err=({3:+.2f},{4:+.2f},{5:+.2f}) '
            'hold={6:.1f}s rc=({7},{8},{9},{10}) yaw={11:+.1f} '
            'yrate={12:+.1f} rej={13}'.format(
                self.curr_state[0], self.curr_state[1], self.curr_state[2],
                self.error[0], self.error[1], -self.error[2],
                self.hold_time,
                self.cmd.rc_pitch, self.cmd.rc_roll, self.cmd.rc_throttle,
                self.cmd.rc_yaw, self.yaw, self.yaw_rate, self.gate_rejects))

    # ------------------------------------------------------------------
    # action server
    # ------------------------------------------------------------------
    def cancel_callback(self, _goal_handle):
        """Accept the Switch cancel so the goal can stop and hold station."""
        self.get_logger().info('Cancel request accepted - holding position')
        return CancelResponse.ACCEPT

    def is_home_goal(self, position):
        """True when the goal is the fixed F6 take-off/landing point."""
        return (math.hypot(position[0] - self.home_xy[0],
                           position[1] - self.home_xy[1]) <= HOME_RADIUS)

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        request = goal_handle.request
        desired = [float(request.waypoint.position.x),
                   float(request.waypoint.position.y),
                   float(request.waypoint.position.z)]

        # A NaN or out-of-range setpoint would poison the control loop (and
        # int(round(nan)) would raise inside publish_command).  Abort it here.
        if (not all(math.isfinite(value) for value in desired)
                or not GOAL_Z_MIN <= desired[2] <= GOAL_Z_MAX):
            self.get_logger().error(
                'Rejecting invalid waypoint [{0}, {1}, {2}]'.format(*desired))
            try:
                goal_handle.abort()
            except Exception:                                   # pragma: no cover
                pass
            return NavToWaypoint.Result()

        hover = bool(request.hover)
        home = hover and self.is_home_goal(desired)
        self.desired_state = desired
        self.target_set = True
        self.landing = False
        self.box_start = None
        self.hold_time = 0.0
        self.get_logger().info(
            'New Waypoint Set: [{0:.2f}, {1:.2f}, {2:.2f}] '
            '(hover={3}{4})'.format(
                desired[0], desired[1], desired[2], hover,
                ', home' if home else ''))

        # hover=False -> fly through a grid intersection.
        # hover=True  -> Visit then Save at a survivor.
        # hover + F6  -> hover over home, then descend and land.
        phase = 'home_hover' if home else ('visit' if hover else 'pass')
        visit_target = [desired[0], desired[1], VISIT_Z]
        save_target = [desired[0], desired[1], SAVE_CTRL_Z]

        feedback = NavToWaypoint.Feedback()
        start = time.monotonic()
        phase_start = None
        pass_start = None
        outcome = 'shutdown'

        while rclpy.ok():
            feedback.current_waypoint.pose.position.x = self.curr_state[0]
            feedback.current_waypoint.pose.position.y = self.curr_state[1]
            feedback.current_waypoint.pose.position.z = self.curr_state[2]
            feedback.current_waypoint.header.stamp = self.get_clock().now().to_msg()
            try:
                goal_handle.publish_feedback(feedback)
            except Exception:                                   # pragma: no cover
                outcome = 'feedback failed'
                break

            now = time.monotonic()
            # A hold may only count on a fresh marker sample.  raw_state is
            # retained when the detector drops a frame; without this check the
            # server can finish a hold on a stale pose while the bag has a gap
            # the portal sees as a broken hold.
            fresh = (self.last_marker_t is not None
                     and now - self.last_marker_t <= HOLD_FRESH)

            if phase == 'pass':
                if self.is_drone_in_box(self.curr_state, desired, PASS_BOX):
                    if pass_start is None:
                        pass_start = now
                    elif now - pass_start >= PASS_DWELL:
                        outcome = 'succeeded'
                        break
                else:
                    pass_start = None

            elif phase == 'visit':
                if fresh and self.is_drone_in_sphere(
                        self.raw_state, visit_target, HOLD_RADIUS):
                    if phase_start is None:
                        phase_start = now
                    elif now - phase_start >= VISIT_HOLD:
                        phase = 'save'
                        self.desired_state = list(save_target)
                        self.box_start = None
                        self.hold_time = 0.0
                        phase_start = None
                        self.get_logger().info(
                            'Visit complete - descending for Save')
                else:
                    phase_start = None

            elif phase == 'save':
                if fresh and self.is_drone_in_sphere(
                        self.raw_state, save_target, HOLD_RADIUS):
                    if phase_start is None:
                        phase_start = now
                    elif now - phase_start >= SAVE_HOLD:
                        outcome = 'succeeded'
                        break
                else:
                    phase_start = None

            elif phase == 'home_hover':
                if fresh and self.is_drone_in_sphere(
                        self.raw_state, visit_target, HOLD_RADIUS):
                    if phase_start is None:
                        phase_start = now
                    elif now - phase_start >= HOME_HOLD:
                        phase = 'landing'
                        self.landing = True
                        self.desired_state = [desired[0], desired[1], LAND_Z]
                        self.box_start = None
                        self.hold_time = 0.0
                        phase_start = None
                        self.land_z_ref = None
                        self.land_z_t = now
                        self.get_logger().info(
                            'F6 hover complete - descending to land')
                else:
                    phase_start = None

            else:  # landing
                in_home_xy = (abs(self.curr_state[0] - desired[0]) <= BOX
                              and abs(self.curr_state[1] - desired[1]) <= BOX)
                if (self.land_z_ref is None
                        or self.curr_state[2]
                        > self.land_z_ref + TOUCHDOWN_STALL_STEP):
                    self.land_z_ref = self.curr_state[2]
                    self.land_z_t = now
                stalled = (self.curr_state[2] >= TOUCHDOWN_STALL_Z
                           and now - self.land_z_t >= TOUCHDOWN_STALL_TIME)
                if (in_home_xy
                        and (self.curr_state[2] >= TOUCHDOWN_Z or stalled)):
                    if phase_start is None:
                        phase_start = now
                    elif now - phase_start >= TOUCHDOWN_HOLD:
                        self.landed = True
                        self.landing = False
                        self.get_logger().info(
                            'Landed on F6 at ({0:+.2f},{1:+.2f}) z={2:.2f} - '
                            'disarmed'.format(self.curr_state[0],
                                              self.curr_state[1],
                                              self.curr_state[2]))
                        outcome = 'succeeded'
                        break
                else:
                    phase_start = None

            if goal_handle.is_cancel_requested:
                # The Switch happened mid-goal: stop here and hold station.
                self.desired_state = list(self.curr_state)
                self.landing = False
                self.box_start = None
                self.hold_time = 0.0
                try:
                    goal_handle.canceled()
                except Exception:                               # pragma: no cover
                    pass
                self.get_logger().info(
                    'Goal cancelled - holding position for the stable route')
                return NavToWaypoint.Result()

            # A goal that cannot be completed must not stall the mission.
            if now - start > GOAL_TIMEOUT:
                try:
                    goal_handle.abort()
                except Exception:                               # pragma: no cover
                    pass
                self.get_logger().warn(
                    'Waypoint [{0:.2f}, {1:.2f}, {2:.2f}] not completed within '
                    '{3:.0f} s - aborted'.format(
                        desired[0], desired[1], desired[2], GOAL_TIMEOUT))
                return NavToWaypoint.Result()

            time.sleep(0.04)

        if outcome != 'succeeded':
            self.get_logger().warn(
                'Waypoint [{0:.2f}, {1:.2f}, {2:.2f}] not reached ({3})'.format(
                    desired[0], desired[1], desired[2], outcome))
            return NavToWaypoint.Result()

        elapsed = time.monotonic() - start
        try:
            goal_handle.succeed()
        except Exception as exc:                                # pragma: no cover
            self.get_logger().warn(f'could not mark the goal succeeded: {exc}')
            return NavToWaypoint.Result()

        result = NavToWaypoint.Result()
        result.hov_time = int(round(elapsed))
        self.get_logger().info(
            'Waypoint [{0:.2f}, {1:.2f}, {2:.2f}] reached in {3:.1f} s '
            '({4})'.format(desired[0], desired[1], desired[2], elapsed,
                           'home landing' if home else
                           ('survivor stop' if hover else 'fly-through')))
        return result


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            waypoint_server.get_logger().info(
                'KeyboardInterrupt, shutting down.\n')
    finally:
        try:
            if waypoint_server.landed:
                waypoint_server.disarm()
            else:
                waypoint_server.publish_stable_hover()
        except Exception:                                       # pragma: no cover
            pass
        waypoint_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

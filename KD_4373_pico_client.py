#!/usr/bin/env python3
"""Task 3A - action client for the full rescue mission.

Mission flow (one continuous run, timed from take-off):

    1. ask the waypoint service ('waypoints') for the critical route, then
       wait for the action server ('waypoint_navigation');
    2. start the 55 s critical-window clock when the server is ready and fly
       the critical route goal by goal.  A plain marker id is a survivor stop;
       a trailing '*' is a fly-through intersection;
    3. at the Switch - the moment the third critical Save completes, or the
       55 s deadline, whichever comes first - stop the critical phase, wait
       for the server to stop, and send the drone's last feedback position to
       the stable-route service ('stable_waypoints').  A survivor Save that is
       already in progress at the deadline gets SAVE_GRACE seconds to finish,
       so a slow host still completes the Save and defines the Switch;
    4. fly the stable route it returns, goal by goal.  The last goal is F6:
       the server hovers there and lands;
    5. only then print the mission-complete line the recording script waits
       for.

Interface (waypoint_navigation/action/NavToWaypoint.action):

    geometry_msgs/Pose waypoint      # goal
    bool hover
    ---
    int32 hov_time                   # result
    ---
    geometry_msgs/PoseStamped current_waypoint   # feedback

Route encoding: 'D5' = stop (Visit + Save), 'D5*' = fly through, 'F6' = home
(hover at Z=13, then land).
"""

import collections
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints, PlanStableRoute


CRITICAL_WINDOW = 55.0          # s, from take-off to the critical deadline
SAVE_GRACE = 15.0               # s, an in-progress Save may finish after it
EXPECTED_CRITICAL = 3           # saves that end the critical phase early
SERVICE_RETRIES = 3             # attempts to obtain a non-empty route
SERVICE_TIMEOUT = 30.0          # s per service request
SERVICE_WAIT_TIMEOUT = 120.0    # s to wait for a service to be advertised
ACTION_SERVER_TIMEOUT = 90.0    # s to wait for waypoint_navigation
GOAL_RESPONSE_TIMEOUT = 10.0    # s for the server to accept a goal
CANCEL_RESPONSE_TIMEOUT = 10.0  # s to hear back about a cancel request
CANCEL_RESULT_TIMEOUT = 10.0    # s for the cancelled goal to stop
POLL_PERIOD = 0.05              # s between spin_once calls while flying
FEEDBACK_LOG_PERIOD = 0.5       # s between feedback log lines


Goal = collections.namedtuple('Goal', 'x y z hover marker_id')
Outcome = collections.namedtuple(
    'Outcome', 'succeeded status position marker_id cancelled')


class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')

        self.action_client = ActionClient(
            self, NavToWaypoint, 'waypoint_navigation')
        self.critical_cli = self.create_client(GetWaypoints, 'waypoints')
        self.stable_cli = self.create_client(
            PlanStableRoute, 'stable_waypoints')

        self.last_feedback = None
        self.last_feedback_log = 0.0
        self.mission_started = False
        self.mission_done = False

    # ------------------------------------------------------------------
    # services
    # ------------------------------------------------------------------
    def wait_for_service(self, client, name):
        self.get_logger().info(f'waiting for the {name} service...')
        deadline = time.monotonic() + SERVICE_WAIT_TIMEOUT
        while rclpy.ok():
            if client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service available')
                return True
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f'{name} service did not appear within '
                    f'{SERVICE_WAIT_TIMEOUT:.0f} s')
                return False
            self.get_logger().info('service not available, waiting again...')
        return False

    def call_service(self, client, request, label):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=SERVICE_TIMEOUT)
        if not future.done():
            self.get_logger().error(
                f'{label} request timed out after {SERVICE_TIMEOUT:.0f} s')
            future.cancel()
            return None
        try:
            return future.result()
        except Exception as exc:                                # pragma: no cover
            self.get_logger().error(f'{label} request failed: {exc}')
            return None

    def parse_route(self, response, label):
        """Turn a service response into Goal records, fail-safe on ids."""
        if response is None:
            return []
        poses = list(response.waypoints.poses)
        marker_ids = list(response.marker_ids)
        if not poses:
            self.get_logger().warn(f'{label} response contains no waypoints')
            return []
        if marker_ids and len(marker_ids) != len(poses):
            self.get_logger().warn(
                f'{label}: {len(marker_ids)} marker ids for {len(poses)} '
                'poses; missing ids are treated as hover stops')

        goals = []
        for index, pose in enumerate(poses):
            waypoint = (float(pose.position.x), float(pose.position.y),
                        float(pose.position.z))
            if not all(math.isfinite(value) for value in waypoint):
                self.get_logger().error(
                    f'{label} waypoint {index + 1} has a non-finite pose; '
                    'skipping it')
                continue
            marker_id = marker_ids[index] if index < len(marker_ids) else ''
            if not marker_ids:
                hover = True
            elif marker_id:
                hover = not marker_id.endswith('*')
            else:
                hover = True
            goals.append(Goal(*waypoint, hover, marker_id))
            self.get_logger().info(
                f'{label} waypoint: ({waypoint[0]:+.2f}, {waypoint[1]:+.2f}, '
                f'{waypoint[2]:+.2f}) marker={marker_id or "-"} '
                f'hover={hover}')
        return goals

    def get_critical_route(self):
        request = GetWaypoints.Request()
        request.get_waypoints = True
        for attempt in range(1, SERVICE_RETRIES + 1):
            response = self.call_service(
                self.critical_cli, request, 'GetWaypoints')
            goals = self.parse_route(response, 'critical')
            if goals:
                return goals
            self.get_logger().warn(
                f'critical route request {attempt}/{SERVICE_RETRIES}: '
                'no route returned')
            if attempt < SERVICE_RETRIES:
                time.sleep(1.0)
        return []

    def get_stable_route(self, position):
        request = PlanStableRoute.Request()
        request.start.position.x = float(position[0])
        request.start.position.y = float(position[1])
        request.start.position.z = float(position[2])
        for attempt in range(1, SERVICE_RETRIES + 1):
            response = self.call_service(
                self.stable_cli, request, 'PlanStableRoute')
            goals = self.parse_route(response, 'stable')
            if goals:
                return goals
            self.get_logger().warn(
                f'stable route request {attempt}/{SERVICE_RETRIES}: '
                'no route returned')
            if attempt < SERVICE_RETRIES:
                time.sleep(1.0)
        return []

    # ------------------------------------------------------------------
    # action client
    # ------------------------------------------------------------------
    def wait_for_action_server(self):
        self.get_logger().info('waiting for the action server...')
        deadline = time.monotonic() + ACTION_SERVER_TIMEOUT
        while rclpy.ok():
            if self.action_client.server_is_ready():
                self.get_logger().info('action server available')
                return True
            if time.monotonic() > deadline:
                self.get_logger().error(
                    'action server waypoint_navigation did not appear within '
                    f'{ACTION_SERVER_TIMEOUT:.0f} s')
                return False
            rclpy.spin_once(self, timeout_sec=0.5)
        return False

    def feedback_callback(self, feedback_msg):
        position = feedback_msg.feedback.current_waypoint.pose.position
        x = float(position.x)
        y = float(position.y)
        z = float(position.z)
        if all(math.isfinite(value) for value in (x, y, z)):
            self.last_feedback = (x, y, z)

        now = time.monotonic()
        if now - self.last_feedback_log < FEEDBACK_LOG_PERIOD:
            return
        self.last_feedback_log = now
        self.get_logger().info(
            f'Received feedback! The current whycon position is: '
            f'{x:.2f}, {y:.2f}, {z:.2f}')

    def send_goal(self, goal):
        goal_msg = NavToWaypoint.Goal()
        goal_msg.waypoint.position.x = goal.x
        goal_msg.waypoint.position.y = goal.y
        goal_msg.waypoint.position.z = goal.z
        goal_msg.hover = bool(goal.hover)

        self.get_logger().info(
            'Sending goal: ({0:+.2f}, {1:+.2f}, {2:+.2f}) marker={3} '
            'hover={4}'.format(goal.x, goal.y, goal.z,
                               goal.marker_id or '-', goal.hover))
        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=GOAL_RESPONSE_TIMEOUT)
        if not future.done():
            self.get_logger().error(
                f'goal response timed out after {GOAL_RESPONSE_TIMEOUT:.0f} s')
            return None
        try:
            handle = future.result()
        except Exception as exc:                                # pragma: no cover
            self.get_logger().error(f'goal request failed: {exc}')
            return None
        if handle is None or not handle.accepted:
            self.get_logger().error('goal was rejected by the action server')
            return None
        self.get_logger().info('Goal accepted, flying...')
        return handle

    def run_goal(self, goal, deadline=None):
        """Fly one goal; cancel at the deadline, except for an active Save.

        A survivor Save that is already in progress at the 55 s window is
        allowed SAVE_GRACE seconds to finish.  Cancelling it would lose both
        the Save and the Switch; finishing late still saves the survivor and
        lets the stable route run after the (late) Switch.
        """
        handle = self.send_goal(goal)
        if handle is None:
            return Outcome(False, 'rejected', self.last_feedback,
                           goal.marker_id, False)

        result_future = handle.get_result_async()
        cancelled = False
        cancel_deadline = None
        cancel_at = deadline
        grace_used = False

        while rclpy.ok() and not result_future.done():
            now = time.monotonic()

            if cancel_at is not None and not cancelled and now >= cancel_at:
                if goal.hover and not grace_used:
                    grace_used = True
                    cancel_at = now + SAVE_GRACE
                    self.get_logger().warn(
                        'Critical window expired - allowing the active Save '
                        f'({goal.marker_id or "no marker"}) to finish '
                        f'(grace {SAVE_GRACE:.0f} s)')
                else:
                    self.get_logger().warn(
                        'Critical window expired - cancelling the active goal '
                        f'({goal.marker_id or "no marker"})')
                    cancel_future = handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(
                        self, cancel_future,
                        timeout_sec=CANCEL_RESPONSE_TIMEOUT)
                    if not cancel_future.done():
                        self.get_logger().error(
                            'no response to the cancel request within '
                            f'{CANCEL_RESPONSE_TIMEOUT:.0f} s')
                    else:
                        try:
                            response = cancel_future.result()
                            accepted = bool(
                                response is not None
                                and len(response.goals_canceling) > 0)
                        except Exception as exc:                # pragma: no cover
                            accepted = False
                            self.get_logger().error(
                                f'cancel request failed: {exc}')
                        self.get_logger().info(
                            'cancel request ' + ('accepted' if accepted
                                                 else 'not accepted'))
                    cancelled = True
                    cancel_deadline = (
                        time.monotonic() + CANCEL_RESULT_TIMEOUT)

            elif (cancelled and cancel_deadline is not None
                  and now > cancel_deadline):
                self.get_logger().error(
                    'cancelled goal did not stop within '
                    f'{CANCEL_RESULT_TIMEOUT:.0f} s')
                return Outcome(False, 'cancel_timeout', self.last_feedback,
                               goal.marker_id, True)

            rclpy.spin_once(self, timeout_sec=POLL_PERIOD)

        if not rclpy.ok():
            return Outcome(False, 'shutdown', self.last_feedback,
                           goal.marker_id, cancelled)

        try:
            result = result_future.result()
            status = result.status
        except Exception as exc:                                # pragma: no cover
            self.get_logger().error(f'goal result failed: {exc}')
            return Outcome(False, 'error', self.last_feedback,
                           goal.marker_id, cancelled)

        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        label = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }.get(status, f'status {status}')
        self.get_logger().info(
            f'Result for {goal.marker_id or "waypoint"}: {label}')
        return Outcome(succeeded, label, self.last_feedback,
                       goal.marker_id, cancelled)

    # ------------------------------------------------------------------
    # mission
    # ------------------------------------------------------------------
    def run_mission(self):
        if not self.wait_for_service(self.critical_cli, 'waypoints'):
            return False
        if not self.wait_for_service(self.stable_cli, 'stable_waypoints'):
            return False

        critical = self.get_critical_route()
        if not critical:
            self.get_logger().error('no usable critical route')
            return False
        if not self.wait_for_action_server():
            return False

        # The action server begins the take-off as soon as it is up.  Starting
        # the clock here (never at client launch, which can be a minute early)
        # makes it no later than lift-off.
        start = time.monotonic()
        self.mission_started = True
        self.get_logger().info(
            f'Take-off clock started: critical window {CRITICAL_WINDOW:.0f} s')

        saves = 0
        switch_position = None
        for goal in critical:
            if time.monotonic() - start >= CRITICAL_WINDOW:
                self.get_logger().info(
                    'Critical window expired before the next goal')
                break
            outcome = self.run_goal(goal, deadline=start + CRITICAL_WINDOW)
            if outcome.position is not None:
                switch_position = outcome.position
            if goal.hover and outcome.succeeded:
                saves += 1
                self.get_logger().info(
                    f'Critical Save complete: {saves}/{EXPECTED_CRITICAL}')
            if outcome.cancelled:
                break
            if saves >= EXPECTED_CRITICAL:
                break

        switched_at = time.monotonic() - start
        self.get_logger().info(
            f'SWITCH at t={switched_at:.1f} s '
            f'(critical saves: {saves}/{EXPECTED_CRITICAL})')

        position = self.last_feedback or switch_position
        if position is None:
            self.get_logger().error(
                'no feedback position available at the Switch')
            return False

        stable = self.get_stable_route(position)
        if not stable:
            self.get_logger().error('no usable stable route')
            return False

        home_ok = False
        for goal in stable:
            outcome = self.run_goal(goal)
            if goal.marker_id == 'F6':
                home_ok = outcome.succeeded

        if not home_ok:
            self.get_logger().error(
                'mission ended without a confirmed landing on F6')
            return False

        self.mission_done = True
        total = time.monotonic() - start
        self.get_logger().info(
            f'Mission complete: landed on F6 at t={total:.1f} s')
        self.get_logger().info(
            'All waypoints have been reached successfully')
        return True


def main(args=None):
    rclpy.init(args=args)
    waypoint_client = WayPointClient()

    try:
        if not waypoint_client.run_mission():
            waypoint_client.get_logger().error(
                'the mission did not complete successfully')
    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            waypoint_client.get_logger().info(
                'KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

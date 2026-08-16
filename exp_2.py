"""
Docstring for exp_2

Command: python3 exp_2.py --kml demo_survey_area_new.kml --speed 5.0
"""

import argparse
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time
import threading
import math
import sys
import os
import cv2
import numpy as np
import threading
import time


class GridSurveyNode(Node):

    def __init__(self):
        super().__init__('grid_survey_node')
        self.get_logger().info('Grid Survey Node Started')

        # mission parameters
        self.altitude = 20.0               # meters
        self.spacing = 0.00005             # degrees approx (lat)
        self.speed = 5.0                   # m/s
        self.kml_file = None
        self.polygon_coords = []           # list of (lat, lon)

        # acceptance criteria
        self.accept_radius_m = 3.0         # meters to consider waypoint reached
        self.waypoint_timeout_s = 400      # seconds to wait for a waypoint

        # control flags
        self.stop_requested = False
        self.mission_thread = None

        # connect to vehicle
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to vehicle via MAVLink")

        # start keyboard watcher (q to stop)
        # made_change - keyboard watcher start disabled
        # self.kb_thread = threading.Thread(target=self._keyboard_watcher, daemon=True)
        # self.kb_thread.start()

        # start mission in background thread so rclpy.spin() isn't blocked
        self.mission_thread = threading.Thread(target=self.load_and_run_mission, daemon=True)
        # Thread will be started manually after configuration

        # ---------- Camera-based human detection (STRUCTURE ONLY) ----------
        # State machine variable (future use)
        self.flight_state = "MISSION"      # TODO: replace stop_requested with full state machine
        self.human_detected = False
        self._detector_running = True
        self._detector_thread = None
        self._human_lock = threading.Lock()
        # Delivery placeholders (set later)
        self.delivery_altitude = None      # TODO: set desired delivery altitude (m)
        self.original_altitude = None      # TODO: store altitude before delivery sequence
        self.delivery_start_time = None    # TODO: timestamp for hover/timeout logic

        # NOTE: start_camera_detector() is invoked to begin background camera detection.
        # If you want to keep running the old keyboard behaviour, comment out the next line.
        self.start_camera_detector()

    def start_mission(self):
        self.mission_thread.start()

    def set_kml_file(self, kml_path):
        self.kml_file = kml_path

    # ---------------------------
    # KML Parsing
    # ---------------------------
    def parse_kml_coordinates(self, kml_path):
        """
        Parses KML file to extract polygon coordinates.
        Supports standard KML (lon,lat) and heuristic check.
        Returns list of (lat, lon).
        """
        if not os.path.exists(kml_path):
            self.get_logger().error(f"KML file not found: {kml_path}")
            return []

        try:
            tree = ET.parse(kml_path)
            root = tree.getroot()
            # Namespace handling
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            # Find coordinates in Polygon/outerBoundaryIs/LinearRing/coordinates
            coords_text = None
            for coords_node in root.findall('.//kml:coordinates', ns):
                coords_text = coords_node.text
                break

            if not coords_text:
                # Try without namespace if failed
                for coords_node in root.findall('.//coordinates'):
                    coords_text = coords_node.text
                    break

            if not coords_text:
                self.get_logger().error("No coordinates found in KML.")
                return []

            points = []
            # Split by whitespace
            raw_points = coords_text.strip().split()
            for p in raw_points:
                # KML format: lon,lat,alt
                parts = p.split(',')
                if len(parts) >= 2:
                    p1 = float(parts[0])
                    p2 = float(parts[1])

                    # Heuristic: Latitude is always [-90, 90]. Longitude [-180, 180].
                    lat, lon = 0.0, 0.0

                    is_p1_lat_candidate = -90 <= p1 <= 90
                    is_p2_lat_candidate = -90 <= p2 <= 90

                    if is_p1_lat_candidate and not is_p2_lat_candidate:
                        # p1 is lat, p2 is lon (Non-standard "lat,lon")
                        lat, lon = p1, p2
                    elif not is_p1_lat_candidate and is_p2_lat_candidate:
                        # p1 is lon, p2 is lat (Standard "lon,lat")
                        lat, lon = p2, p1
                    else:
                        # Both could be lat? Assume Standard lon, lat
                        lat, lon = p2, p1

                    points.append((lat, lon))

            self.get_logger().info(f"Parsed {len(points)} points from KML.")
            return points

        except Exception as e:
            self.get_logger().error(f"Failed to parse KML: {e}")
            return []

    # ---------------------------
    # Ray Casting Algo
    # ---------------------------
    def is_point_in_polygon(self, lat, lon, polygon):
        """
        Ray-casting algorithm to find if (lat, lon) is inside polygon.
        Polygon is list of (lat, lon).
        """
        inside = False
        n = len(polygon)
        p1lat, p1lon = polygon[0]
        for i in range(1, n + 1):
            p2lat, p2lon = polygon[i % n]
            if self._target_lat_intersects_edge(lat, lon, p1lat, p1lon, p2lat, p2lon):
               inside = not inside
            p1lat, p1lon = p2lat, p2lon
        return inside

    def _target_lat_intersects_edge(self, lat, lon, p1lat, p1lon, p2lat, p2lon):
        # Check if ray from (lat, lon) to (lat, +inf) crosses edge (p1, p2)
        if min(p1lat, p2lat) < lat <= max(p1lat, p2lat):
            # find x intersection
            if p1lat != p2lat:
                x_inters = (lat - p1lat) * (p2lon - p1lon) / (p2lat - p1lat) + p1lon
            else:
                x_inters = p2lon
            if p1lon == p2lon:
                 x_inters = p1lon
            if lon < x_inters:
                return True
        return False

    # ---------------------------
    # Grid generation
    # ---------------------------
    def generate_grid_from_polygon(self, polygon, spacing):
        """
        Generates vertical lawn-mower pattern inside the polygon.
        """
        if not polygon:
            return []

        lats = [p[0] for p in polygon]
        lons = [p[1] for p in polygon]

        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)

        waypoints = []
        # Start from left (min_lon) to right
        curr_lon = min_lon
        reverse = False

        while curr_lon <= max_lon:
            col_points = []
            # Scan latitude from max to min
            curr_lat = max_lat
            while curr_lat >= min_lat:
                if self.is_point_in_polygon(curr_lat, curr_lon, polygon):
                    col_points.append((curr_lat, curr_lon))
                curr_lat -= spacing

            if col_points:
                # User Request: Only start and end points of the line
                if len(col_points) > 1:
                    col_points = [col_points[0], col_points[-1]]

                if reverse:
                    col_points.reverse()
                waypoints.extend(col_points)
                reverse = not reverse

            curr_lon += spacing

        return waypoints

    # ---------------------------
    # Utility: haversine distance (meters)
    # ---------------------------
    def haversine_m(self, lat1, lon1, lat2, lon2):
        # returns distance in meters between two lat/lon points
        R = 6371000.0  # Earth radius meters
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2.0)**2 + math.cos(phi1).math.cos(phi2) * math.sin(dlambda/2.0)**2 if False else math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    # ---------------------------
    # Read last known global position (non-blocking timeout)
    # ---------------------------
    def get_current_global_position(self, timeout=1.0):
        """
        Returns tuple (lat_deg, lon_deg, alt_m) or None on timeout.
        Uses GLOBAL_POSITION_INT messages (lat/lon in 1e7).
        """
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=True,
                timeout=0.5
            )
            if msg is None:
                continue
            try:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.relative_alt / 1000.0  # mm -> m
                return (lat, lon, alt)
            except Exception:
                continue
        return None

    # ---------------------------
    # Speed control
    # ---------------------------
    def set_speed(self, speed_m_s):
        """
        Sets the vehicle speed using WPNAV_SPEED parameter.
        WPNAV_SPEED is in cm/s.
        """
        try:
            # WPNAV_SPEED is in cm/s
            speed_cm_s = int(speed_m_s * 100)

            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'WPNAV_SPEED',
                speed_cm_s,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            self.get_logger().info(f"WPNAV_SPEED set to {speed_cm_s} cm/s ({speed_m_s} m/s)")
        except Exception as e:
            self.get_logger().error(f"Failed to set speed: {e}")

    # ---------------------------
    # Send a guided set-position command (global)
    # ---------------------------
    def send_guided_position(self, lat, lon, alt):
        """
        Robust send using SET_POSITION_TARGET_GLOBAL_INT that tries signed first,
        then unsigned (two's-complement) if packing complains.
        Works for pymavlink variants that pack lat/lon as 'i' or 'I'.
        """
        # prepare representations
        signed_lat = int(round(lat * 1e7))
        signed_lon = int(round(lon * 1e7))
        unsigned_lat = signed_lat & 0xFFFFFFFF
        unsigned_lon = signed_lon & 0xFFFFFFFF

        # time_boot_ms as uint32
        # Use 0 as we don't track boot time, ensuring it's ignored/handled correctly
        time_ms = 0

        # frame: prefer integer frame if available
        if hasattr(mavutil.mavlink, 'MAV_FRAME_GLOBAL_RELATIVE_ALT_INT'):
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        else:
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

        # position-only type_mask
        type_mask = 0b0000111111111000

        # helper to call the pymavlink send and wrap errors
        def _call_setpos(lat_val, lon_val):
            self.master.mav.set_position_target_global_int_send(
                time_ms,
                self.master.target_system,
                self.master.target_component,
                frame,
                type_mask,
                lat_val,
                lon_val,
                float(alt),
                0, 0, 0,            # vx, vy, vz
                0, 0, 0,            # afx, afy, afz
                float('nan'),       # yaw (ignored by mask)
                0                   # yaw_rate
            )

        # attempt signed first (natural)
        try:
            _call_setpos(signed_lat, signed_lon)
            # Log reduced to debug to avoid spamming if frequent, but here it's per waypoint
            self.get_logger().debug(
                f"Sent guided position (signed int) -> {lat:.6f},{lon:.6f},{alt:.1f}"
            )
            return
        except Exception as e_signed:
            s_err = str(e_signed)
            # if error indicates unsigned packing required, try unsigned
            if "'I' format" in s_err or "unsigned" in s_err or "requires 0 <=" in s_err:
                try:
                    _call_setpos(unsigned_lat, unsigned_lon)
                    self.get_logger().debug(
                        f"Sent guided position (unsigned int fallback) -> {lat:.6f},{lon:.6f},{alt:.1f}"
                    )
                    return
                except Exception as e_unsigned:
                    self.get_logger().error(f"Unsigned fallback failed: {e_unsigned}")
                    return

        # If we reached here it means signed attempt failed but didn't indicate unsigned; try unsigned anyway
        try:
            _call_setpos(unsigned_lat, unsigned_lon)
            return
        except Exception as e_final:
            self.get_logger().error(f"Final unsigned retry failed: {e_final}")
            return

    # ---------------------------
    # Mode & arm helpers
    # ---------------------------
    def set_mode(self, mode):
        # uses mavutil.set_mode helper
        try:
            self.master.set_mode(mode)
            self.get_logger().info(f"Set mode to {mode}")
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"Failed to set mode {mode}: {e}")

    def arm(self):
        try:
            self.master.arducopter_arm()
            self.get_logger().info("Arming command sent")
            # wait a bit; you can extend this to actually check ARMING state
            start = time.time()
            while time.time() - start < 8:
                state = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if state and getattr(state, 'base_mode', None) is not None:
                    # Could decode base_mode flags here if desired
                    break
                time.sleep(0.2)
        except Exception as e:
            self.get_logger().error(f"Failed to arm: {e}")

    def takeoff(self, target_alt_m):
        """
        Sends a NAV_TAKEOFF command in GUIDED to request takeoff.
        """
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0, 0, 0, 0, 0, 0,
                float(target_alt_m)
            )
            self.get_logger().info(f"Takeoff command sent to {target_alt_m} m")
        except Exception as e:
            self.get_logger().error(f"Failed to send takeoff: {e}")

    # ---------------------------
    # Keyboard watcher
    # ---------------------------
    def _keyboard_watcher(self):
        """
        Simple keyboard watcher that blocks on input() and sets stop_requested when 'q' is typed.
        Note: user must press Enter after typing 'q'.
        """
        self.get_logger().info("Keyboard watcher started: type 'q' + Enter to stop and hold position.")
        try:
            while True:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.1)
                    continue
                line = line.strip().lower()
                if line == 'q':
                    self.get_logger().warn(
                        "Stop requested via keyboard ('q'). Switching to LOITER and stopping mission."
                    )
                    self.stop_requested = True
                    # put vehicle into LOITER mode to hold position (LOITER works for ArduPilot)
                    try:
                        self.master.set_mode('LOITER')
                    except Exception as e:
                        self.get_logger().error(f"Failed to set LOITER: {e}")
                    break
        except Exception as e:
            self.get_logger().error(f"Keyboard watcher exception: {e}")

    # ---------------------------
    # Camera detector (currently structural)
    # ---------------------------
    def start_camera_detector(self):

        if self._detector_thread is not None and self._detector_thread.is_alive():
            self.get_logger().info("Camera detector already running.")
            return
        self._detector_running = True
        self._detector_thread = threading.Thread(
            target=self._camera_detector_loop,
            daemon=True
        )
        self._detector_thread.start()
        self.get_logger().info("Camera detector started.")

    def stop_camera_detector(self):
        """
        TODO:
        - Gracefully stop camera detector thread and release resources.
        - Call this from shutdown to close camera.
        """
        self._detector_running = False
        if self._detector_thread is not None:
            try:
                self._detector_thread.join(timeout=1.0)
            except Exception:
                pass
        self.get_logger().info("Camera detector stopped (TODO: ensure camera released).")

    def _camera_detector_loop(self):
        """
        Continuously runs in background:
        - reads camera frames
        - calls detect_human(frame)
        - triggers the detection handler (on_human_detected)
        """
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            self.get_logger().error("Camera not available. Detector disabled.")
            return

        while self._detector_running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            # ---- PLACEHOLDER: HUMAN DETECTION GOES HERE ----
            # detect_human must return (detected: bool, confidence: float)
            detected, confidence = self.detect_human(frame)
            # ------------------------------------------------

            if detected:
                with self._human_lock:
                    if not self.human_detected:
                        self.human_detected = True
                        self.get_logger().warn(
                            f"HUMAN DETECTED (conf={confidence:.2f})"
                        )
                        # Route detection through a single handler so logic is centralized.
                        # on_human_detected currently mirrors old behavior for compatibility,
                        # but you should modify it later to use flight_state state machine.
                        try:
                            self.on_human_detected(confidence)
                        except Exception as e:
                            self.get_logger().error(f"on_human_detected handler failed: {e}")

            time.sleep(0.05)

        try:
            cap.release()
        except Exception:
            pass
        self.get_logger().info("Camera detector loop terminated.")

    def detect_human(self, frame):
        """
        PLACEHOLDER for offline human detection.

        Replace later with:
        - MobileNet-SSD
        - YOLO
        - OpenCV HOG
        - custom model

        Must return:
            (detected: bool, confidence: float)

        TODOs:
          - Load model weights once (not per-frame)
          - Run inference on 'frame' and return detection result
          - Keep this function fast and avoid blocking long operations
          - Ensure it runs offline (no internet)
        """
        # TODO: implement offline human detection
        return False, 0.0

    # ---------------------------
    # Detection event handler (centralized)
    # ---------------------------
    def on_human_detected(self, confidence=None):
        """
        Central handler called once when a human is detected.

        CURRENT BEHAVIOR (backwards-compatible):
          - sets self.stop_requested = True (exact same effect as pressing 'q')

        FUTURE PLAN (TODO):
          - Instead of stop_requested, set self.flight_state = "HUMAN_DETECTED"
          - Do NOT execute long-running or flight commands here
          - Spawn threads or let mission loop handle motion/state transitions

        TODOs:
          - Replace stop_requested assignment with state transition when ready
          - Add logging of detection metadata (time, confidence, position)
          - Add debounce/cooldown logic to avoid repeated triggers
        """
        try:
            # Quick legacy behavior to preserve existing mission logic:
            # This makes the detection behave exactly like pressing 'q' today.
            self.get_logger().info("on_human_detected: setting stop_requested (legacy behaviour).")
            self.stop_requested = True

            # TODO: Replace legacy behaviour with state machine:
            # Example:
            # if self.flight_state == "MISSION":
            #     self.flight_state = "HUMAN_DETECTED"
            #     self.original_altitude = self.get_current_global_position(timeout=1.0)[2]  # optional
            #     self.delivery_start_time = time.time()

        except Exception as e:
            self.get_logger().error(f"Error in on_human_detected: {e}")

    # ---------------------------
    # Mission runner (background)
    # ---------------------------
    def load_and_run_mission(self):
        """
        Generate grid -> switch to GUIDED -> arm -> takeoff -> iterate waypoints -> send guided commands one-by-one
        """
        if self.kml_file:
            self.polygon_coords = self.parse_kml_coordinates(self.kml_file)

        if not self.polygon_coords:
             self.get_logger().error("No polygon coordinates available. Cannot generate grid.")
             return

        # generate grid
        grid = self.generate_grid_from_polygon(self.polygon_coords, self.spacing)
        if not grid:
            self.get_logger().error("No waypoints generated.")
            return

        self.get_logger().info(f"Generated {len(grid)} waypoints.")

        # switch to GUIDED
        self.set_mode("GUIDED")

        # arm
        self.arm()

        # Set speed
        self.set_speed(self.speed)

        # (Optional) read one position message before takeoff
        try:
            alt_msg = self.master.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=True,
                timeout=10
            )
            if alt_msg:
                init_alt = float(alt_msg.relative_alt) / 1000.0
                self.get_logger().info(f"Initial altitude before takeoff: {init_alt:.1f} m")
        except Exception as e:
            self.get_logger().warn(f"Failed to read initial altitude: {e}")

        # takeoff
        self.takeoff(self.altitude)

        # Wait until we are near the target altitude (within 1 m), up to some timeout
        self.get_logger().info("Waiting to reach takeoff altitude...")
        start = time.time()
        while time.time() - start < 60 and not self.stop_requested:
            pos = self.get_current_global_position(timeout=1.0)
            if pos is not None:
                _, _, cur_alt = pos
                self.get_logger().info(f"Current altitude: {cur_alt:.1f} m")
                if abs(cur_alt - self.altitude) <= 1.0:
                    self.get_logger().info("Reached takeoff altitude.")
                    break
            else:
                self.get_logger().debug("No position while waiting for takeoff altitude.")
            time.sleep(0.02)

        # iterate waypoints
        for idx, (lat, lon) in enumerate(grid):
            # NOTE: Existing mission logic checks stop_requested. In future you will replace this
            # with flight_state checks. See on_human_detected() TODOs.
            if self.stop_requested:
                self.get_logger().info("Stop requested; aborting waypoint loop.")
                return

            self.get_logger().info(
                f"Sending waypoint {idx+1}/{len(grid)} -> lat:{lat:.6f}, lon:{lon:.6f}"
            )

            # wait until inside accept radius or timeout
            start = time.time()
            reached = False
            while time.time() - start < self.waypoint_timeout_s and not self.stop_requested:
                # Resend command periodically (1Hz) to prevent timeout and handle packet loss
                self.send_guided_position(lat, lon, self.altitude)

                cur = self.get_current_global_position(timeout=1.0)
                if cur is None:
                    self.get_logger().debug("No position message received while waiting.")
                    continue

                cur_lat, cur_lon, cur_alt = cur
                d = self.haversine_m(cur_lat, cur_lon, lat, lon)
                self.get_logger().info(
                    f"Distance to WP: {d:.1f} m | current alt: {cur_alt:.1f} m"
                )

                if d <= self.accept_radius_m:
                    reached = True
                    self.get_logger().info(
                        f"Waypoint {idx+1} reached (d={d:.1f} m)."
                    )
                    break

                # We reuse the get_current_global_position timeout as partial sleep, 
                # but better to have explicit rate control if that returns fast.
                time.sleep(0.5)

            if not reached:
                self.get_logger().warn(
                    f"Timeout waiting for waypoint {idx+1}. Continuing to next waypoint."
                )

        self.get_logger().info("All waypoints processed (or mission ended).")

    # ---------------------------
    # Delivery pipeline placeholders (structure only)
    # ---------------------------
    def enter_delivery_mode(self):
        """
        TODO:
          - Pause mission progression (don't issue new waypoints)
          - Ensure vehicle is in GUIDED (or appropriate) mode
          - Record original altitude and position
        """
        pass

    def descend_for_delivery(self):
        """
        TODO:
          - Command controlled descent to self.delivery_altitude
          - Include safety checks: GPS health, minimum altitude, obstacles, etc.
        """
        pass

    def hold_for_drop(self):
        """
        TODO:
          - Stabilize and hover for self.delivery_hover_time
          - Verify position/accuracy before trigger
        """
        pass

    def trigger_drop_node(self):
        """
        TODO:
          - Notify the payload-release node or send appropriate MAVLink/servo command
          - This function should NOT block; it should publish/command and return
        """
        pass

    def ascend_after_drop(self):
        """
        TODO:
          - Command climb back to mission altitude or safe altitude
          - Update any state variables for resume
        """
        pass

    def resume_mission(self):
        """
        TODO:
          - Restore mission state and continue waypoints (if desired)
          - Or choose to RTL/LOITER based on mission policy
        """
        pass


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Grid Survey Node')
    parser.add_argument('--kml', type=str, default='demo_survey_area_new.kml', help='Path to KML file')
    parser.add_argument('--connect', type=str, default='udp:127.0.0.1:14551', help='MAVLink connection string')
    parser.add_argument('--speed', type=float, default=5.0, help='Drone speed in m/s')

    # We need to parse args but rclpy also uses args.
    # rclpy.init handles ROS args. We will use argparse for the rest.
    parsed, unknown = parser.parse_known_args()

    node = GridSurveyNode()

    # Configure node
    node.set_kml_file(parsed.kml)
    node.speed = parsed.speed

    node.get_logger().info(f"Using KML: {node.kml_file}")
    node.get_logger().info(f"Target Speed: {node.speed} m/s")

    # Start mission now that config is set
    node.start_mission()

    try:
        # keep node alive, mission is running in background thread
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected in main; exiting.")
    finally:
        # Ensure detector stopped on shutdown
        try:
            node.stop_camera_detector()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()


'''
Bugs / Notes (unchanged):

1. No waypoint control speed is set; vehicle may move too fast or too slow between waypoints.
2. No handling of vehicle disconnection or loss of MAVLink link.
3. No retries for failed MAVLink commands beyond basic error logging.
4. No logging of mission progress to file; all logs go to console only.
5. Altitude during takeoff is not monitored beyond a simple wait loop.
6. Time synchronization issues may arise if system time changes during mission.
7. No complex error recovery; mission aborts on first critical failure.
8. Heavy computation in main thread may block rclpy.spin() if not handled properly.

'''

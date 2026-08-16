"""
Docstring for res_2

Command: python3 res_2.py --kml demo_survey_area_new.kml --speed 5.0

Camera connection: Line

Pixhawk connection Line: 198, 1017 ('/dev/ttyACM0')

Alt: 201
Swath lenght: 202
Speed: 203

SAME_PERSON_RADIUS: 336
SERVE_RADIUS: 337

Sim to raspi:

Line: 209, 254, 1032

"""



import argparse
import xml.etree.ElementTree as ET
import logging
import random
from pymavlink import mavutil
import time
import threading
import math
import sys
import os
import numpy as np
import cv2
import csv
import subprocess
from geopy.distance import geodesic
from collections import deque
from ultralytics import YOLO
from datetime import datetime
import termios
import tty
import select

# ---------------------------------------------------------
# Geotagging Constants & Helpers
# ---------------------------------------------------------


    # ---------------------------
    # Global flag for pausing
    # ---------------------------


class ThreadedVideoCapture:
    """
    Spawns a dark thread that continually reads the latest frame
    from the video stream and discards the old ones.
    """
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src)
        self.ret, self.frame = self.cap.read()
        self.stopped = False
        self.lock = threading.Lock()
        
        # Start the background thread
        if self.cap.isOpened():
            self.thread = threading.Thread(target=self.update, args=())
            self.thread.daemon = True
            self.thread.start()

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if not ret:
                self.stopped = True
                break
            
            with self.lock:
                self.ret = ret
                self.frame = frame
            time.sleep(0.005) # slight yield

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.stopped = True
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        self.cap.release()






class GridSurveyNode:

    # def __int__(self, connection_string='/dev/ttyACM0'):
        
    def __init__(self, connection_string='udp:127.0.0.1:14551'):
        # super().__init__('grid_survey_node') 
        logging.info('Grid Survey Node Started')

        # mission parameters
        self.altitude = 20.0               # meters
        self.spacing = 0.00015             # degrees approx (lat)
        self.speed = 5.0                   # m/s
        self.kml_file = None
        self.polygon_coords = []           # list of (lat, lon)

        # Geotagging / Detection State
        self.latest_frame = None
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.gimbal_pitch = -45.0 # Fixed or read from MOUNT_STATUS
        
        # MAVLink State Cache
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None # relative
        self.current_hdg = 0.0  # True North Heading (deg)
        self.connected = False
        self.mutex = threading.Lock()
        
        # Detection Control
        self.detection_active = False
        self.rtsp_url = 0
        self.mqtt_started = False
        
        # Batching State
        self.batch_id = 1
        self.ids_in_current_batch = 0

        # acceptance criteria
        self.accept_radius_m = 7.0         # meters to consider waypoint reached
        self.waypoint_timeout_s = 400       # seconds to wait for a waypoint

        # control flags
        self.stop_requested = False
        self.mission_thread = None
        self.served_targets = [] # Initialize list of served targets

        # connect to vehicle
        # self.master = mavutil.mavlink_connection(connection_string, baud=115200, autoreconnect=True)
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        self.connected = True
        logging.info("Connected to vehicle via MAVLink")

        # Start MAVLink Listener Thread
        self.listener_thread = threading.Thread(target=self._mavlink_listener, daemon=True)
        self.listener_thread.start()



        # start mission in background thread
        self.mission_thread = threading.Thread(target=self.load_and_run_mission, daemon=True)
        
        # Detection thread (started later)
        self.detect_thread = threading.Thread(target=self.detection_loop, daemon=True)

        # Keyboard listener thread
        self.keyboard_thread = threading.Thread(target=self.monitor_keyboard, daemon=True)
        self.keyboard_thread.start()

    def _mavlink_listener(self):
        """
        Continuously reads MAVLink messages and updates state variables.
        This prevents race conditions on recv_match and ensures fresh data (especially ATTITUDE) is available.
        """
        while not self.stop_requested:
            try:
                # Read any message
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if not msg:
                    continue

                type_ = msg.get_type()

                if type_ == 'GLOBAL_POSITION_INT':
                    with self.mutex:
                        self.current_lat = msg.lat / 1e7
                        self.current_lon = msg.lon / 1e7
                        self.current_alt = msg.relative_alt / 1000.0
                        self.current_hdg = msg.hdg / 100.0 # cdeg to deg

                elif type_ == 'ATTITUDE':
                    with self.mutex:
                        self.imu_roll = math.degrees(msg.roll)
                        self.imu_pitch = math.degrees(msg.pitch)
                        self.imu_yaw = math.degrees(msg.yaw) # 0..360 usually

                elif type_ == 'HEARTBEAT':
                    # Could update mode here if needed
                    pass
                
                # Add MOUNT_STATUS handling here if using gimbal feedback

            except Exception as e:
                logging.debug(f"Listener error: {e}")
                time.sleep(0.1)

    def start_mission(self):
        self.mission_thread.start()

    def start_detection(self):
        if not self.detection_active:
            self.detection_active = True
            self.detect_thread.start()
            logging.info("Detection thread started.")

    def detection_loop(self):
        """
        Runs YOLO detection on video stream (Simplified Logic).
        """
        MODEL_PATH = r"E:\NIDAR\runs\detect\human_detection_y8n_stage22\weights\best.pt"
    
        SAVE_DIR = "detected_humans"
        os.makedirs(SAVE_DIR, exist_ok=True)
        
        PERSON_CLASS_ID = 0
        CONF_TH = 0.65

        logging.info("🚀 Human detection started (terminal-only mode)...")
        
        try:
            model = YOLO(MODEL_PATH)
            cap = cv2.VideoCapture(0)
            saved_track_ids = set()

            while not self.stop_requested and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    logging.info("📹 Video ended or camera stopped.")
                    break

                results = model.track(
                    frame,
                    conf=CONF_TH,
                    persist=True,
                    verbose=False
                )[0]

                if results.boxes is None:
                    continue

                for box in results.boxes:
                    if int(box.cls[0]) != PERSON_CLASS_ID:
                        continue

                    if box.id is None:
                        continue

                    track_id = int(box.id[0])

                    if track_id in saved_track_ids:
                        continue

                    conf = float(box.conf[0])
                    if conf < CONF_TH:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    save_frame = frame.copy()

                    cv2.rectangle(save_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(
                        save_frame,
                        f"Human ID {track_id} | conf={conf:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    img_path = os.path.join(
                        SAVE_DIR, f"human_{track_id}_{timestamp}.jpg"
                    )

                    cv2.imwrite(img_path, save_frame)
                    saved_track_ids.add(track_id)

                    logging.info(f"✅ SAVED | ID={track_id} | conf={conf:.2f} | file={img_path}")

            cap.release()
            logging.info("🛑 Detection stopped.")

        except Exception as e:
            logging.error(f"Detection loop error: {e}")

    def monitor_keyboard(self):
        """
        Monitors for 'SPACE' key press to trigger RTL.
        """
        logging.info("Keyboard monitor started. Press SPACE for RTL.")
        
        while not self.stop_requested:
            try:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    # Save terminal settings
                    old_settings = termios.tcgetattr(sys.stdin)
                    try:
                        tty.setcbreak(sys.stdin.fileno())
                        key = sys.stdin.read(1)
                        if key == ' ':
                            logging.warning("SPACE pressed! Triggering RTL...")
                            self.set_mode("RTL")
                            self.stop_requested = True
                    finally:
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass
            time.sleep(0.1)

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
            logging.error(f"KML file not found: {kml_path}")
            return []

        try:
            tree = ET.parse(kml_path)
            root = tree.getroot()
            # Namespace handling
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            # Find coordinates in Polygon/outerBoundaryIs/LinearRing/coordinates
            # This path works for the provided examples.
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
                logging.error("No coordinates found in KML.")
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
                    # Standard KML is lon, lat.
                    # If the first number is > 90 or < -90, it MUST be lon.
                    # If the second number is > 90 or < -90, it MUST be lon (and first is lat).
                    # If both are within range, we assume standard lon, lat unless we have reason not to.
                    
                    lat, lon = 0.0, 0.0
                    
                    if abs(p2) > 90:
                        # p2 is definitely lon? No, lat cant be > 90.
                        # Wait, if p2 > 90, it CANNOT be lat. So p2=lon, p1=lat.
                        # But wait, standard is lon,lat => p1=lon, p2=lat.
                        # If p2 > 90, then p2 is invalid latitude.
                        pass
                        
                    # Let's check ranges.
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
            
            logging.info(f"Parsed {len(points)} points from KML.")
            return points

        except Exception as e:
            logging.error(f"Failed to parse KML: {e}")
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
        # We use latitude as the Y-axis, Longitude as X-axis just for mental mapping, 
        # but standard algorithm:
        # Cross if p1lon > lon != p2lon > lon (one point to right) ... wait.
        # Ray casting: usually cast horizontal ray to right.
        # lat is Y, lon is X.
        # Check if point.y is between p1.y and p2.y
        
        if min(p1lat, p2lat) < lat <= max(p1lat, p2lat):
            # find x intersection
            if p1lat != p2lat:
                x_inters = (lat - p1lat) * (p2lon - p1lon) / (p2lat - p1lat) + p1lon
            else:
                x_inters = p2lon # Horizontal line?
                
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
        a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    # ---------------------------
    # Read last known global position (non-blocking timeout)
    # ---------------------------
    # ---------------------------
    # Read last known global position (from cache)
    # ---------------------------
    def get_current_global_position(self, timeout=1.0):
        """
        Returns tuple (lat_deg, lon_deg, alt_m) or None if no data.
        Uses cached data from listener.
        """
        start = time.time()
        while time.time() - start < timeout:
            with self.mutex:
                if self.current_lat is not None:
                    return (self.current_lat, self.current_lon, self.current_alt)
            time.sleep(0.05)
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
            logging.info(f"WPNAV_SPEED set to {speed_cm_s} cm/s ({speed_m_s} m/s)")
        except Exception as e:
            logging.error(f"Failed to set speed: {e}")

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
            logging.debug(
                f"Sent guided position (signed int) -> {lat:.6f},{lon:.6f},{alt:.1f}"
            )
            return
        except Exception as e_signed:
            s_err = str(e_signed)
            # if error indicates unsigned packing required, try unsigned
            if "'I' format" in s_err or "unsigned" in s_err or "requires 0 <=" in s_err:
                try:
                    _call_setpos(unsigned_lat, unsigned_lon)
                    logging.debug(
                        f"Sent guided position (unsigned int fallback) -> {lat:.6f},{lon:.6f},{alt:.1f}"
                    )
                    return
                except Exception as e_unsigned:
                    logging.error(f"Unsigned fallback failed: {e_unsigned}")
                    return

        # If we reached here it means signed attempt failed but didn't indicate unsigned; try unsigned anyway
        try:
            _call_setpos(unsigned_lat, unsigned_lon)
            return
        except Exception as e_final:
            logging.error(f"Final unsigned retry failed: {e_final}")
            return

    # ---------------------------
    # Mode & arm helpers
    # ---------------------------
    def set_mode(self, mode):
        # uses mavutil.set_mode helper
        try:
            self.master.set_mode(mode)
            logging.info(f"Set mode to {mode}")
            time.sleep(1.0)
        except Exception as e:
            logging.error(f"Failed to set mode {mode}: {e}")

    def arm(self):
        try:
            self.master.arducopter_arm()
            logging.info("Arming command sent")
            time.sleep(2.0) # Simple wait since listener consumes heartbeats
            # In a robust system, we would check self.current_mode from listener
        except Exception as e:
            logging.error(f"Failed to arm: {e}")

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
            logging.info(f"Takeoff command sent to {target_alt_m} m")
        except Exception as e:
            logging.error(f"Failed to send takeoff: {e}")


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
             logging.error("No polygon coordinates available. Cannot generate grid.")
             return

        # generate grid
        grid = self.generate_grid_from_polygon(self.polygon_coords, self.spacing)
        if not grid:
            logging.error("No waypoints generated.")
            return

        logging.info(f"Generated {len(grid)} waypoints.")


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
                timeout=5.0
            )
            if alt_msg:
                init_alt = float(alt_msg.relative_alt) / 1000.0
                logging.info(f"Initial altitude before takeoff: {init_alt:.1f} m")
        except Exception as e:
            logging.warning(f"Failed to read initial altitude: {e}")

        # takeoff
        self.takeoff(self.altitude)

        # Wait until we are near the target altitude (within 1 m), up to some timeout
        logging.info("Waiting to reach takeoff altitude...")
        start = time.time()
        while time.time() - start < 60 and not self.stop_requested:
            pos = self.get_current_global_position(timeout=1.0)
            if pos is not None:
                _, _, cur_alt = pos
                logging.info(f"Current altitude: {cur_alt:.1f} m")
                if abs(cur_alt - self.altitude) <= 2.0:
                    logging.info("Reached takeoff altitude.")
                    break
            else:
                logging.debug("No position while waiting for takeoff altitude.")
            time.sleep(0.02)

        # iterate waypoints
        for idx, (lat, lon) in enumerate(grid):
            if self.stop_requested:
                logging.info("Stop requested; aborting waypoint loop.")
                return

            logging.info(
                f"Sending waypoint {idx+1}/{len(grid)} -> lat:{lat:.6f}, lon:{lon:.6f}"
            )
            
            # START DETECTION AFTER REACHING 1st WAYPOINT (i.e. heading to 2nd)
            # OR start immediately? User said "When drone reaches first waypoint, start detection"
            # So if idx == 0 (first WP reached loop end), start it.
            # But here we are SENDING WP 1.
            # Let's start it after the first waypoint is REACHED.
            # See logic below 'reached = True'.

            # wait until inside accept radius or timeout
            start = time.time()
            reached = False
            while time.time() - start < self.waypoint_timeout_s and not self.stop_requested:
                


                if self.stop_requested:
                    break

                # Resend command periodically (1Hz) to prevent timeout and handle packet loss
                self.send_guided_position(lat, lon, self.altitude)
                
                cur = self.get_current_global_position(timeout=1.0)
                if cur is None:
                    logging.debug("No position message received while waiting.")
                    continue

                cur_lat, cur_lon, cur_alt = cur
                d = self.haversine_m(cur_lat, cur_lon, lat, lon)
                logging.info(
                    f"Distance to WP: {d:.1f} m | current alt: {cur_alt:.1f} m"
                )

                if d <= self.accept_radius_m:
                    reached = True
                    logging.info(
                        f"Waypoint {idx+1} reached (d={d:.1f} m)."
                    )
                    
                    # START DETECTION if this was the First Waypoint
                    if idx == 0:
                        logging.info("Reached 1st Waypoint. Starting Human Detection...")
                        self.start_detection()
                        
                    break
                    
                time.sleep(0.5)

            if not reached:
                logging.warning(
                    f"Timeout waiting for waypoint {idx+1}. Continuing to next waypoint."
                )

        logging.info("All waypoints processed (or mission ended).")
        
        # Mission Complete -> RTL
        if not self.stop_requested:
            logging.info("Mission Complete. Triggering RTL.")
            self.set_mode("RTL")


def main(args=None):
    # Configure logging
    log_filename = f"mission_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[
            logging.FileHandler(log_filename),
            logging.StreamHandler(sys.stdout)
        ]
    )
    logging.info(f"Logging to file: {log_filename}")

    parser = argparse.ArgumentParser(description='Grid Survey Node')
    parser.add_argument('--kml', type=str, default='demo_survey_area_new.kml', help='Path to KML file')
    # parser.add_argument('--connect', type=str, default='/dev/ttyACM0', help='MAVLink connection string')
    parser.add_argument('--connect', type=str, default='udp:127.0.0.1:14551', help='MAVLink connection string')
    parser.add_argument('--speed', type=float, default=5.0, help='Drone speed in m/s')
    
    parsed = parser.parse_args()

    node = GridSurveyNode(parsed.connect)
    
    # Configure node
    node.set_kml_file(parsed.kml)
    node.speed = parsed.speed
    
    # Update connection if needed (currently hardcoded in init)
    # Re-connect logic would go here if we updated __init__
    
    logging.info(f"Using KML: {node.kml_file}")
    logging.info(f"Target Speed: {node.speed} m/s")

    # Start mission
    node.start_mission()

    try:
        # Main loop to keep the script running
        while not node.stop_requested:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt detected in main; exiting.")
    finally:
        # Simple cleanup if needed
        pass


if __name__ == '__main__':
    main()





'''
Bugs:

*1. No waypoint control speed is set; vehicle may move too fast or too slow between waypoints.
*2. No handling of vehicle disconnection or loss of MAVLink link.
*3. No retries for failed MAVLink commands beyond basic error logging.
*4. No logging of mission progress to file; all logs go to console only.
*5. Altitude during takeoff is not monitored beyond a simple wait loop.
*6. Time synchronization issues may arise if system time changes during mission.
*7. No complex error recovery; mission aborts on first critical failure.

'''
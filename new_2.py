# Modified Grid Survey Node with build_waypoints_from_kml integration
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
import logging
from typing import List, Optional, Tuple

# Attempt to import shapely/pyproj for precise geospatial processing.
# If not available, code will fallback to the simpler grid generator that was
# present originally.
try:
    from shapely.geometry import Polygon, LineString, mapping
    from shapely.ops import transform
    from shapely.affinity import rotate, translate
    from pyproj import Transformer, CRS
    GEOS_AVAILABLE = True
except Exception as e:
    GEOS_AVAILABLE = False
    logging.getLogger().warning("shapely/pyproj not available: %s. Falling back to original grid generator.", e)


# ---------------------------
# Helper functions (geospatial)
# ---------------------------
def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2.0)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2.0)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


def dedupe_waypoints(wps: List[Tuple[float, float, float]], min_dist_m: float = 0.5) -> List[Tuple[float, float, float]]:
    """
    Remove near duplicates from a list of (lon, lat, alt) triplets.
    Uses haversine distance on lat/lon to compute separation.
    Keeps first occurrence in runs of near duplicates.
    """
    if not wps:
        return []
    out: List[Tuple[float, float, float]] = [wps[0]]
    for (lon, lat, alt) in wps[1:]:
        prev_lon, prev_lat, _ = out[-1]
        if haversine_m(prev_lat, prev_lon, lat, lon) >= min_dist_m:
            out.append((lon, lat, alt))
    return out


def pick_utm_crs_for_lonlat(lon: float, lat: float) -> CRS:
    """
    Returns a pyproj CRS object for an appropriate UTM zone for the given lon/lat.
    """
    zone = int((math.floor((lon + 180) / 6) % 60) + 1)
    is_northern = lat >= 0
    utm_crs_code = f"EPSG:{32600 + zone if is_northern else 32700 + zone}"
    return CRS.from_user_input(utm_crs_code)


def read_polygons_from_kml(kml_path: str) -> Polygon:
    """
    Read first polygon from KML. Returns a shapely Polygon in WGS84 (lon/lat).
    Uses simple XML parsing and the coordinate format lon,lat[,alt].
    """
    if not os.path.exists(kml_path):
        raise FileNotFoundError(f"KML not found: {kml_path}")

    tree = ET.parse(kml_path)
    root = tree.getroot()
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    coords_text = None
    # try namespaced
    for coords_node in root.findall('.//kml:coordinates', ns):
        if coords_node.text and coords_node.text.strip():
            coords_text = coords_node.text.strip()
            break
    # try without namespace
    if not coords_text:
        for coords_node in root.findall('.//coordinates'):
            if coords_node.text and coords_node.text.strip():
                coords_text = coords_node.text.strip()
                break

    if not coords_text:
        raise ValueError("No coordinates found in KML")

    raw_points = coords_text.strip().split()
    coords = []
    for p in raw_points:
        parts = p.split(',')
        if len(parts) >= 2:
            lon = float(parts[0])
            lat = float(parts[1])
            coords.append((lon, lat))
    if len(coords) < 3:
        raise ValueError("Insufficient polygon points in KML")
    # Make sure polygon is closed for shapely
    if coords[0] != coords[-1]:
        coords.append(coords[0])
    return Polygon(coords)


def project_geometry_to_utm(geom: Polygon, proj_to_utm) -> Polygon:
    """
    Projects shapely geometry using a transformer function to UTM (meters).
    proj_to_utm should be a callable(x, y) -> (x_utm, y_utm) with always_xy=True ordering (lon, lat).
    """
    return transform(lambda x, y, z=None: proj_to_utm(x, y), geom)


def generate_lawnmower(poly_utm: Polygon,
                       line_spacing_m: float,
                       point_spacing_m: float,
                       heading_deg: float,
                       inward_buffer: float = 0.0) -> List[LineString]:
    """
    Generate lawnmower segments inside the polygon (all in UTM coordinates).
    Approach:
      - Optionally buffer the polygon inward (negative buffer).
      - Rotate polygon so that lines align with X or Y axis (we rotate by -heading so the sweep axis becomes vertical).
      - Create parallel lines across bounding box spaced by line_spacing_m.
      - Intersect each line with polygon to get segments.
      - Rotate segments back to original orientation.
    Returns list of LineString segments (each in UTM).
    """
    if poly_utm.is_empty:
        return []

    # buffer inward (negative buffer) to avoid edges
    if inward_buffer and inward_buffer > 0:
        working = poly_utm.buffer(-inward_buffer)
        if working.is_empty:
            # buffering removed polygon; fallback to original
            working = poly_utm
    else:
        working = poly_utm

    # rotate polygon so that mowing lines are axis-aligned
    rotated = rotate(working, -heading_deg, origin='centroid', use_radians=False)

    minx, miny, maxx, maxy = rotated.bounds
    # create vertical lines (constant x) across bounding box
    segments = []
    x = minx
    # ensure numeric stability; guard against zero spacing
    if line_spacing_m <= 0:
        line_spacing_m = (maxx - minx) / 50.0 if (maxx - minx) > 0 else 1.0

    while x <= maxx + 1e-6:
        line = LineString([(x, miny - 10 * line_spacing_m), (x, maxy + 10 * line_spacing_m)])
        inter = rotated.intersection(line)
        # intersection can be MultiLineString or LineString or GeometryCollection with pieces
        if inter.is_empty:
            x += line_spacing_m
            continue
        # normalize to iterable of LineStrings
        if inter.geom_type == 'LineString':
            segs = [inter]
        else:
            # it's MultiLineString or collection -> extract lines
            segs = [g for g in inter.geoms if g.geom_type == 'LineString']
        for s in segs:
            # rotate segment back to original heading
            seg_back = rotate(s, heading_deg, origin='centroid', use_radians=False)
            segments.append(seg_back)
        x += line_spacing_m
    # Filter very short segments
    segments = [s for s in segments if s.length >= (point_spacing_m * 0.5)]
    return segments


def build_waypoints_from_kml(kml_path: str,
                             line_spacing: float,
                             point_spacing: float,
                             heading_deg: float,
                             altitude: float,
                             buffer_m: float,
                             max_waypoints: Optional[int] = None) -> List[Tuple[float, float, float]]:
    """
    Implements the algorithm you provided.
    Input line_spacing and point_spacing are in meters (UTM space).
    Returns list of (lon, lat, altitude).
    Requires shapely & pyproj. Raises if unavailable.
    """
    if not GEOS_AVAILABLE:
        raise RuntimeError("Shapely/pyproj required for build_waypoints_from_kml")

    geom = read_polygons_from_kml(kml_path)
    centroid = geom.centroid
    lon_c, lat_c = centroid.x, centroid.y
    utm_crs = pick_utm_crs_for_lonlat(lon_c, lat_c)
    proj_to_utm = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True).transform
    proj_to_wgs84 = Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True).transform

    poly_utm = project_geometry_to_utm(geom, proj_to_utm)

    segments = generate_lawnmower(poly_utm, line_spacing, point_spacing, heading_deg, inward_buffer=buffer_m)

    all_waypoints: List[Tuple[float, float, float]] = []
    reverse = False
    for seg in segments:
        coords = list(seg.coords)
        if len(coords) < 2:
            continue
        pts = [coords[0], coords[-1]]
        if reverse:
            pts = pts[::-1]
        for (x_m, y_m) in pts:
            lon, lat = proj_to_wgs84(x_m, y_m)
            all_waypoints.append((lon, lat, altitude))
            if max_waypoints and len(all_waypoints) >= max_waypoints:
                logging.warning("Reached max_waypoints cap (%d); truncating.", max_waypoints)
                return all_waypoints
        reverse = not reverse

    # remove near-duplicates (small epsilon)
    all_waypoints = dedupe_waypoints(all_waypoints, min_dist_m=0.5)

    if max_waypoints and len(all_waypoints) > max_waypoints:
        logging.warning("Final waypoints %d exceed max %d; truncating.", len(all_waypoints), max_waypoints)
        all_waypoints = all_waypoints[:max_waypoints]

    return all_waypoints


# ---------------------------
# Original Node (mostly unchanged)
# ---------------------------
class GridSurveyNode(Node):

    def __init__(self):
        super().__init__('grid_survey_node')
        self.get_logger().info('Grid Survey Node Started')

        # mission parameters
        self.altitude = 20.0               # meters
        self.spacing = 0.00025             # degrees approx (lat)
        self.speed = 5.0                   # m/s
        self.kml_file = None
        self.polygon_coords = []           # list of (lat, lon)


        # acceptance criteria
        self.accept_radius_m = 3.0         # meters to consider waypoint reached
        self.waypoint_timeout_s = 40       # seconds to wait for a waypoint

        # control flags
        self.stop_requested = False
        self.mission_thread = None

        # connect to vehicle
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to vehicle via MAVLink")

        # start keyboard watcher (q to stop)
        self.kb_thread = threading.Thread(target=self._keyboard_watcher, daemon=True)
        self.kb_thread.start()

        # start mission in background thread so rclpy.spin() isn't blocked
        self.mission_thread = threading.Thread(target=self.load_and_run_mission, daemon=True)
        # Thread will be started manually after configuration

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
                x_inters = p2lon # Horizontal line?

            if p1lon == p2lon:
                 x_inters = p1lon

            if lon < x_inters:
                return True
        return False

    # ---------------------------
    # Grid generation (original fallback)
    # ---------------------------
    def generate_grid_from_polygon(self, polygon, spacing):
        """
        Generates lawn-mower pattern inside the polygon. (Fallback simple method)
        """
        if not polygon:
            return []

        lats = [p[0] for p in polygon]
        lons = [p[1] for p in polygon]

        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)

        waypoints = []
        lat = max_lat # Start from top
        reverse = False

        # We need to cover the bounding box
        while lat >= min_lat:
            row_points = []
            curr_lon = min_lon
            while curr_lon <= max_lon:
                if self.is_point_in_polygon(lat, curr_lon, polygon):
                    row_points.append((lat, curr_lon))
                curr_lon += spacing # spacing in degrees

            if row_points:
                # User Request: Only start and end points of the line
                if len(row_points) > 1:
                    row_points = [row_points[0], row_points[-1]]

                if reverse:
                    row_points.reverse()
                waypoints.extend(row_points)
                reverse = not reverse

            lat -= spacing

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
        Sets the vehicle speed using MAV_CMD_DO_CHANGE_SPEED
        param1: Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
        param2: Speed (m/s)
        param3: Throttle (%, -1 indicates no change)
        param4: Relative (0: absolute, 1: relative)
        """
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1,              # Ground speed
                float(speed_m_s),
                -1,             # Throttle no change
                0,              # Absolute
                0, 0, 0
            )
            self.get_logger().info(f"Speed set command sent: {speed_m_s} m/s")
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

        # generate grid using new build_waypoints_from_kml if available, else fallback
        grid = []
        try:
            if GEOS_AVAILABLE:
                # convert self.spacing (degrees) roughly to meters (latitude meters)
                approx_m_per_deg = 111320.0
                line_spacing_m = max(1.0, self.spacing * approx_m_per_deg)
                point_spacing_m = max(1.0, line_spacing_m / 4.0)
                heading_deg = 0.0  # default; could be exposed via CLI
                buffer_m = 0.0     # default; could be exposed via CLI
                wp_triples = build_waypoints_from_kml(
                    self.kml_file,
                    line_spacing=line_spacing_m,
                    point_spacing=point_spacing_m,
                    heading_deg=heading_deg,
                    altitude=self.altitude,
                    buffer_m=buffer_m,
                    max_waypoints=None
                )
                # wp_triples are (lon, lat, alt). Convert to (lat, lon) pairs for existing mission logic
                grid = [(lat, lon) for (lon, lat, alt) in wp_triples]
            else:
                raise RuntimeError("Geo libs unavailable - using fallback.")
        except Exception as e_geo:
            self.get_logger().warn("Geospatial method failed (%s). Falling back to simple generator.", e_geo)
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
            time.sleep(0.5)

        # iterate waypoints
        for idx, (lat, lon) in enumerate(grid):
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

                time.sleep(0.5)

            if not reached:
                self.get_logger().warn(
                    f"Timeout waiting for waypoint {idx+1}. Continuing to next waypoint."
                )

        self.get_logger().info("All waypoints processed (or mission ended).")


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Grid Survey Node')
    parser.add_argument('--kml', type=str, default='demo_survey_area_new.kml', help='Path to KML file')
    parser.add_argument('--connect', type=str, default='udp:127.0.0.1:14551', help='MAVLink connection string')
    parser.add_argument('--speed', type=float, default=5.0, help='Drone speed in m/s')

    parsed, unknown = parser.parse_known_args()

    node = GridSurveyNode()

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
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

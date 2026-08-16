import time
import threading
from pymavlink import mavutil
import cv2
import csv
import os
import math
import numpy as np
from datetime import datetime
from ultralytics import YOLO

# ===================== PATHS =====================
MODEL_PATH = r"best.pt"
SAVE_DIR = "detections"
IMG_DIR = os.path.join(SAVE_DIR, "images")
CSV_PATH = os.path.join(SAVE_DIR, "final_human_coordinates.csv")

os.makedirs(IMG_DIR, exist_ok=True)
os.makedirs(SAVE_DIR, exist_ok=True)

# ===================== CAMERA CALIBRATION =====================
CAMERA_MATRIX = np.array([
    [1176.58, 0.0, 999.32],
    [0.0, 1176.61, 522.03],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

DIST_COEFFS = np.array([
    0.25010672,
   -0.53448585,
   -0.01314542,
    0.01928691,
    0.4317226
], dtype=np.float64)

fx = CAMERA_MATRIX[0, 0]
fy = CAMERA_MATRIX[1, 1]
cx0 = CAMERA_MATRIX[0, 2]
cy0 = CAMERA_MATRIX[1, 2]

# ===================== PARAMETERS =====================
PERSON_CLASS_ID = 0
CONF_TH = 0.60        # 🔥 STRICT CONFIDENCE THRESHOLD

BOTTOM_RATIO = 0.5
CENTER_LEFT_RATIO = 0.3
CENTER_RIGHT_RATIO = 0.7

CAM_PITCH = math.radians(35)
FRAME_CONFIRM = 3
DUPLICATE_RADIUS_M = 5
CELL_SIZE_M = 1.0

# ===================== DRONE STATE (LIVE MAVLINK) =====================
class DroneConnection:
    def __init__(self, connection_string='udp:127.0.0.1:14551', baud=115200):
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None  # Relative altitude
        self.imu_pitch = 0.0     # Pitch in radians (if needed)
        self.connected = False
        self.stop_requested = False
        self.lock = threading.Lock()
        
        print(f"Connecting to drone at {connection_string}...")
        try:
            self.master = mavutil.mavlink_connection(connection_string, baud=baud)
            self.master.wait_heartbeat()
            self.connected = True
            print("✅ Connected to Pixhawk via MAVLink")
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.connected = False

        # Start listener thread
        self.thread = threading.Thread(target=self._listener, daemon=True)
        self.thread.start()

    def _listener(self):
        while not self.stop_requested and self.connected:
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if not msg:
                    continue
                
                msg_type = msg.get_type()
                
                if msg_type == 'GLOBAL_POSITION_INT':
                    with self.lock:
                        self.current_lat = msg.lat / 1e7
                        self.current_lon = msg.lon / 1e7    
                        self.current_alt = msg.relative_alt / 1000.0  # mm to m
                
                elif msg_type == 'ATTITUDE':
                    with self.lock:
                        self.imu_pitch = msg.pitch  # radians
            
            except Exception as e:
                pass
                
    def get_location(self):
        with self.lock:
            return self.current_lat, self.current_lon, self.current_alt

    def close(self):
        self.stop_requested = True
        if self.connected:
            self.master.close()

# Initialize Drone Connection
# drone = DroneConnection(connection_string='/dev/ttyACM0')
drone = DroneConnection(connection_string='udp:127.0.0.1:14551')


# ===================== CSV INIT =====================
if not os.path.exists(CSV_PATH):
    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "human_id", "latitude", "longitude"])

# ===================== HELPERS =====================
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6378137.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) *
         math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

# ===================== MODEL & CAMERA =====================
model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(0)

# ===================== MEMORY =====================
humans = []                 # [{"id": int, "lat": float, "lon": float}]
saved_ids = set()           # IDs already written to CSV
candidate_counter = {}      # {(cell_x, cell_y): count}
next_human_id = 1

# ===================== MAIN LOOP =====================
try:
    while cap.isOpened():

        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS)
        h, w = frame.shape[:2]
        bottom_limit = int(BOTTOM_RATIO * h)

        results = model(frame, conf=CONF_TH, verbose=False)[0]
        seen_cells = set()

        if results.boxes is not None:
            for box in results.boxes:

                # ---------- CLASS FILTER ----------
                if int(box.cls[0]) != PERSON_CLASS_ID:
                    continue

                # ---------- CONFIDENCE FILTER (FINAL GUARD) ----------
                conf = float(box.conf[0])
                if conf < 0.65:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                u = (x1 + x2) / 2
                v = (y1 + y2) / 2

                # ---------- BOTTOM-FRAME FILTER ----------
                if v < bottom_limit:
                    continue

                # ---------- CLAMP HORIZONTAL OFFSET ----------
                cl = CENTER_LEFT_RATIO * w
                cr = CENTER_RIGHT_RATIO * w

                if cl < u < cr:
                    u_used = cx0
                elif u <= cl:
                    u_used = cl
                else:
                    u_used = cr

                v_used = bottom_limit

                # ---------- PIXEL → ANGLES ----------
                theta_x = math.atan((u_used - cx0) / fx)
                theta_y = math.atan((v_used - cy0) / fy)

                ground_angle = CAM_PITCH + theta_y
                if ground_angle <= 0:
                    continue

                # Get live drone state
                curr_lat, curr_lon, curr_alt = drone.get_location()

                # Fallback if no GPS yet (or wait)
                if curr_lat is None or curr_alt is None:
                    # print("Waiting for GPS...")
                    continue

                forward_dist = curr_alt / math.tan(ground_angle)
                lateral_dist = forward_dist * math.tan(theta_x)

                R = 6378137.0
                dlat = (forward_dist / R) * (180 / math.pi)
                dlon = (lateral_dist / (R * math.cos(math.radians(curr_lat)))) * (180 / math.pi)

                human_lat = curr_lat + dlat
                human_lon = curr_lon + dlon

                cell = (
                    int(forward_dist / CELL_SIZE_M),
                    int(lateral_dist / CELL_SIZE_M)
                )

                seen_cells.add(cell)
                candidate_counter[cell] = candidate_counter.get(cell, 0) + 1

                # ---------- 7-FRAME CONFIRMATION ----------
                if candidate_counter[cell] == FRAME_CONFIRM:

                    assigned_id = None
                    for hmn in humans:
                        if haversine_m(hmn["lat"], hmn["lon"], human_lat, human_lon) < DUPLICATE_RADIUS_M:
                            assigned_id = hmn["id"]
                            break

                    if assigned_id is None:
                        assigned_id = next_human_id
                        humans.append({
                            "id": assigned_id,
                            "lat": human_lat,
                            "lon": human_lon
                        })
                        next_human_id += 1

                    # ---------- SAVE ONLY ONCE PER ID ----------
                    if assigned_id not in saved_ids:
                        ts = datetime.now().strftime("%Y%m%d_%H%M%S")

                        img_path = os.path.join(IMG_DIR, f"human_{assigned_id}_{ts}.jpg")
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.imwrite(img_path, frame)

                        with open(CSV_PATH, "a", newline="") as f:
                            writer = csv.writer(f)
                            writer.writerow([
                                ts,
                                assigned_id,
                                round(human_lat, 7),
                                round(human_lon, 7)
                            ])

                        saved_ids.add(assigned_id)
                        print(f"✅ SAVED HUMAN ID {assigned_id} (conf={conf:.2f})")

        candidate_counter = {k: v for k, v in candidate_counter.items() if k in seen_cells}

        # Overlay GPS Status
        curr_lat, curr_lon, curr_alt = drone.get_location()
        gps_status = f"GPS: {curr_lat:.6f}, {curr_lon:.6f}, {curr_alt:.1f}m" if curr_lat else "GPS: WAITING..."
        color = (0, 255, 0) if curr_lat else (0, 0, 255)
        cv2.putText(frame, gps_status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        cv2.line(frame, (0, bottom_limit), (w, bottom_limit), (0, 0, 255), 2)
        cv2.imshow("Human Detection (CONF ≥ 0.65)", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

except KeyboardInterrupt:
    print("\n🛑 Interrupted by user.")
finally:
    cap.release()
    drone.close()
    cv2.destroyAllWindows()
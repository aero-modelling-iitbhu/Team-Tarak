import cv2
import os
import csv
import threading
from pymavlink import mavutil
from datetime import datetime
from ultralytics import YOLO

# ===================== PATHS =====================
MODEL_PATH = r"best.pt"
SAVE_DIR = "detected_humans"
IMG_DIR = os.path.join(SAVE_DIR, "images")
CSV_PATH = os.path.join(SAVE_DIR, "detections.csv")

os.makedirs(IMG_DIR, exist_ok=True)
os.makedirs(SAVE_DIR, exist_ok=True)

# ===================== PARAMETERS =====================
PERSON_CLASS_ID = 0
CONF_TH = 0.50

# ===================== DRONE GPS (LIVE MAVLINK) =====================
class DroneConnection:
    def __init__(self, connection_string='/dev/ttyACM0', baud=115200):
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
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

        self.thread = threading.Thread(target=self._listener, daemon=True)
        self.thread.start()

    def _listener(self):
        while not self.stop_requested and self.connected:
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if not msg:
                    continue
                
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    with self.lock:
                        self.current_lat = msg.lat / 1e7
                        self.current_lon = msg.lon / 1e7
                        self.current_alt = msg.relative_alt / 1000.0  # mm to m
            except Exception:
                pass
                
    def get_location(self):
        with self.lock:
            return self.current_lat, self.current_lon, self.current_alt

    def close(self):
        self.stop_requested = True
        if self.connected:
            self.master.close()

drone = DroneConnection()

# ===================== CSV INIT =====================
if not os.path.exists(CSV_PATH):
    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp",
            "human_id",
            "confidence",
            "drone_lat",
            "drone_lon",
            "drone_alt"
        ])

# ===================== MODEL & VIDEO =====================
model = YOLO(MODEL_PATH)
cap = cv2.VideoCapture(0)

# ===================== MEMORY =====================
saved_track_ids = set()
active_person_id = None   # currently handled person

print("🚀 Human detection started (event-based mode)...")

# ===================== MAIN LOOP =====================
while cap.isOpened():

    ret, frame = cap.read()
    if not ret:
        print("📹 Video ended.")
        break
    
    # Get live GPS
    lat, lon, alt = drone.get_location()
    
    # Overlay GPS Status
    gps_status = f"GPS: {lat:.6f}, {lon:.6f}, {alt:.1f}m" if lat else "GPS: WAITING..."
    color = (0, 255, 0) if lat else (0, 0, 255)
    cv2.putText(frame, gps_status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    results = model.track(
        frame,
        conf=CONF_TH,
        persist=True,
        verbose=False
    )[0]

    if results.boxes is None:
        continue

    for box in results.boxes:

        # -------- CLASS FILTER --------
        if int(box.cls[0]) != PERSON_CLASS_ID:
            continue

        if box.id is None:
            continue

        track_id = int(box.id[0])
        conf = float(box.conf[0])

        # Ignore already saved persons forever
        if track_id in saved_track_ids:
            continue

        # If already processing another person → WAIT
        if active_person_id is not None and track_id != active_person_id:
            continue

        if conf < CONF_TH:
            continue

        # -------- NEW PERSON DETECTED --------
        active_person_id = track_id

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        save_frame = frame.copy()

        cv2.rectangle(save_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)

        # Use live values if available, else fallback to defaults (or skip)
        current_lat = lat if lat else 0.0
        current_lon = lon if lon else 0.0
        current_alt = alt if alt else 0.0

        overlay = [
            f"Human ID: {track_id}",
            f"Conf: {conf:.2f}",
            f"Lat: {current_lat:.6f}",
            f"Lon: {current_lon:.6f}",
            f"Alt: {current_alt:.1f} m"
        ]

        y = y1 - 10
        for line in overlay:
            cv2.putText(
                save_frame,
                line,
                (x1, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )
            y -= 20

        # -------- SAVE IMAGE --------
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        img_path = os.path.join(
            IMG_DIR, f"human_{track_id}_{timestamp}.jpg"
        )
        cv2.imwrite(img_path, save_frame)

        # -------- SAVE CSV --------
        with open(CSV_PATH, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                track_id,
                round(conf, 3),
                round(current_lat, 7),
                round(current_lon, 7),
                current_alt
            ])

        saved_track_ids.add(track_id)
        active_person_id = None  # RESET → wait for new person

        print(
            f"✅ SAVED | ID={track_id} | conf={conf:.2f} | "
            f"GPS=({current_lat:.6f}, {current_lon:.6f}, {current_alt}m)"
        )

        break  # stop checking other boxes this frame

cap.release()
drone.close()
print("🛑 Detection stopped.")
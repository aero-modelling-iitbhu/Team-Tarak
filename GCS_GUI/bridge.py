import csv
import time
import os
from pymavlink import mavutil

# ================= CONFIGURATION =================
# Identify your USB ports. 
# Linux: '/dev/ttyUSB0', '/dev/ttyUSB1'
# Windows: 'COM3', 'COM4'
DRONE_CONFIGS = [
    {
        "id": "Drone 1",
        "port": "/dev/ttyUSB0", 
        "baud": 57600,
        "csv": "drone1_telemetry.csv"
    },
    {
        "id": "Drone 2",
        "port": "/dev/ttyUSB1",
        "baud": 57600,
        "csv": "drone2_telemetry.csv"
    }
]

def connect_drones():
    connections = []
    for config in DRONE_CONFIGS:
        print(f"Connecting to {config['id']} on {config['port']}...")
        try:
            conn = mavutil.mavlink_connection(config['port'], baud=config['baud'])
            # Wait for heartbeat to confirm physical link
            conn.wait_heartbeat(timeout=5)
            print(f"{config['id']} Connected!")
            connections.append({"conn": conn, "csv": config['csv'], "id": config['id']})
        except Exception as e:
            print(f" Could not connect to {config['id']}: {e}")
    return connections

def update_csv(filename, survivors, location, status):
    """Writes the data to the mailbox file for the GCS to read."""
    header = ["Survivors", "Location", "Status"]
    try:
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerow([survivors, location, status])
    except Exception as e:
        print(f"Error writing to {filename}: {e}")

def run_bridge():
    drones = connect_drones()
    
    if not drones:
        print("No drones connected. Exiting.")
        return

    print("\n--- Telemetry Bridge Active ---")
    
    while True:
        for drone in drones:
            # Try to receive a message from this specific drone
            # 'GLOBAL_POSITION_INT' gives GPS data
            # 'STATUSTEXT' is often used by onboard AI to send "Person Detected" alerts
            msg = drone['conn'].recv_match(type=['GLOBAL_POSITION_INT', 'STATUSTEXT'], blocking=False)
            
            if msg:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    current_loc = f"{lat:.5f}, {lon:.5f}"
                    
                    # Logic: In a real mission, you'd only update the CSV 
                    # when your AI script sends a "Detection" flag.
                    # For now, we update with a "Searching" or "Detected" status.
                    update_csv(drone['csv'], "1", current_loc, "Searching")
                
                elif msg.get_type() == 'STATUSTEXT':
                    # If the onboard AI sends "SURVIVOR FOUND" via MavLink text
                    if "FOUND" in msg.text.upper():
                        # We would grab the last known GPS and mark as Detected
                        print(f"🚨 ALERT from {drone['id']}: {msg.text}")
                        # (Note: You'd need to cache the GPS to update here properly)

        time.sleep(0.1) # Prevent CPU hogging

if __name__ == "__main__":
    run_bridge()
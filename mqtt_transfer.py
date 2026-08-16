import argparse
import time
import csv
import os
import sys
import paho.mqtt.client as mqtt

# Configuration
BROKER_ADDRESS = "broker.hivemq.com"
BROKER_PORT = 1883
TOPIC = "nidar/drone/survey/locations"

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT Broker!")
        if userdata == "receiver":
            client.subscribe(TOPIC)
            print(f"Subscribed to {TOPIC}")
    else:
        print(f"Failed to connect, reason code {reason_code}")


def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        print(f"Received: {payload}")
        
        # Append to local CSV (receiver always appends to one master file or we could verify batching too)
        # For now, let's keep singular receiver file for simplicity or user requirement.
        # User said "save next 5 ids in another csv" -> arguably receiver should also separate?
        # User request: "it should save next 5 ids in another csv [on sender side]. Once that csv gets 5 ids, it should again be transmitted"
        # Receiver behavior wasn't explicitly changed, but appending to one file is safe.
        
        CSV_FILE_RECEIVER = "received_locations.csv"
        file_exists = os.path.isfile(CSV_FILE_RECEIVER)
        with open(CSV_FILE_RECEIVER, 'a', newline='') as f:
            if not file_exists:
                f.write("person_id,latitude,longitude,timestamp\n")
            f.write(payload + "\n")
            
    except Exception as e:
        print(f"Error processing message: {e}")

def run_sender(filename):
    if hasattr(mqtt, 'CallbackAPIVersion'):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    else:
        client = mqtt.Client()
    client.on_connect = on_connect
    client.user_data_set("sender")
    
    print(f"Connecting to {BROKER_ADDRESS} as Sender...")
    client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
    client.loop_start()
    
    # Wait for connection
    time.sleep(1)
    
    if not os.path.exists(filename):
        print(f"File {filename} does not exist.")
        return

    print(f"Sending file: {filename}")
    
    with open(filename, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line.strip()
            if line:
                print(f"Publishing: {line}")
                info = client.publish(TOPIC, line)
                info.wait_for_publish() # Ensure sent
                time.sleep(0.1)
                
    print("File sent. Disconnecting.")
    client.loop_stop()
    client.disconnect()

def run_receiver():
    if hasattr(mqtt, 'CallbackAPIVersion'):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    else:
        client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.user_data_set("receiver")
    
    print(f"Connecting to {BROKER_ADDRESS} as Receiver...")
    client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
    
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("Disconnecting...")
        client.disconnect()

def main():
    parser = argparse.ArgumentParser(description='MQTT File Transfer for Drone Survey')
    parser.add_argument('--mode', choices=['sender', 'receiver'], required=True, help='Mode: sender (Drone 1) or receiver (Drone 2)')
    parser.add_argument('--file', type=str, help='Path to CSV file to send (sender mode only)')
    
    args = parser.parse_args()
    
    if args.mode == 'sender':
        if not args.file:
            print("Sender mode requires --file argument")
            return
        run_sender(args.file)
    else:
        run_receiver()

if __name__ == "__main__":
    main()

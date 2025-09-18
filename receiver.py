#!/usr/bin/python3
# -- coding: UTF-8 --

import serial
import pynmea2
import time
from datetime import datetime

# ===============================
# CONFIGURATION
# ===============================
UART_PORT = "/dev/ttyAMA0"   # GPS UART port (adjust if using USB, e.g., "/dev/ttyUSB0")
BAUDRATE = 9600              # Default GPS baud rate

# ===============================
# INITIALIZE SERIAL
# ===============================
try:
    gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=1)
    print(f"[INFO] Connected to GPS module on {UART_PORT} at {BAUDRATE} baud.")
except serial.SerialException as e:
    print(f"[ERROR] Could not open serial port {UART_PORT}: {e}")
    exit(1)

# ===============================
# HELPER FUNCTION
# ===============================
def parse_gps_data(line):
    """Parse NMEA sentence and extract useful data safely"""
    try:
        msg = pynmea2.parse(line)
        
        if isinstance(msg, pynmea2.types.talker.GGA):  # GGA sentence = fix data
            if msg.latitude and msg.longitude:
                return {
                    "type": "GGA",
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "altitude": msg.altitude,
                    "timestamp": msg.timestamp
                }
            else:
                return None

        elif isinstance(msg, pynmea2.types.talker.RMC):  # RMC sentence = speed and time
            if msg.status == "A":  # A = Active fix
                return {
                    "type": "RMC",
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "speed_knots": msg.spd_over_grnd,
                    "timestamp": msg.datestamp
                }
            else:
                return None
        else:
            return None
    except pynmea2.ParseError:
        return None
    except Exception as e:
        print(f"[WARN] Unexpected parsing error: {e}")
        return None

# ===============================
# MAIN LOOP
# ===============================
print("[INFO] Reading GPS data... Press CTRL+C to stop.\n")
try:
    while True:
        line = gps_serial.readline().decode('ascii', errors='replace').strip()
        
        if not line.startswith('$'):  # Ignore garbage data
            continue

        gps_data = parse_gps_data(line)
        if gps_data:
            if gps_data["type"] == "GGA":
                print(f"[GGA] Time: {gps_data['timestamp']} | "
                      f"Lat: {gps_data['latitude']:.6f} | "
                      f"Lon: {gps_data['longitude']:.6f} | "
                      f"Altitude: {gps_data['altitude']} m")
            
            elif gps_data["type"] == "RMC":
                speed_kmh = gps_data["speed_knots"] * 1.852 if gps_data["speed_knots"] else 0
                print(f"[RMC] Date: {gps_data['timestamp']} | "
                      f"Lat: {gps_data['latitude']:.6f} | "
                      f"Lon: {gps_data['longitude']:.6f} | "
                      f"Speed: {speed_kmh:.2f} km/h")

        time.sleep(0.1)  # Small delay to prevent CPU overuse

except KeyboardInterrupt:
    print("\n[INFO] GPS reading stopped by user.")
finally:
    gps_serial.close()
    print("[INFO] Serial port closed.")

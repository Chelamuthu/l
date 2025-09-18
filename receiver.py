#!/usr/bin/python3
# -- coding: UTF-8 --

import serial
import pynmea2
import time
import math

# ===============================
# CONFIGURATION
# ===============================
UART_PORT = "/dev/ttyAMA0"  # GPS UART port on Raspberry Pi
BAUDRATE = 9600             # GPS default baud rate

# ===============================
# Helper Functions
# ===============================

def haversine(lat1, lon1, lat2, lon2):
    """Calculate distance between two GPS coordinates in meters"""
    R = 6371000  # Radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # Distance in meters


# ===============================
# Main Program
# ===============================

def read_gnss():
    try:
        # Open serial port
        gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=1)
        print("Connected to GNSS module on", UART_PORT)

        last_lat, last_lon = None, None
        total_distance = 0.0

        while True:
            line = gps_serial.readline().decode('ascii', errors='replace').strip()
            
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):  # Position fix data
                msg = pynmea2.parse(line)
                
                latitude = msg.latitude
                longitude = msg.longitude
                altitude = msg.altitude

                print(f"[GGA] Lat: {latitude:.6f}, Lon: {longitude:.6f}, Alt: {altitude} m")

                # Calculate distance traveled
                if last_lat is not None and last_lon is not None:
                    distance = haversine(last_lat, last_lon, latitude, longitude)
                    total_distance += distance
                    print(f"Distance moved: {distance:.2f} m | Total: {total_distance:.2f} m")

                last_lat, last_lon = latitude, longitude

            elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):  # Recommended minimum data
                msg = pynmea2.parse(line)
                
                speed_knots = float(msg.spd_over_grnd) if msg.spd_over_grnd else 0.0
                speed_kmh = speed_knots * 1.852  # Convert knots to km/h
                timestamp = msg.timestamp
                status = msg.status  # 'A' = Active, 'V' = Void

                print(f"[RMC] Time: {timestamp} | Speed: {speed_kmh:.2f} km/h | Status: {status}")

            time.sleep(0.1)  # Small delay to reduce CPU usage

    except serial.SerialException:
        print("Error: Could not open GPS serial port.")
    except KeyboardInterrupt:
        print("\nExiting GPS reader...")
    except Exception as e:
        print("Unexpected error:", str(e))


if __name__ == "__main__":
    read_gnss()

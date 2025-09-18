#!/usr/bin/python3
import serial
import pynmea2
import time
import math

# UART configuration
UART_PORT = "/dev/ttyAMA0"  # GPS connected to Pi's main UART
BAUDRATE = 9600             # Neo-6M default baud rate

# Haversine formula for distance calculation
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

def read_gps():
    try:
        gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=1)
        print("Connected to Neo-6M GPS on", UART_PORT)

        last_lat, last_lon = None, None
        total_distance = 0.0

        while True:
            line = gps_serial.readline().decode('ascii', errors='replace').strip()

            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):  # Position fix
                msg = pynmea2.parse(line)
                latitude, longitude, altitude = msg.latitude, msg.longitude, msg.altitude

                print(f"[GGA] Lat: {latitude:.6f}, Lon: {longitude:.6f}, Alt: {altitude:.2f} m")

                # Calculate distance traveled
                if last_lat is not None and last_lon is not None:
                    distance = haversine(last_lat, last_lon, latitude, longitude)
                    total_distance += distance
                    print(f"Moved: {distance:.2f} m | Total: {total_distance:.2f} m")

                last_lat, last_lon = latitude, longitude

            elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):  # Speed + time
                msg = pynmea2.parse(line)
                speed_knots = float(msg.spd_over_grnd) if msg.spd_over_grnd else 0.0
                speed_kmh = speed_knots * 1.852  # Convert knots to km/h

                print(f"[RMC] Time: {msg.timestamp} | Speed: {speed_kmh:.2f} km/h | Status: {msg.status}")

            time.sleep(0.1)

    except serial.SerialException:
        print("Error: Could not open GPS serial port.")
    except KeyboardInterrupt:
        print("\nExiting GPS reader...")
    except Exception as e:
        print("Unexpected error:", str(e))

if __name__ == "__main__":
    read_gps()

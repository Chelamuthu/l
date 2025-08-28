import os, sys
import time
import serial
from LoRaRF import SX126x

# --- LoRa Setup ---
busId = 0; csId = 0 
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1 
LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Something wrong, can't begin LoRa radio")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 64, True)
LoRa.setSyncWord(0x3444)

print("\n-- LoRa Transmitter with GNSS + Speed + Time --\n")

# --- GNSS Setup ---
GNSS_PORT = "/dev/ttyAMA0"   # Linux
# GNSS_PORT = "COM5"         # Windows
GNSS_BAUD = 9600

try:
    gps_serial = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
    print(f"Opened GNSS on {GNSS_PORT}")
except Exception as e:
    print("Error opening GNSS:", e)
    gps_serial = None

def convert_lat_lon(lat_str, ns, lon_str, ew):
    """Convert NMEA lat/lon format to decimal degrees"""
    if not lat_str or not lon_str:
        return None, None
    # Latitude
    lat_deg = float(lat_str[:2])
    lat_min = float(lat_str[2:])
    lat = lat_deg + (lat_min / 60.0)
    if ns == "S":
        lat = -lat
    # Longitude
    lon_deg = float(lon_str[:3])
    lon_min = float(lon_str[3:])
    lon = lon_deg + (lon_min / 60.0)
    if ew == "W":
        lon = -lon
    return lat, lon

counter = 0
while True:
    latitude, longitude, speed_kmh, utc_time = None, None, None, None

    if gps_serial and gps_serial.in_waiting > 0:
        line = gps_serial.readline().decode("utf-8", errors="ignore").strip()

        # RMC sentence has time, lat, lon, speed
        if line.startswith("$GPRMC"):
            parts = line.split(",")
            try:
                utc_time = parts[1]              # hhmmss.sss
                lat_str, ns = parts[3], parts[4]
                lon_str, ew = parts[5], parts[6]
                latitude, longitude = convert_lat_lon(lat_str, ns, lon_str, ew)

                speed_knots = parts[7]
                if speed_knots:
                    speed_kmh = float(speed_knots) * 1.852
            except Exception as e:
                print("Parse error (RMC):", e)

        # GGA sentence has time, lat, lon
        elif line.startswith("$GPGGA"):
            parts = line.split(",")
            try:
                utc_time = parts[1]
                lat_str, ns = parts[2], parts[3]
                lon_str, ew = parts[4], parts[5]
                latitude, longitude = convert_lat_lon(lat_str, ns, lon_str, ew)
            except Exception as e:
                print("Parse error (GGA):", e)

    # Build message
    if latitude and longitude:
        spd_text = f"{speed_kmh:.2f}km/h" if speed_kmh is not None else "N/A"
        time_text = utc_time if utc_time else "N/A"
        message = f"GPS:{latitude:.5f},{longitude:.5f} Spd:{spd_text} Time:{time_text} Cnt:{counter}"
    else:
        message = f"No GPS Fix Cnt:{counter}"

    # Convert to bytes
    message_bytes = [ord(c) for c in message]

    # Send via LoRa
    LoRa.beginPacket()
    LoRa.write(message_bytes, len(message_bytes))
    LoRa.endPacket()
    LoRa.wait()

    print(f"Sent: {message}")
    print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s"
          .format(LoRa.transmitTime(), LoRa.dataRate()))

    time.sleep(5)
    counter = (counter + 1) % 256

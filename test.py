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
LoRa.setFrequency(865000000)   # India freq (865–867 MHz)
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 64, True)
LoRa.setSyncWord(0x3444)

print("\n-- LoRa Transmitter with GNSS + Speed + Time --\n")

# --- GNSS Setup ---
GNSS_PORT = "/dev/ttyAMA0"   # On Raspberry Pi, GNSS UART
# GNSS_PORT = "COM5"         # Windows
GNSS_BAUD = 9600

try:
    gps_serial = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
    print(f"Opened GNSS on {GNSS_PORT}")
except Exception as e:
    print("Error opening GNSS:", e)
    gps_serial = None

def convert_lat_lon(lat_str, ns, lon_str, ew):
    """Convert NMEA lat/lon format to decimal degrees, return None if invalid"""
    if not lat_str or not lon_str:
        return None, None
    try:
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
    except:
        return None, None

def format_utc(utc_str):
    """Convert hhmmss.sss to hh:mm:ss UTC"""
    if not utc_str or len(utc_str) < 6:
        return "N/A"
    try:
        hh = utc_str[0:2]
        mm = utc_str[2:4]
        ss = utc_str[4:6]
        return f"{hh}:{mm}:{ss} UTC"
    except:
        return "N/A"

counter = 0
while True:
    latitude, longitude, speed_kmh, utc_time = None, None, None, None

    try:
        if gps_serial and gps_serial.in_waiting > 0:
            line = gps_serial.readline().decode("utf-8", errors="ignore").strip()

            # Debug: print NMEA raw line
            if line.startswith("$"):
                print("NMEA:", line)

            # --- RMC sentence (time, lat, lon, speed, fix status) ---
            if line.startswith("$GPRMC"):
                parts = line.split(",")
                if len(parts) > 7 and parts[2] == "A":   # 'A' means valid fix
                    utc_time = format_utc(parts[1])
                    latitude, longitude = convert_lat_lon(parts[3], parts[4], parts[5], parts[6])
                    if parts[7]:
                        try:
                            speed_knots = float(parts[7])
                            speed_kmh = speed_knots * 1.852
                        except:
                            speed_kmh = None

            # --- GGA sentence (time, lat, lon, fix quality) ---
            elif line.startswith("$GPGGA"):
                parts = line.split(",")
                if len(parts) > 6 and parts[6] != "0":   # fix quality > 0 means valid fix
                    utc_time = format_utc(parts[1])
                    latitude, longitude = convert_lat_lon(parts[2], parts[3], parts[4], parts[5])

    except Exception as e:
        print("GPS parse error:", e)

    # --- Build message ---
    if latitude is not None and longitude is not None:
        spd_text = f"{speed_kmh:.2f}km/h" if speed_kmh is not None else "N/A"
        time_text = utc_time if utc_time else "N/A"
        message = f"GPS:{latitude:.5f},{longitude:.5f} Spd:{spd_text} Time:{time_text} Cnt:{counter}"
    else:
        message = f"No GPS Fix Cnt:{counter}"

    # --- Send via LoRa ---
    try:
        message_bytes = [ord(c) for c in message]
        LoRa.beginPacket()
        LoRa.write(message_bytes, len(message_bytes))
        LoRa.endPacket()
        LoRa.wait()

        print(f"Sent: {message}")
        print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s"
              .format(LoRa.transmitTime(), LoRa.dataRate()))
    except Exception as e:
        print("LoRa send error:", e)

    time.sleep(5)
    counter = (counter + 1) % 256

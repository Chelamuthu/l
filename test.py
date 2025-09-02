import os, sys
import time
import serial
from LoRaRF import SX126x

# ---------- LoRa setup ----------
busId = 0; csId = 0
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1
LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Something wrong, can't begin LoRa radio")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)                 # 865–867 MHz (India ISM band)
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 128, True)  
LoRa.setSyncWord(0x3444)

print("\n-- LoRa GNSS Transmitter --\n")

# ---------- GNSS setup ----------
GNSS_PORT = "/dev/ttyAMA0"       
GNSS_BAUD = 9600

try:
    gps_serial = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
    print(f"Opened GNSS on {GNSS_PORT}")
except Exception as e:
    print("Error opening GNSS:", e)
    gps_serial = None

# ---------- Helpers ----------
def convert_lat_lon(lat_str, ns, lon_str, ew):
    """Convert NMEA ddmm.mmmm to decimal degrees"""
    if not lat_str or not lon_str or ns not in ("N","S") or ew not in ("E","W"):
        return None, None
    try:
        # Latitude (2° digits)
        lat_deg = float(lat_str[:2])
        lat_min = float(lat_str[2:])
        lat = lat_deg + lat_min / 60.0
        if ns == "S": lat = -lat

        # Longitude (3° digits)
        lon_deg = float(lon_str[:3])
        lon_min = float(lon_str[3:])
        lon = lon_deg + lon_min / 60.0
        if ew == "W": lon = -lon

        return lat, lon
    except:
        return None, None

def is_rmc(sentence):
    return sentence.startswith("$") and sentence[3:6] == "RMC"

counter = 0
last_fix = None

while True:
    latitude = longitude = speed_kmh = None

    try:
        line = gps_serial.readline().decode("utf-8", errors="ignore").strip() if gps_serial else ""
        
        # -------- Parse RMC: lat, lon, speed --------
        if is_rmc(line):
            parts = line.split(",")
            if len(parts) >= 9 and parts[2] == "A":   # 'A' = valid fix
                lat, lon = convert_lat_lon(parts[3], parts[4], parts[5], parts[6])
                if lat is not None and lon is not None:
                    latitude, longitude = lat, lon
                try:
                    if parts[7] != "":  # speed in knots -> km/h
                        speed_kmh = float(parts[7]) * 1.852
                except:
                    pass
    except Exception as e:
        print("GPS parse error:", e)

    # Fallback to last known fix
    if latitude is None or longitude is None:
        if last_fix:
            latitude, longitude, speed_kmh = last_fix
    else:
        last_fix = (latitude, longitude, speed_kmh)

    # -------- Build & send LoRa message --------
    if latitude is not None and longitude is not None:
        spd_txt = f"{speed_kmh:.2f}" if isinstance(speed_kmh, (int,float)) else "0.00"
        message = f"{latitude:.6f},{longitude:.6f},{spd_txt},{counter}"
    else:
        message = f"NO_FIX,{counter}"

    try:
        payload = [ord(c) for c in message]
        LoRa.beginPacket()
        LoRa.write(payload, len(payload))
        LoRa.endPacket()
        LoRa.wait()

        print(f"Sent: {message}")
        print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s"
              .format(LoRa.transmitTime(), LoRa.dataRate()))
    except Exception as e:
        print("LoRa send error:", e)

    time.sleep(1.0)   # send every 1 second
    counter = (counter + 1) % 256

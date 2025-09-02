#!/usr/bin/env python3
import time, serial
import RPi.GPIO as GPIO
from LoRaRF import SX126x

# ---------------- GPIO FIX ----------------
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# ---------------- GNSS SETUP ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

def nmea_to_decimal(raw, hemi, is_lat=True):
    """Convert NMEA lat/lon to decimal degrees"""
    try:
        if not raw or raw == "":
            return None
        deg_len = 2 if is_lat else 3
        deg = float(raw[:deg_len])
        minutes = float(raw[deg_len:])
        decimal = deg + minutes / 60
        if hemi in ["S", "W"]:
            decimal *= -1
        return decimal
    except:
        return None

def get_gps_data():
    """Read GPS NMEA data and return lat, lon, fix status"""
    try:
        line = gps.readline().decode(errors="ignore").strip()
        if line.startswith("$GNGGA"):  # GPS fix data
            parts = line.split(",")
            if len(parts) < 7:
                return None, None, False
            lat = nmea_to_decimal(parts[2], parts[3], is_lat=True)
            lon = nmea_to_decimal(parts[4], parts[5], is_lat=False)
            fix = parts[6] != "0"  # fix quality (0 = no fix)
            return lat, lon, fix
    except Exception as e:
        print("GPS read error:", e)
    return None, None, False

# ---------------- LoRa SETUP ----------------
busId = 0; csId = 0
resetPin = 18; busyPin = 20; irqPin = -1
txenPin = 6; rxenPin = -1

LoRa = SX126x()
print("Begin LoRa radio...")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    print("LoRa init failed")
    exit(1)

LoRa.setFrequency(868000000)   # Adjust freq for your region (e.g., 868 MHz / 915 MHz / 433 MHz)
LoRa.setTxPower(22, SX126x.TX_POWER_SX1262)
LoRa.setSpreadingFactor(7)
LoRa.setBandwidth(125000)
LoRa.setCodingRate(5)
LoRa.setSyncWord(0x1424)

print("LoRa init success")

# ---------------- MAIN LOOP ----------------
while True:
    lat, lon, fix = get_gps_data()

    if fix and lat and lon:
        msg = f"LAT:{lat:.6f}, LON:{lon:.6f}"
    else:
        msg = "NO FIX"

    print("TX:", msg)
    LoRa.beginPacket()
    LoRa.print(msg)
    LoRa.endPacket()

    time.sleep(2)

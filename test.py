#!/usr/bin/env python3
import time
import serial
import RPi.GPIO as GPIO
from LoRaRF import SX126x

# ---------------- GPIO SETUP ----------------
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# ---------------- GPS SETUP ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.5)

# ---------------- LoRa SETUP ----------------
lora = SX126x()
lora.begin(freq=868000000, bw=125000, sf=7, cr=5, syncWord=0x12, power=22, currentLimit=60.0, preambleLength=8, implicit=False, implicitLen=0xFF)

print("LoRa transmitter ready...")

def parse_gga(sentence):
    try:
        parts = sentence.split(",")
        if parts[0] != "$GPGGA":
            return None, None

        # Latitude
        lat_raw = parts[2]
        lat_hemi = parts[3]
        lon_raw = parts[4]
        lon_hemi = parts[5]

        if lat_raw == "" or lon_raw == "":
            return None, None

        lat_deg = float(lat_raw[:2])
        lat_min = float(lat_raw[2:])
        latitude = lat_deg + lat_min / 60.0
        if lat_hemi == "S":
            latitude *= -1

        lon_deg = float(lon_raw[:3])
        lon_min = float(lon_raw[3:])
        longitude = lon_deg + lon_min / 60.0
        if lon_hemi == "W":
            longitude *= -1

        return latitude, longitude
    except Exception:
        return None, None

while True:
    line = gps.readline().decode("ascii", errors="ignore").strip()
    if line.startswith("$GPGGA"):
        lat, lon = parse_gga(line)
        if lat and lon:
            msg = f"{lat:.6f},{lon:.6f}"
            print(f"Sending: {msg}")
            lora.beginPacket()
            lora.print(msg)
            lora.endPacket()
    time.sleep(1)

#!/usr/bin/env python3
import serial
import time
import RPi.GPIO as GPIO
from LoRaRF import SX126x

# ---------------- GPIO Setup ----------------
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# ---------------- GPS Setup ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

def nmea_to_decimal(raw, hemi, is_lat=True):
    """Convert NMEA coordinate format to decimal degrees"""
    try:
        if is_lat:  # Latitude DDMM.MMMM
            deg = int(raw[0:2])
            minutes = float(raw[2:])
        else:  # Longitude DDDMM.MMMM
            deg = int(raw[0:3])
            minutes = float(raw[3:])
        decimal = deg + (minutes / 60.0)
        if hemi in ["S", "W"]:
            decimal = -decimal
        return round(decimal, 6)
    except:
        return None

def get_gps():
    """Fetch latitude and longitude from NMEA GPGGA/GPRMC sentences"""
    while True:
        line = gps.readline().decode("ascii", errors="ignore").strip()
        if line.startswith("$GPGGA") or line.startswith("$GPRMC"):
            parts = line.split(",")
            if len(parts) > 5 and parts[2] and parts[4]:
                lat = nmea_to_decimal(parts[2], parts[3], True)
                lon = nmea_to_decimal(parts[4], parts[5], False)
                if lat and lon:
                    return lat, lon

# ---------------- LoRa Setup ----------------
lora = SX126x()
lora.begin(freq=865.2, bw=125.0, sf=7, cr=5, syncWord=0x12, power=14, currentLimit=60.0, 
           preambleLength=8, implicit=False, implicitLen=0xFF, crc=True, invertIQ=False)
print("LoRa initialized for GNSS telemetry...")

# ---------------- Main Loop ----------------
while True:
    lat, lon = get_gps()
    msg = f"LAT:{lat}, LON:{lon}"
    print("Sending:", msg)
    
    lora.send(msg.encode("utf-8"))
    time.sleep(1)  # 1 second delay

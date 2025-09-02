#!/usr/bin/env python3
import serial
import time
from LoRaRF import SX126x
import RPi.GPIO as GPIO

# ---------------- GPIO Setup ----------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.cleanup()

# ---------------- GPS Setup ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

# ---------------- LoRa Setup ----------------
lora = SX126x()
lora.begin()
lora.setFrequency(865000000)   # Adjust frequency as per your region
lora.setTxPower(22, 1)         # 22 dBm, high power PA
lora.setLoRaModulation(7, 125000, 5)  # SF7, BW=125kHz, CR=4/5
lora.setPacketParams(255, 0, 1, True, False)

def nmea_to_decimal(raw, hemi, is_lat=True):
    """Convert NMEA coordinate to decimal degrees"""
    try:
        if is_lat:
            deg = int(raw[:2])
            mins = float(raw[2:])
        else:
            deg = int(raw[:3])
            mins = float(raw[3:])
        dec = deg + mins / 60
        if hemi in ["S", "W"]:
            dec = -dec
        return dec
    except:
        return None

def get_gps_data():
    """Read GPS data from serial"""
    while True:
        line = gps.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("$GPGGA"):
            parts = line.split(",")
            if len(parts) > 5 and parts[2] and parts[4]:
                lat = nmea_to_decimal(parts[2], parts[3], is_lat=True)
                lon = nmea_to_decimal(parts[4], parts[5], is_lat=False)
                return lat, lon
        time.sleep(0.1)

# ---------------- Main Loop ----------------
print("LoRa GNSS Transmitter started...")

while True:
    lat, lon = get_gps_data()
    if lat and lon:
        message = f"{lat:.6f},{lon:.6f}"
        print(f"Transmitting: {message}")
        lora.beginPacket()
        lora.print(message)
        lora.endPacket()
    time.sleep(1)   # send every 1 sec

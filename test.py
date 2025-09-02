#!/usr/bin/env python3
import time, serial
from LoRaRF import SX126x
import RPi.GPIO as GPIO
from datetime import datetime

# ---------------- GPIO SETUP ----------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# ---------------- GPS SETUP ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

def parse_nmea_latlon(nmea_sentence):
    """Extract latitude, longitude, speed from NMEA GPRMC"""
    try:
        parts = nmea_sentence.split(",")
        if parts[0].endswith("RMC") and parts[2] == "A":  # Valid fix
            # Latitude
            raw_lat = parts[3]
            lat_dir = parts[4]
            lat = float(raw_lat[:2]) + float(raw_lat[2:]) / 60
            if lat_dir == "S":
                lat = -lat

            # Longitude
            raw_lon = parts[5]
            lon_dir = parts[6]
            lon = float(raw_lon[:3]) + float(raw_lon[3:]) / 60
            if lon_dir == "W":
                lon = -lon

            # Speed (knots → km/h)
            speed_knots = float(parts[7])
            speed_kmh = speed_knots * 1.852

            return lat, lon, speed_kmh
    except:
        return None
    return None

# ---------------- LORA SETUP ----------------
lora = SX126x()
lora.begin()

# Frequency 868 MHz (set as per your dongle)
lora.setFrequency(868000000)

# Modulation Params
lora.setModulationParams(
    lora.SF7,          # spreading factor
    lora.BW125,        # bandwidth 125kHz
    lora.CR_4_5,       # coding rate 4/5
    lora.LDRO_AUTO     # auto low data rate optimize
)

# Packet Params
lora.setPacketParams(
    preambleLength=12,
    headerType=lora.HEADER_EXPLICIT,
    payloadLength=255,
    crcType=lora.CRC_ON,
    invertIQ=lora.IQ_STANDARD
)

# ---------------- MAIN LOOP ----------------
counter = 0
print("LoRa GNSS transmitter started...")

while True:
    line = gps.readline().decode("utf-8", errors="ignore").strip()
    gnss_data = parse_nmea_latlon(line)

    if gnss_data:
        lat, lon, speed = gnss_data
        now = datetime.now().strftime("%H:%M:%S")

        message = (
            f"Latitude: {lat:.6f}, Longitude: {lon:.6f}, "
            f"Speed: {speed:.2f} km/h, Counter: {counter}, Time: {now}"
        )

        # Clear FIFO if needed
        lora.flushBuffer()

        # Transmit
        lora.send(message.encode("utf-8"))

        # Print same as transmitted
        print(message)

        # Roll counter (0–255)
        counter = (counter + 1) % 256

        time.sleep(1)  # adjust interval if needed

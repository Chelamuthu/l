#!/usr/bin/env python3
import time
import serial
from LoRaRF import SX126x
import RPi.GPIO as GPIO

# ---------------- GPIO SETUP ----------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# ---------------- GNSS SETUP ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.5)

# ---------------- LoRa SETUP ----------------
lora = SX126x()
lora.begin()
lora.setFrequency(868000000)   # set frequency (adjust for your region)
lora.setTxPower(22, lora.TX_POWER_SX1262)
lora.setLoRaModulation(7, 5, 125000, lora.LDRO_AUTO)

# Correct buffer-safe packet params
lora.setPacketParams(
    12,                        # preambleLength
    lora.HEADER_EXPLICIT,      # headerType
    255,                       # payloadLength (max)
    lora.CRC_ON,               # crcType
    lora.IQ_STANDARD           # invertIQ
)

# ---------------- UTILITIES ----------------
def nmea_to_decimal(raw, hemi, is_lat=True):
    """Convert NMEA raw coordinate to decimal degrees."""
    try:
        deg_len = 2 if is_lat else 3
        deg = float(raw[:deg_len])
        minutes = float(raw[deg_len:])
        decimal = deg + (minutes / 60.0)
        if hemi in ['S', 'W']:
            decimal *= -1
        return decimal
    except:
        return None

def read_gnss():
    """Read one valid GNSS line and extract lat, lon, speed."""
    line = gps.readline().decode("ascii", errors="ignore").strip()
    if line.startswith("$GPRMC"):
        parts = line.split(",")
        if len(parts) > 7 and parts[2] == "A":  # Data valid
            lat = nmea_to_decimal(parts[3], parts[4], True)
            lon = nmea_to_decimal(parts[5], parts[6], False)
            speed = float(parts[7]) * 1.852  # knots → km/h
            return lat, lon, speed
    return None, None, None

# ---------------- MAIN LOOP ----------------
counter = 0
print("Starting GNSS → LoRa transmission...")

while True:
    lat, lon, speed = read_gnss()
    if lat and lon:
        now = time.strftime("%Y-%m-%d %H:%M:%S")
        message = f"Latitude={lat:.6f}, Longitude={lon:.6f}, Speed={speed:.2f}km/h, Counter={counter}, Time={now}"

        # --- BUFFER HANDLING ---
        try:
            lora.send(message.encode("utf-8"))
        except Exception as e:
            print("⚠ Buffer overflow, clearing FIFO and retrying...")
            lora.begin()  # re-init clears buffer
            lora.setFrequency(868000000)
            lora.setTxPower(22, lora.TX_POWER_SX1262)
            lora.setLoRaModulation(7, 5, 125000, lora.LDRO_AUTO)
            lora.setPacketParams(12, lora.HEADER_EXPLICIT, 255, lora.CRC_ON, lora.IQ_STANDARD)
            lora.send(message.encode("utf-8"))

        print("TX ->", message)

        counter = (counter + 1) % 256  # roll over at 255

    time.sleep(1)  # small delay to avoid flooding

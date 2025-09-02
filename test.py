#!/usr/bin/env python3
import time
import serial
from LoRaRF import SX126x

# ---------------- GPS Setup ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

# ---------------- LoRa Setup ----------------
lora = SX126x()
lora.begin()
lora.setFrequency(868000000)  # Adjust frequency as per your module
lora.setTxPower(22)
lora.setLoRaModulation(bw=125000, sf=7, cr=5)
lora.setLoRaPacket(preambleLength=12,
                   headerType=lora.HEADER_EXPLICIT,
                   payloadLength=255,
                   crcType=lora.CRC_ON,
                   invertIQ=lora.IQ_STANDARD)

# ---------------- Helpers ----------------
def nmea_to_decimal(raw, hemi, is_lat=True):
    try:
        raw_val = float(raw)
        if is_lat:
            deg = int(raw_val / 100)
            minutes = raw_val - deg * 100
            decimal = deg + minutes / 60
        else:
            deg = int(raw_val / 100)
            minutes = raw_val - deg * 100
            decimal = deg + minutes / 60
        if hemi in ["S", "W"]:
            decimal = -decimal
        return round(decimal, 6)
    except:
        return None

def parse_gprmc(nmea):
    parts = nmea.split(",")
    if parts[0].endswith("RMC") and parts[2] == "A":  # Valid fix
        lat = nmea_to_decimal(parts[3], parts[4], True)
        lon = nmea_to_decimal(parts[5], parts[6], False)
        speed = float(parts[7]) * 1.852  # knots → km/h
        return lat, lon, round(speed, 2)
    return None, None, None

# ---------------- Main Loop ----------------
counter = 0

print("LoRa GNSS Transmitter Started...")
while True:
    try:
        line = gps.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("$GPRMC"):
            lat, lon, speed = parse_gprmc(line)
            if lat and lon:
                timestamp = time.strftime("%H:%M:%S")
                message = (f"Latitude: {lat}, Longitude: {lon}, "
                           f"Speed: {speed} km/h, "
                           f"Counter: {counter}, Time: {timestamp}")

                # Send over LoRa
                lora.beginPacket()
                lora.print(message)
                lora.endPacket()

                # Print exactly what is sent
                print("Transmitting ->", message)

                # Update counter (0–255 rollover)
                counter = (counter + 1) % 256
                time.sleep(1)  # 1s interval
    except KeyboardInterrupt:
        print("\nStopped by user")
        break
    except Exception as e:
        print("Error:", e)

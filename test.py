#!/usr/bin/python3
# -- coding: UTF-8 --

import os
import sys
import time
import serial
import pynmea2
from datetime import datetime

# ==============================
# CONFIGURATION
# ==============================
UART_PORT = "/dev/ttyAMA0"  # Neo-6M GPS connected to Raspberry Pi UART
BAUDRATE = 9600

# LoRa Configuration
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1
payloadLength = 100

# ==============================
# IMPORT LORA LIBRARY
# ==============================
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# ==============================
# INITIALIZE SERIAL (GPS)
# ==============================
try:
    gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=0.5)  # Non-blocking, fast reading
    print(f"[INFO] Connected to GPS module on {UART_PORT} at {BAUDRATE} baud.")
except serial.SerialException as e:
    print(f"[ERROR] Cannot open GPS port {UART_PORT}: {e}")
    sys.exit(1)

# ==============================
# INITIALIZE LORA
# ==============================
print("[INFO] Initializing LoRa...")
LoRa = SX126x()

if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Failed to initialize LoRa module!")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)
LoRa.setTxPower(14, LoRa.TX_POWER_SX1262)  # Safe power
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, payloadLength, True)
LoRa.setSyncWord(0x3444)
print("[INFO] LoRa ready.\n")

# ==============================
# PARSE GPS DATA
# ==============================
def parse_gps_data(line):
    """Parse NMEA sentence and return GPS fix info."""
    try:
        msg = pynmea2.parse(line)
        
        # Use RMC for location + speed
        if isinstance(msg, pynmea2.types.talker.RMC) and msg.status == "A":  # A = Active fix
            speed_knots = msg.spd_over_grnd or 0.0
            speed_kmh = speed_knots * 1.852  # Convert knots to km/h
            return {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "speed_kmh": speed_kmh,
                "date": msg.datestamp.strftime("%d-%m-%Y") if msg.datestamp else "N/A",
                "time": msg.timestamp.strftime("%H:%M:%S") if msg.timestamp else "N/A"
            }

        # GGA can confirm fix quality
        if isinstance(msg, pynmea2.types.talker.GGA) and int(msg.gps_qual) > 0:
            return {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "speed_kmh": 0.0,  # No speed data in GGA
                "date": datetime.utcnow().strftime("%d-%m-%Y"),
                "time": datetime.utcnow().strftime("%H:%M:%S")
            }
        return None
    except Exception:
        return None

# ==============================
# SEND LORA MESSAGE
# ==============================
def send_lora_message(message):
    """Send a message through LoRa, truncated if too long."""
    if len(message) > payloadLength:
        message = message[:payloadLength]
    message_bytes = list(message.encode('utf-8'))
    LoRa.beginPacket()
    LoRa.write(message_bytes, len(message_bytes))
    LoRa.endPacket()
    LoRa.wait()
    print(f"[INFO] Sent via LoRa: {message}")

# ==============================
# MAIN LOOP
# ==============================
print("[INFO] Starting live GPS tracking...\n")

try:
    while True:
        start_time = time.time()
        gps_data = None

        # Continuously read for ~1 second to catch the latest GPS sentence
        while time.time() - start_time < 1.0:
            line = gps_serial.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$'):
                parsed = parse_gps_data(line)
                if parsed:
                    gps_data = parsed  # Use last valid fix in this second

        # Build message
        if gps_data:
            message = (
                f"GNSS|Date:{gps_data['date']}|Time:{gps_data['time']}|"
                f"Lat:{gps_data['latitude']:.6f}|Lon:{gps_data['longitude']:.6f}|"
                f"Speed:{gps_data['speed_kmh']:.2f}km/h"
            )
        else:
            message = "NO GNSS FIX"

        # Send via LoRa
        send_lora_message(message)

except KeyboardInterrupt:
    print("\n[INFO] Stopped by user.")
finally:
    gps_serial.close()
    LoRa.end()
    print("[INFO] Closed GPS and LoRa safely.")

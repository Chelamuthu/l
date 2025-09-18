#!/usr/bin/python3
# -- coding: UTF-8 --

import os
import sys
import time
import serial
import pynmea2
from datetime import datetime

# =======================================
# CONFIGURATION
# =======================================
UART_PORT = "/dev/ttyAMA0"  # GPS UART port
BAUDRATE = 9600

# LoRa Configuration
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1
payloadLength = 100  # Max LoRa payload size

# =======================================
# IMPORT LORA LIBRARY
# =======================================
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# =======================================
# INITIALIZE SERIAL (GPS)
# =======================================
try:
    gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=1)
    print(f"[INFO] Connected to GPS module on {UART_PORT} at {BAUDRATE} baud.")
except serial.SerialException as e:
    print(f"[ERROR] Cannot open GPS port {UART_PORT}: {e}")
    sys.exit(1)

# =======================================
# INITIALIZE LORA
# =======================================
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

# =======================================
# HELPER FUNCTION: PARSE GPS DATA
# =======================================
def parse_gps_data(line):
    try:
        msg = pynmea2.parse(line)
        if isinstance(msg, pynmea2.types.talker.RMC) and msg.status == "A":
            speed_knots = msg.spd_over_grnd or 0.0
            speed_kmh = speed_knots * 1.852
            return {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "speed_kmh": speed_kmh,
                "date": msg.datestamp.strftime("%d-%m-%Y") if msg.datestamp else "N/A",
                "time": msg.timestamp.strftime("%H:%M:%S") if msg.timestamp else "N/A"
            }
        return None
    except Exception:
        return None

# =======================================
# MAIN LOOP
# =======================================
print("[INFO] Starting GPS + LoRa transmission...\n")
last_gps_data = None  # Store last valid coordinates

try:
    while True:
        try:
            line = gps_serial.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$'):
                gps_data = parse_gps_data(line)
                if gps_data:
                    last_gps_data = gps_data

            # Use last known GPS if current line is invalid
            if last_gps_data:
                message = (
                    f"RMC|Date:{last_gps_data['date']}|Time:{last_gps_data['time']}|"
                    f"Lat:{last_gps_data['latitude']:.6f}|Lon:{last_gps_data['longitude']:.6f}|"
                    f"Speed:{last_gps_data['speed_kmh']:.2f}km/h"
                )

                if len(message) > payloadLength:
                    message = message[:payloadLength]

                message_bytes = list(message.encode('utf-8'))
                LoRa.beginPacket()
                LoRa.write(message_bytes, len(message_bytes))
                LoRa.endPacket()
                LoRa.wait()

                print(f"[INFO] Sent via LoRa: {message}")

            else:
                print("[INFO] Waiting for initial GPS fix...")

            time.sleep(1)  # Transmit every 1 second

        except Exception as e:
            print(f"[ERROR] Loop error: {e}")
            time.sleep(1)

except KeyboardInterrupt:
    print("\n[INFO] Stopped by user.")

finally:
    gps_serial.close()
    LoRa.end()
    print("[INFO] Closed GPS and LoRa safely.")

#!/usr/bin/python3
# -- coding: UTF-8 --

import os
import sys
import time
import serial
import pynmea2
from datetime import datetime

# ===============================
# CONFIGURATION
# ===============================
UART_PORT = "/dev/ttyAMA0"
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

# ===============================
# IMPORT LORA LIBRARY
# ===============================
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# ===============================
# INITIALIZE SERIAL (GPS)
# ===============================
try:
    gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=0.1)
    print(f"[INFO] Connected to GPS on {UART_PORT} at {BAUDRATE} baud.")
except serial.SerialException as e:
    print(f"[ERROR] Cannot open GPS port {UART_PORT}: {e}")
    sys.exit(1)

# ===============================
# INITIALIZE LORA
# ===============================
LoRa = SX126x()
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Failed to initialize LoRa module!")
LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)
LoRa.setTxPower(14, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, payloadLength, True)
LoRa.setSyncWord(0x3444)
print("[INFO] LoRa ready.\n")

# ===============================
# GPS PARSER
# ===============================
def parse_gps_data(line):
    try:
        msg = pynmea2.parse(line)
        if isinstance(msg, pynmea2.types.talker.GGA):
            if msg.latitude and msg.longitude:
                return {
                    "type": "GGA",
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "altitude": msg.altitude,
                    "timestamp": msg.timestamp.strftime("%H:%M:%S") if msg.timestamp else "N/A"
                }
        elif isinstance(msg, pynmea2.types.talker.RMC):
            if msg.status == "A":
                speed_kmh = (msg.spd_over_grnd or 0.0) * 1.852
                return {
                    "type": "RMC",
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "speed_kmh": speed_kmh,
                    "date": msg.datestamp.strftime("%d-%m-%Y") if msg.datestamp else "N/A",
                    "timestamp": msg.timestamp.strftime("%H:%M:%S") if msg.timestamp else "N/A"
                }
        return None
    except:
        return None

# ===============================
# MAIN LOOP
# ===============================
print("[INFO] Starting continuous GPS + LoRa transmission...\n")
last_gps_data = None
buffer = ""

try:
    while True:
        # Read all available data from GPS without blocking
        data = gps_serial.read(1024).decode('ascii', errors='replace')
        if data:
            buffer += data
            lines = buffer.split('\n')
            buffer = lines[-1]  # incomplete line stays in buffer
            for line in lines[:-1]:
                line = line.strip()
                if line.startswith('$'):
                    gps_data = parse_gps_data(line)
                    if gps_data:
                        last_gps_data = gps_data

        # Send last valid GPS fix every second
        if last_gps_data:
            if last_gps_data["type"] == "GGA":
                message = (f"GGA|Time:{last_gps_data['timestamp']}|"
                           f"Lat:{last_gps_data['latitude']:.6f}|"
                           f"Lon:{last_gps_data['longitude']:.6f}|"
                           f"Alt:{last_gps_data['altitude']}m")
            else:
                message = (f"RMC|Date:{last_gps_data['date']}|Time:{last_gps_data['timestamp']}|"
                           f"Lat:{last_gps_data['latitude']:.6f}|"
                           f"Lon:{last_gps_data['longitude']:.6f}|"
                           f"Speed:{last_gps_data['speed_kmh']:.2f}km/h")

            if len(message) > payloadLength:
                message = message[:payloadLength]

            try:
                LoRa.beginPacket()
                LoRa.write(list(message.encode('utf-8')), len(message))
                LoRa.endPacket()
                LoRa.wait()
                print(f"[INFO] Sent via LoRa: {message}")
            except Exception as e:
                print(f"[ERROR] Failed to send LoRa packet: {e}")

        time.sleep(1)  # Send interval

except KeyboardInterrupt:
    print("\n[INFO] Transmission stopped by user.")
finally:
    gps_serial.close()
    LoRa.end()
    print("[INFO] Closed GPS and LoRa safely.")

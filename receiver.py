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
UART_PORT = "/dev/ttyAMA0"  # GNSS UART (Neo-6M or onboard GNSS)
BAUDRATE = 9600

# LoRa Configuration (SX1262 Expansion Board)
BUS_ID = 0
CS_ID = 0
RESET_PIN = 18
BUSY_PIN = 20
IRQ_PIN = -1
TX_EN = 6
RX_EN = -1
PAYLOAD_LENGTH = 100
LORA_FREQ = 868000000         # Frequency band
LORA_TX_POWER = 12            # Safe TX power (dBm)
TX_INTERVAL = 1               # Seconds

# ==============================
# IMPORT LORA LIBRARY
# ==============================
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# ==============================
# INITIALIZE SERIAL (GNSS)
# ==============================
try:
    gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=1)
    print(f"[INFO] GNSS connected on {UART_PORT} at {BAUDRATE} baud.")
except serial.SerialException as e:
    print(f"[ERROR] Cannot open GNSS port {UART_PORT}: {e}")
    sys.exit(1)

# ==============================
# INITIALIZE LORA
# ==============================
print("[INFO] Initializing SX1262 LoRa Module...")
LoRa = SX126x()

if not LoRa.begin(BUS_ID, CS_ID, RESET_PIN, BUSY_PIN, IRQ_PIN, TX_EN, RX_EN):
    raise Exception("Failed to initialize LoRa module!")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(LORA_FREQ)
LoRa.setTxPower(LORA_TX_POWER, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, PAYLOAD_LENGTH, True)
LoRa.setSyncWord(0x3444)
print("[INFO] LoRa ready.\n")

# ==============================
# GNSS PARSER
# ==============================
def parse_gnss_data(line):
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

# ==============================
# MAIN LOOP
# ==============================
print("[INFO] Starting GNSS + LoRa transmission...\n")
last_tx_time = time.monotonic()

try:
    while True:
        line = gps_serial.readline().decode('ascii', errors='replace').strip()
        if not line.startswith('$'):
            continue

        gnss_data = parse_gnss_data(line)
        current_time = time.monotonic()

        # Transmit every TX_INTERVAL seconds if GNSS fix available
        if gnss_data and current_time - last_tx_time >= TX_INTERVAL:
            last_tx_time = current_time

            message = (
                f"RMC|Date:{gnss_data['date']}|Time:{gnss_data['time']}|"
                f"Lat:{gnss_data['latitude']:.6f}|Lon:{gnss_data['longitude']:.6f}|"
                f"Speed:{gnss_data['speed_kmh']:.2f}km/h"
            )

            if len(message) > PAYLOAD_LENGTH:
                message = message[:PAYLOAD_LENGTH]

            try:
                message_bytes = list(message.encode('utf-8'))
                LoRa.beginPacket()
                LoRa.write(message_bytes, len(message_bytes))
                LoRa.endPacket()
                LoRa.wait()
                print(f"[INFO] Sent via LoRa: {message}")
                time.sleep(0.2)  # short delay to reduce current spike
            except Exception as e:
                print(f"[ERROR] LoRa send failed: {e}")

except KeyboardInterrupt:
    print("\n[INFO] Transmission stopped by user.")

finally:
    gps_serial.close()
    LoRa.end()
    print("[INFO] Closed GNSS and LoRa safely.")

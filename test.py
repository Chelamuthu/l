#!/usr/bin/python3
# -- coding: UTF-8 --

import os
import sys
import time
import serial
import pynmea2
from datetime import datetime
import hashlib

# ==============================
# CONFIGURATION
# ==============================
UART_PORT = "/dev/ttyAMA0"      # Neo-6M GPS connected to Pi UART
BAUDRATE = 9600

# LoRa Configuration
LORA_FREQ = 868000000
PAYLOAD_LENGTH = 100
LORA_POWER = 14  # dBm
TX_INTERVAL = 1  # seconds between transmissions

# SX1262 Pin Configuration
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1

# ==============================
# IMPORT LORA LIBRARY
# ==============================
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# ==============================
# INITIALIZATION
# ==============================
def init_gps():
    """Initialize GPS serial connection."""
    try:
        gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=1)
        log(f"GPS connected on {UART_PORT} at {BAUDRATE} baud.")
        return gps_serial
    except serial.SerialException as e:
        log(f"[ERROR] Cannot open GPS port {UART_PORT}: {e}", error=True)
        sys.exit(1)

def init_lora():
    """Initialize LoRa module."""
    log("Initializing LoRa...")
    LoRa = SX126x()
    if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
        raise Exception("Failed to initialize LoRa module!")

    LoRa.setDio2RfSwitch()
    LoRa.setFrequency(LORA_FREQ)
    LoRa.setTxPower(LORA_POWER, LoRa.TX_POWER_SX1262)
    LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
    LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, PAYLOAD_LENGTH, True)
    LoRa.setSyncWord(0x3444)
    log("LoRa initialized successfully.")
    return LoRa

def log(message, error=False):
    """Timestamped logging."""
    now = datetime.now().strftime("%H:%M:%S")
    prefix = "[ERROR]" if error else "[INFO]"
    print(f"{prefix} {now} - {message}")

# ==============================
# GPS DATA PARSING
# ==============================
def parse_gps_data(line):
    """Parse NMEA GPS data and return useful fields."""
    try:
        msg = pynmea2.parse(line)
        if isinstance(msg, pynmea2.types.talker.RMC) and msg.status == "A":
            speed_knots = msg.spd_over_grnd or 0.0
            speed_kmh = speed_knots * 1.852
            return {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "speed_kmh": round(speed_kmh, 2),
                "date": msg.datestamp.strftime("%d-%m-%Y") if msg.datestamp else "N/A",
                "time": msg.timestamp.strftime("%H:%M:%S") if msg.timestamp else "N/A"
            }
        return None
    except Exception:
        return None

# ==============================
# CHECKSUM GENERATION
# ==============================
def generate_checksum(data_str):
    """Generate simple checksum for data integrity."""
    return hashlib.md5(data_str.encode('utf-8')).hexdigest()[:6]

# ==============================
# SEND VIA LORA
# ==============================
def send_lora_data(LoRa, message):
    """Send data via LoRa after trimming to payload length."""
    if len(message) > PAYLOAD_LENGTH:
        message = message[:PAYLOAD_LENGTH]

    message_bytes = list(message.encode('utf-8'))
    LoRa.beginPacket()
    LoRa.write(message_bytes, len(message_bytes))
    LoRa.endPacket()
    LoRa.wait()
    log(f"Sent via LoRa: {message}")

# ==============================
# MAIN FUNCTION
# ==============================
def main():
    gps_serial = init_gps()
    LoRa = init_lora()

    packet_counter = 0
    gps_fix_acquired = False
    last_sent_time = time.monotonic()

    log("Starting GPS + LoRa transmission...\n")

    try:
        while True:
            line = gps_serial.readline().decode('ascii', errors='replace').strip()
            if not line.startswith('$'):
                continue

            gps_data = parse_gps_data(line)

            # Send every TX_INTERVAL seconds
            current_time = time.monotonic()
            if current_time - last_sent_time >= TX_INTERVAL:
                last_sent_time = current_time

                if gps_data:
                    gps_fix_acquired = True
                    packet_counter += 1

                    # Build structured message
                    core_message = (
                        f"PKT:{packet_counter}|Date:{gps_data['date']}|Time:{gps_data['time']}|"
                        f"Lat:{gps_data['latitude']:.6f}|Lon:{gps_data['longitude']:.6f}|"
                        f"Speed:{gps_data['speed_kmh']:.2f}km/h"
                    )
                    checksum = generate_checksum(core_message)
                    final_message = f"{core_message}|CHK:{checksum}"

                    send_lora_data(LoRa, final_message)

                else:
                    if not gps_fix_acquired:
                        log("Waiting for GPS fix...")
                        gps_fix_acquired = True  # Only log once until fix

    except KeyboardInterrupt:
        log("Stopped by user.")
    finally:
        gps_serial.close()
        LoRa.end()
        log("Closed GPS and LoRa safely.")

# ==============================
# ENTRY POINT
# ==============================
if __name__ == "__main__":
    main()

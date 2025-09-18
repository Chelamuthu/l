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
    gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=0.5)  # Fast non-blocking
except serial.SerialException as e:
    sys.exit(f"[ERROR] Cannot open GPS port {UART_PORT}: {e}")

# ==============================
# INITIALIZE LORA
# ==============================
LoRa = SX126x()
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    sys.exit("[ERROR] Failed to initialize LoRa module!")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)
LoRa.setTxPower(14, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, payloadLength, True)
LoRa.setSyncWord(0x3444)

# ==============================
# PARSE GPS DATA
# ==============================
def parse_gps_data(line):
    """
    Parse GPS NMEA sentences and return latitude, longitude, speed, and timestamp.
    Returns None if no valid fix or corrupted data.
    """
    try:
        msg = pynmea2.parse(line)

        # Use RMC for accurate position + speed
        if isinstance(msg, pynmea2.types.talker.RMC) and msg.status == "A":
            if msg.latitude == 0.0 or msg.longitude == 0.0:
                return None
            speed_knots = msg.spd_over_grnd or 0.0
            speed_kmh = speed_knots * 1.852
            timestamp = msg.timestamp.strftime("%H:%M:%S") if msg.timestamp else datetime.utcnow().strftime("%H:%M:%S")
            return {
                "latitude": round(msg.latitude, 6),
                "longitude": round(msg.longitude, 6),
                "speed_kmh": round(speed_kmh, 2),
                "time": timestamp
            }

        # Use GGA as backup if RMC isn't available
        if isinstance(msg, pynmea2.types.talker.GGA) and int(msg.gps_qual) > 0:
            if msg.latitude == 0.0 or msg.longitude == 0.0:
                return None
            timestamp = datetime.utcnow().strftime("%H:%M:%S")
            return {
                "latitude": round(msg.latitude, 6),
                "longitude": round(msg.longitude, 6),
                "speed_kmh": 0.0,
                "time": timestamp
            }

        return None
    except pynmea2.ParseError:
        return None
    except Exception:
        return None

# ==============================
# SEND DATA VIA LORA
# ==============================
def send_lora_message(message):
    """Send a message via LoRa, truncated to the payload limit."""
    if len(message) > payloadLength:
        message = message[:payloadLength]
    message_bytes = list(message.encode('utf-8'))
    LoRa.beginPacket()
    LoRa.write(message_bytes, len(message_bytes))
    LoRa.endPacket()
    LoRa.wait()

# ==============================
# MAIN LOOP
# ==============================
try:
    while True:
        start_time = time.time()
        gps_data = None

        # Continuously read for ~1 second to get the latest valid GPS data
        while time.time() - start_time < 1.0:
            line = gps_serial.readline().decode('ascii', errors='replace').strip()
            if line.startswith('$'):
                parsed = parse_gps_data(line)
                if parsed:
                    gps_data = parsed  # Keep the most recent valid fix

        # If valid GPS fix is available
        if gps_data:
            message = (
                f"{gps_data['time']}|LAT:{gps_data['latitude']:.6f}|"
                f"LON:{gps_data['longitude']:.6f}|SPD:{gps_data['speed_kmh']:.2f}km/h"
            )
        else:
            message = "NO GNSS FIX"

        # Send message over LoRa
        send_lora_message(message)

        # Ensure 1-second interval
        time.sleep(max(0, 1.0 - (time.time() - start_time)))

except KeyboardInterrupt:
    print("\n[INFO] Stopped by user.")
finally:
    gps_serial.close()
    LoRa.end()

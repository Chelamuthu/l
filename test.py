#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import os
import sys
import time
import serial
import pynmea2
from datetime import datetime

# ================================================
# IMPORT LORA LIBRARY
# ================================================
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# ================================================
# CONFIGURATION
# ================================================
# GPS Settings
GPS_PORT = "/dev/serial0"    # Try /dev/ttyAMA0 if this fails
GPS_BAUDRATE = 9600

# LoRa Pins
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1

# ================================================
# INITIALIZE LORA
# ================================================
print("Initializing LoRa module...")
LoRa = SX126x()
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Failed to initialize LoRa module!")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)  # 868 MHz
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)

# LoRa Modulation
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)

# LoRa Packet Parameters
payloadLength = 100
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, preambleLength=12,
                   payloadLength=payloadLength, crcType=True)
LoRa.setSyncWord(0x3444)
print("LoRa setup completed.\n")

# ================================================
# INITIALIZE GPS
# ================================================
print(f"Connecting to GPS module on {GPS_PORT} ...")
gps_serial = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=0)  # Non-blocking
print("GPS serial initialized.\n")

# ================================================
# FUNCTION: Parse GPS
# ================================================
def parse_gps_sentence(line):
    """Parse NMEA sentence and return dictionary or None."""
    try:
        if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
            msg = pynmea2.parse(line)
            if msg.status == 'A':  # Active Fix
                # Speed conversion: knots → km/h
                speed_kmh = float(msg.spd_over_grnd or 0.0) * 1.852

                # Timestamp
                if msg.datestamp and msg.timestamp:
                    timestamp = f"{msg.datestamp.strftime('%Y-%m-%d')} {msg.timestamp}"
                else:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                return {
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "speed_kmh": round(speed_kmh, 2),
                    "timestamp": timestamp
                }
    except pynmea2.ParseError:
        return None
    except Exception as e:
        print(f"[ERROR] GPS parse exception: {e}")
        return None
    return None

# ================================================
# MAIN LOOP
# ================================================
print("-- LoRa GPS Transmitter (Real-Time) --\n")
buffer = ""

try:
    while True:
        # Read available GPS data
        if gps_serial.in_waiting > 0:
            raw_data = gps_serial.read(gps_serial.in_waiting).decode('ascii', errors='replace')
            buffer += raw_data

            # Process line-by-line
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()

                if not line:
                    continue

                print(f"[GPS RAW] {line}")  # Debug raw GPS sentence

                gps_data = parse_gps_sentence(line)
                if gps_data:
                    # Build message
                    message = (
                        f"Lat:{gps_data['latitude']:.6f},"
                        f"Lon:{gps_data['longitude']:.6f},"
                        f"Speed:{gps_data['speed_kmh']}km/h,"
                        f"Time:{gps_data['timestamp']}"
                    )

                    # Trim message if too long
                    if len(message) > payloadLength:
                        message = message[:payloadLength]

                    # Convert to bytes
                    message_bytes = list(message.encode('utf-8'))

                    # Send via LoRa
                    print(f"[LORA] Sending: {message}")
                    LoRa.beginPacket()
                    LoRa.write(message_bytes, len(message_bytes))
                    LoRa.endPacket()
                    LoRa.wait()

                    print("Transmit Complete | Time: {:.2f} ms | Rate: {:.2f} byte/s\n".format(
                        LoRa.transmitTime(), LoRa.dataRate()
                    ))

        # No sleep → real-time operation

except KeyboardInterrupt:
    print("\nStopping transmitter...")

finally:
    gps_serial.close()
    LoRa.end()
    print("Closed GPS and LoRa connections safely.")

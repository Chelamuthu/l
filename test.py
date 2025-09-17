#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import os
import sys
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
GPS_PORT = "/dev/serial0"      # Change to /dev/ttyAMA0 if needed
GPS_BAUDRATE = 9600

# LoRa Pins
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1

# LoRa Frequency and Packet Size
LORA_FREQ = 868000000          # 868 MHz
PAYLOAD_LENGTH = 100           # Max size of message

# ================================================
# INITIALIZE LORA
# ================================================
print("Initializing LoRa module...")
LoRa = SX126x()
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Failed to initialize LoRa module!")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(LORA_FREQ)
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, preambleLength=12, payloadLength=PAYLOAD_LENGTH, crcType=True)
LoRa.setSyncWord(0x3444)
print("LoRa setup completed.\n")

# ================================================
# INITIALIZE GPS
# ================================================
print(f"Connecting to GPS module on {GPS_PORT} ...")
gps_serial = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=0)  # Non-blocking read
print("GPS serial initialized.\n")

# ================================================
# FUNCTION: PARSE GPS
# ================================================
def parse_gps(line):
    """
    Parses $GPRMC or $GNRMC NMEA sentence.
    Returns dict with latitude, longitude, speed, timestamp.
    """
    try:
        if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
            msg = pynmea2.parse(line)
            if msg.status == 'A':  # Active Fix only
                speed_kmh = float(msg.spd_over_grnd or 0.0) * 1.852  # knots -> km/h

                # Build timestamp
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
        print(f"[ERROR] GPS parsing failed: {e}")
        return None
    return None

# ================================================
# MAIN LOOP
# ================================================
print("-- LoRa GPS Transmitter --\n")
buffer = ""

try:
    while True:
        # Read GPS data
        if gps_serial.in_waiting > 0:
            raw_data = gps_serial.read(gps_serial.in_waiting).decode('ascii', errors='replace')
            buffer += raw_data

            # Process line-by-line
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if not line:
                    continue

                # Parse GPS line
                gps_data = parse_gps(line)
                if gps_data:
                    # Create formatted message with ONLY required fields
                    message = (
                        f"{gps_data['latitude']:.6f},"
                        f"{gps_data['longitude']:.6f},"
                        f"{gps_data['speed_kmh']}km/h,"
                        f"{gps_data['timestamp']}"
                    )

                    # Trim message if too long
                    if len(message) > PAYLOAD_LENGTH:
                        message = message[:PAYLOAD_LENGTH]

                    # Convert to bytes
                    message_bytes = list(message.encode('utf-8'))

                    # Transmit via LoRa
                    print(f"[LORA] Sending: {message}")
                    LoRa.beginPacket()
                    LoRa.write(message_bytes, len(message_bytes))
                    LoRa.endPacket()
                    LoRa.wait()

                    print("Transmit Complete | Time: {:.2f} ms | Rate: {:.2f} byte/s\n".format(
                        LoRa.transmitTime(), LoRa.dataRate()
                    ))

except KeyboardInterrupt:
    print("\nStopping transmitter...")

finally:
    gps_serial.close()
    LoRa.end()
    print("Closed GPS and LoRa connections safely.")

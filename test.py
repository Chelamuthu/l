#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import os
import sys
import time
import serial
import pynmea2
from datetime import datetime

# Import LoRa Library
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# ===============================
# CONFIGURATION
# ===============================
GPS_PORT = "/dev/serial0"    # Neo-6M GPS
GPS_BAUDRATE = 9600

# LoRa SX1262 Pins
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1

# ===============================
# Initialize LoRa
# ===============================
LoRa = SX126x()
print("Initializing LoRa module...")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Failed to initialize LoRa module!")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)  # 868 MHz
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)

# LoRa Modulation
sf = 7
bw = 125000
cr = 5
LoRa.setLoRaModulation(sf, bw, cr)

# Packet Parameters
headerType = LoRa.HEADER_EXPLICIT
preambleLength = 12
payloadLength = 100
crcType = True
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)
LoRa.setSyncWord(0x3444)

print("LoRa setup completed.\n")

# ===============================
# Initialize GPS
# ===============================
print(f"Connecting to GPS module on {GPS_PORT}")
gps_serial = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=0)  # Non-blocking

# ===============================
# Helper Function: Parse GPS
# ===============================
def get_gps_data(line):
    try:
        if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
            msg = pynmea2.parse(line)

            if msg.status == 'A':  # Active fix
                lat = msg.latitude
                lon = msg.longitude

                # Speed: knots → km/h
                speed_knots = float(msg.spd_over_grnd or 0.0)
                speed_kmh = speed_knots * 1.852

                # Timestamp
                if msg.datestamp and msg.timestamp:
                    timestamp = msg.datestamp.strftime('%Y-%m-%d') + " " + str(msg.timestamp)
                else:
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                return {
                    "latitude": lat,
                    "longitude": lon,
                    "speed_kmh": round(speed_kmh, 2),
                    "timestamp": timestamp
                }
    except pynmea2.ParseError:
        return None
    except Exception as e:
        print(f"GPS parse error: {e}")
        return None
    return None

# ===============================
# LoRa Transmit GPS Data
# ===============================
print("-- LoRa GPS Transmitter (Real-Time) --\n")

buffer = ""

try:
    while True:
        # Read GPS data non-blocking
        if gps_serial.in_waiting > 0:
            data = gps_serial.read(gps_serial.in_waiting).decode('ascii', errors='replace')
            buffer += data

            # Process full NMEA sentences
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()

                gps_data = get_gps_data(line)
                if gps_data:
                    # Build message
                    message = (
                        f"Lat:{gps_data['latitude']:.6f},"
                        f"Lon:{gps_data['longitude']:.6f},"
                        f"Speed:{gps_data['speed_kmh']}km/h,"
                        f"Time:{gps_data['timestamp']}"
                    )

                    # Ensure message fits LoRa payload
                    if len(message) > payloadLength:
                        message = message[:payloadLength]

                    # Send via LoRa
                    message_bytes = list(message.encode('utf-8'))
                    LoRa.beginPacket()
                    LoRa.write(message_bytes, len(message_bytes))
                    LoRa.endPacket()
                    LoRa.wait()  # Wait until transmission complete

                    # Debug
                    print(f"Sent GPS Data: {message}")
                    print("Transmit Time: {:.2f} ms | Data Rate: {:.2f} byte/s\n".format(
                        LoRa.transmitTime(), LoRa.dataRate()
                    ))

        # No sleep → real-time loop
except KeyboardInterrupt:
    print("Stopping transmitter...")

finally:
    gps_serial.close()
    LoRa.end()
    print("Closed GPS and LoRa connections safely.")

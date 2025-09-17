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
# Neo-6M GPS serial configuration
GPS_PORT = "/dev/serial0"   # Raspberry Pi UART0
GPS_BAUDRATE = 9600

# LoRa configuration
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

# Modulation Parameters
sf = 7
bw = 125000
cr = 5
LoRa.setLoRaModulation(sf, bw, cr)

# Packet Parameters
headerType = LoRa.HEADER_EXPLICIT
preambleLength = 12
payloadLength = 100  # max GPS packet size
crcType = True
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)

LoRa.setSyncWord(0x3444)

print("LoRa setup completed.\n")

# ===============================
# Initialize GPS
# ===============================
print("Connecting to GPS module on", GPS_PORT)
gps_serial = serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1.0)

# ===============================
# Helper Function to Read GPS
# ===============================
def get_gps_data():
    """
    Reads NMEA sentence from Neo-6M and parses GPS data.
    Returns dictionary with latitude, longitude, speed, and timestamp.
    """
    try:
        line = gps_serial.readline().decode('ascii', errors='replace').strip()
        if line.startswith('$GPRMC') or line.startswith('$GNRMC'):
            msg = pynmea2.parse(line)

            if msg.status == 'A':  # A = Active, V = Void
                lat = msg.latitude
                lon = msg.longitude

                # Speed in knots -> km/h
                try:
                    speed_knots = float(msg.spd_over_grnd)
                except:
                    speed_knots = 0.0
                speed_kmh = speed_knots * 1.852

                # Safe timestamp handling
                if msg.datestamp is not None and msg.timestamp is not None:
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
        print(f"GPS read error: {e}")
        return None
    return None

# ===============================
# LoRa Transmit GPS Data
# ===============================
print("-- LoRa GPS Transmitter --\n")

try:
    while True:
        gps_data = get_gps_data()
        if gps_data:
            # Create formatted GPS message
            message = (
                f"Lat:{gps_data['latitude']:.6f},"
                f"Lon:{gps_data['longitude']:.6f},"
                f"Speed:{gps_data['speed_kmh']}km/h,"
                f"Time:{gps_data['timestamp']}"
            )

            # Trim message if it exceeds LoRa packet limit
            if len(message) > payloadLength:
                message = message[:payloadLength]

            # Convert to byte array
            message_bytes = list(message.encode('utf-8'))

            # Send via LoRa
            LoRa.beginPacket()
            LoRa.write(message_bytes, len(message_bytes))
            LoRa.endPacket()

            # Wait until transmission finishes
            LoRa.wait()

            # Debug print
            print(f"Sent GPS Data: {message}")
            print("Transmit Time: {:.2f} ms | Data Rate: {:.2f} byte/s\n".format(
                LoRa.transmitTime(), LoRa.dataRate()
            ))

        else:
            print("Waiting for valid GPS fix...")

        time.sleep(1)  # Adjust transmission interval

except KeyboardInterrupt:
    print("Stopping transmitter...")

finally:
    gps_serial.close()
    LoRa.end()
    print("Closed GPS and LoRa connections safely.")

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
# GPS UART Config
UART_PORT = "/dev/ttyAMA0"   # Use "/dev/ttyUSB0" if connected via USB
BAUDRATE = 9600              # Default Neo-6M baud rate

# LoRa Configuration
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1
payloadLength = 100  # Max LoRa packet size

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
    print(f"[ERROR] Could not open GPS serial port {UART_PORT}: {e}")
    sys.exit(1)

# =======================================
# INITIALIZE LORA
# =======================================
print("[INFO] Initializing LoRa module...")
LoRa = SX126x()

if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Failed to initialize LoRa module!")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)  # 868 MHz for India/Europe

# 🔹 Lower power to prevent Pi shutdown
LoRa.setTxPower(14, LoRa.TX_POWER_SX1262)  # Safe power level

# LoRa Modulation Parameters
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)

# LoRa Packet Parameters
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, payloadLength, True)
LoRa.setSyncWord(0x3444)

print("[INFO] LoRa setup completed.\n")

# =======================================
# HELPER FUNCTION: PARSE GPS DATA
# =======================================
def parse_gps_data(line):
    """
    Parse NMEA sentence and safely return a dictionary
    with useful GPS data or None if invalid.
    """
    try:
        msg = pynmea2.parse(line)

        # GGA: Fix data, altitude
        if isinstance(msg, pynmea2.types.talker.GGA):
            if msg.latitude and msg.longitude:
                return {
                    "type": "GGA",
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "altitude": msg.altitude,
                    "timestamp": msg.timestamp.strftime("%H:%M:%S") if msg.timestamp else "N/A"
                }
            return None

        # RMC: Position, speed, date
        elif isinstance(msg, pynmea2.types.talker.RMC):
            if msg.status == "A":  # A = Active fix
                speed_knots = msg.spd_over_grnd if msg.spd_over_grnd is not None else 0.0
                speed_kmh = speed_knots * 1.852  # convert knots to km/h

                return {
                    "type": "RMC",
                    "latitude": msg.latitude,
                    "longitude": msg.longitude,
                    "speed_kmh": speed_kmh,
                    "date": msg.datestamp.strftime("%d-%m-%Y") if msg.datestamp else "N/A",
                    "timestamp": msg.timestamp.strftime("%H:%M:%S") if msg.timestamp else "N/A"
                }
            return None

        return None
    except pynmea2.ParseError:
        return None
    except Exception as e:
        print(f"[WARN] Parsing error: {e}")
        return None

# =======================================
# MAIN LOOP
# =======================================
print("[INFO] Starting GPS + LoRa transmission. Press CTRL+C to stop.\n")

try:
    while True:
        # Read one line of GPS data
        line = gps_serial.readline().decode('ascii', errors='replace').strip()

        if not line.startswith('$'):  # Ignore incomplete/invalid NMEA lines
            continue

        gps_data = parse_gps_data(line)

        if gps_data:
            # ==========================
            # FORMAT MESSAGE BASED ON TYPE
            # ==========================
            if gps_data["type"] == "GGA":
                message = (
                    f"GGA|Time:{gps_data['timestamp']}|"
                    f"Lat:{gps_data['latitude']:.6f}|"
                    f"Lon:{gps_data['longitude']:.6f}|"
                    f"Alt:{gps_data['altitude']}m"
                )
                print(f"[GGA] {message}")

            elif gps_data["type"] == "RMC":
                message = (
                    f"RMC|Date:{gps_data['date']}|Time:{gps_data['timestamp']}|"
                    f"Lat:{gps_data['latitude']:.6f}|"
                    f"Lon:{gps_data['longitude']:.6f}|"
                    f"Speed:{gps_data['speed_kmh']:.2f}km/h"
                )
                print(f"[RMC] {message}")

            # ==========================
            # SEND MESSAGE VIA LORA
            # ==========================
            if len(message) > payloadLength:
                message = message[:payloadLength]  # Trim to fit LoRa payload

            try:
                message_bytes = list(message.encode('utf-8'))

                LoRa.beginPacket()
                LoRa.write(message_bytes, len(message_bytes))
                LoRa.endPacket()
                LoRa.wait()

                print(f"[INFO] Sent via LoRa: {message}")
                print("Transmit Time: {:.2f} ms | Data Rate: {:.2f} byte/s\n".format(
                    LoRa.transmitTime(), LoRa.dataRate()
                ))

                # 🔹 Small delay after send to prevent power surge
                time.sleep(0.5)

            except Exception as e:
                print(f"[ERROR] Failed to send LoRa packet: {e}")

        else:
            print("[INFO] Waiting for valid GPS fix...")

        # Send data every 1 second
        time.sleep(1)

except KeyboardInterrupt:
    print("\n[INFO] Transmission stopped by user.")

finally:
    gps_serial.close()
    LoRa.end()
    print("[INFO] Closed GPS and LoRa connections safely.")

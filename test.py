#!/usr/bin/python3
# -- coding: UTF-8 --

import os
import sys
import time
import serial
import pynmea2

UART_PORT = "/dev/ttyAMA0"
BAUDRATE = 9600
payloadLength = 100
SEND_INTERVAL = 1.0
MAX_DELAY = 1.2

busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1

# Add LoRa library path
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# -------------------
# Initialize GPS
# -------------------
try:
    gps_serial = serial.Serial(UART_PORT, BAUDRATE, timeout=0.5)
    print(f"[INFO] Connected to GPS on {UART_PORT} at {BAUDRATE} baud.")
except serial.SerialException as e:
    print(f"[ERROR] Cannot open GPS port {UART_PORT}: {e}")
    sys.exit(1)

# -------------------
# Initialize LoRa
# -------------------
def init_lora():
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
    return LoRa

LoRa = init_lora()

# -------------------
# Parse GPS
# -------------------
def parse_gps_data(line):
    try:
        msg = pynmea2.parse(line)
        if isinstance(msg, pynmea2.types.talker.RMC) and msg.status == "A":
            speed_kmh = (msg.spd_over_grnd or 0.0) * 1.852
            return {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "speed_kmh": speed_kmh,
                "date": msg.datestamp.strftime("%d-%m-%Y") if msg.datestamp else "N/A",
                "time": msg.timestamp.strftime("%H:%M:%S") if msg.timestamp else "N/A"
            }
        return None
    except:
        return None

# -------------------
# MAIN LOOP
# -------------------
print("[INFO] Starting GPS + LoRa transmission...\n")
buffer = ""
last_gps_data = None
last_successful_send = time.time()

try:
    while True:
        current_time = time.time()

        # Read GPS
        data = gps_serial.read(1024).decode('ascii', errors='replace')
        if data:
            buffer += data
            lines = buffer.split('\n')
            buffer = lines[-1]
            for line in lines[:-1]:
                line = line.strip()
                if line.startswith('$'):
                    gps_data = parse_gps_data(line)
                    if gps_data:
                        last_gps_data = gps_data

        # Send message if interval passed
        if current_time - last_successful_send >= SEND_INTERVAL:
            if last_gps_data:
                message = (f"RMC|Date:{last_gps_data['date']}|Time:{last_gps_data['time']}|"
                           f"Lat:{last_gps_data['latitude']:.6f}|Lon:{last_gps_data['longitude']:.6f}|"
                           f"Speed:{last_gps_data['speed_kmh']:.2f}km/h")
            else:
                message = "No GPS Fix"

            if len(message) > payloadLength:
                message = message[:payloadLength]

            # Attempt to send with try-except
            try:
                LoRa.beginPacket()
                LoRa.write(list(message.encode('utf-8')), len(message))
                LoRa.endPacket()

                # Wait with timeout to avoid lock
                wait_start = time.time()
                while True:
                    if LoRa.transmitDone():  # Check for completion
                        break
                    if time.time() - wait_start > 0.5:  # 0.5s timeout
                        raise TimeoutError("LoRa transmit timeout")
                    time.sleep(0.01)

                print(f"[INFO] Sent via LoRa: {message}")
                last_successful_send = time.time()

            except Exception as e:
                print(f"[ERROR] LoRa send failed: {e}")

        # Auto-restart LoRa if no successful send in MAX_DELAY
        if time.time() - last_successful_send > MAX_DELAY:
            print("[WARN] LoRa sender stuck >1.2s. Restarting LoRa module...")
            try:
                LoRa.end()
            except:
                pass
            LoRa = init_lora()
            last_successful_send = time.time()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n[INFO] Transmission stopped by user.")
finally:
    gps_serial.close()
    LoRa.end()
    print("[INFO] Closed GPS and LoRa safely.")

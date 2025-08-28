import os, sys
import time
import serial
import pynmea2
from datetime import datetime
from LoRaRF import SX126x

# --- LoRa Setup ---
busId = 0; csId = 0 
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1 
LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Something wrong, can't begin LoRa radio")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 128, True)  # payloadLength increased
LoRa.setSyncWord(0x3444)

print("\n-- LoRa Transmitter with GNSS + Speed + Time + Latency --\n")

# --- GNSS Setup ---
GNSS_PORT = "/dev/ttyUSB0"   # Linux
# GNSS_PORT = "COM5"         # Windows
GNSS_BAUD = 9600

try:
    gps_serial = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
    print(f"Opened GNSS on {GNSS_PORT}")
except Exception as e:
    print("Error opening GNSS:", e)
    gps_serial = None

counter = 0
while True:
    latitude, longitude, speed_kmh = None, None, None

    if gps_serial and gps_serial.in_waiting > 0:
        line = gps_serial.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("$GPRMC"):   # RMC contains speed
            try:
                msg = pynmea2.parse(line)
                latitude = msg.latitude
                longitude = msg.longitude
                if hasattr(msg, "spd_over_grnd") and msg.spd_over_grnd:
                    speed_knots = float(msg.spd_over_grnd)
                    speed_kmh = speed_knots * 1.852   # convert knots → km/h
            except Exception as e:
                print("Parse error:", e)

        elif line.startswith("$GPGGA"):   # fallback for lat/lon only
            try:
                msg = pynmea2.parse(line)
                latitude = msg.latitude
                longitude = msg.longitude
            except:
                pass

    # --- Add current time (system clock) ---
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # --- Build message ---
    if latitude and longitude:
        if speed_kmh is not None:
            message = (f"Time:{timestamp} "
                       f"GPS:{latitude:.5f},{longitude:.5f} "
                       f"Spd:{speed_kmh:.2f}km/h "
                       f"Cnt:{counter}")
        else:
            message = (f"Time:{timestamp} "
                       f"GPS:{latitude:.5f},{longitude:.5f} "
                       f"Spd:N/A "
                       f"Cnt:{counter}")
    else:
        message = f"Time:{timestamp} No GPS Fix Cnt:{counter}"

    # Convert to bytes
    message_bytes = [ord(c) for c in message]

    # --- Measure latency ---
    t0 = time.time()
    LoRa.beginPacket()
    LoRa.write(message_bytes, len(message_bytes))
    LoRa.endPacket()
    LoRa.wait()
    t1 = time.time()
    latency_ms = (t1 - t0) * 1000

    # Print and send latency in message
    full_message = message + f" Latency:{latency_ms:.1f}ms"

    # For debugging
    print(f"Sent: {full_message}")
    print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s"
          .format(LoRa.transmitTime(), LoRa.dataRate()))

    time.sleep(5)
    counter = (counter + 1) % 256

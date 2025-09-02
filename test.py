#!/usr/bin/env python3
import time, serial
from LoRaRF import SX126x

# ---------------- GPS Setup ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.5)
gps.reset_input_buffer()

def parse_nmea(sentence):
    """Extract latitude & longitude from NMEA GGA/RMC sentences"""
    try:
        parts = sentence.split(",")
        if len(parts) < 6:
            return None
        if parts[0] in ["$GPGGA", "$GNGGA", "$GPRMC", "$GNRMC"]:
            lat_raw, lat_hemi, lon_raw, lon_hemi = parts[2], parts[3], parts[4], parts[5]
            if lat_raw and lon_raw:
                # Convert NMEA to decimal degrees
                lat = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
                lon = float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
                if lat_hemi == "S":
                    lat = -lat
                if lon_hemi == "W":
                    lon = -lon
                return lat, lon
    except Exception:
        pass
    return None

# ---------------- LoRa Setup ----------------
busId, csId = 0, 0
resetPin, busyPin, irqPin, txenPin, rxenPin = 18, 20, -1, 6, -1

LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Something wrong, can't begin LoRa radio")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)   # India ISM band
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)

# LoRa parameters
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, preambleLength=12,
                   payloadLength=255, crcType=True)
LoRa.setSyncWord(0x3444)

print("\n-- LoRa GNSS Transmitter --\n")

# ---------------- Transmit Loop ----------------
counter = 0
try:
    while True:
        line = gps.readline().decode("ascii", errors="ignore").strip()

        if line.startswith(("$GPGGA", "$GNGGA", "$GPRMC", "$GNRMC")):
            coords = parse_nmea(line)
            counter += 1
            if coords:
                lat, lon = coords
                packet = f"{counter},{lat:.6f},{lon:.6f}"
                data = packet.encode("utf-8")

                LoRa.beginPacket()
                LoRa.write(data, len(data))
                LoRa.endPacket(False)

                print(f"[TX] {packet}")
            else:
                print(f"[NO_FIX] sentence={line[:20]}...")

        time.sleep(1)  # send once per second

except KeyboardInterrupt:
    print("\nStopping GNSS transmitter...")
    LoRa.end()
    gps.close()

#!/usr/bin/env python3
"""
lora_gnss_tx.py
Read GNSS (NMEA) from UART, parse lat/lon and send over SX1262 LoRa.
Designed for Waveshare SX1262 expansion board on Raspberry Pi.
"""
import time
import serial
import os
from LoRaRF import SX126x

# --- GNSS serial auto-detect ---
POSSIBLE_PORTS = ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0", "/dev/ttyUSB0"]
gps = None
for p in POSSIBLE_PORTS:
    try:
        gps = serial.Serial(p, baudrate=9600, timeout=1)
        print(f"[GPS] opened {p}")
        break
    except Exception:
        pass
if gps is None:
    raise SystemExit("ERROR: Could not open any GPS serial port. Check wiring and enable UART.")

# --- Utility: Convert NMEA ddmm.mmmm to decimal degrees ---
def nmea_to_decimal(raw, hemi, is_lat=True):
    if not raw:
        return None
    try:
        # latitude: ddmm.mmmm (deg_len=2), longitude: dddmm.mmmm (deg_len=3)
        deg_len = 2 if is_lat else 3
        deg = float(raw[:deg_len])
        minutes = float(raw[deg_len:])
        dec = deg + (minutes / 60.0)
        if hemi in ("S", "W"):
            dec = -dec
        return dec
    except Exception:
        return None

# --- Parse NMEA lines ($GPRMC or $GPGGA) and return (lat, lon) as floats or (None, None) ---
def parse_nmea(line):
    if not line:
        return None, None
    line = line.strip()
    parts = line.split(",")
    try:
        if line.startswith("$GPRMC"):
            # $GPRMC,hhmmss,A,llll.ll,a,yyyyy.yy,a,...  (A = valid)
            if len(parts) > 6 and parts[2] == "A":
                lat_raw, lat_dir = parts[3], parts[4]
                lon_raw, lon_dir = parts[5], parts[6]
                lat = nmea_to_decimal(lat_raw, lat_dir, is_lat=True)
                lon = nmea_to_decimal(lon_raw, lon_dir, is_lat=False)
                return lat, lon
        elif line.startswith("$GPGGA") or line.startswith("$GNGGA"):
            # $GPGGA,hhmmss,llll.ll,a,yyyyy.yy,a,fix,...  (fix > 0 means valid)
            if len(parts) > 6 and parts[6] and int(parts[6]) > 0:
                lat_raw, lat_dir = parts[2], parts[3]
                lon_raw, lon_dir = parts[4], parts[5]
                lat = nmea_to_decimal(lat_raw, lat_dir, is_lat=True)
                lon = nmea_to_decimal(lon_raw, lon_dir, is_lat=False)
                return lat, lon
    except Exception:
        pass
    return None, None

# --- LoRa initialization ---
busId = 0; csId = 0
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1

LoRa = SX126x()
print("[LoRa] starting...")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise SystemExit("ERROR: LoRa.begin() failed")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)  # Set correct band for your country (India example)
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(7, 125000, 5)   # SF7, BW=125kHz, CR=4/5

# Use dynamic payload length (0) to avoid length mismatch hangs
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, preambleLength=12, payloadLength=0, crcType=True)
LoRa.setSyncWord(0x3444)

print("[LoRa] ready. Sending GNSS coordinates when available.\nPress Ctrl+C to stop.")

counter = 0
try:
    while True:
        try:
            raw_line = gps.readline().decode("ascii", errors="replace").strip()
        except Exception:
            raw_line = None

        lat, lon = parse_nmea(raw_line) if raw_line else (None, None)
        if lat is not None and lon is not None:
            # Compose small ASCII message: "lat,lon"
            msg = f"{lat:.6f},{lon:.6f}"
            payload = msg.encode("ascii", errors="replace")
            data_list = list(payload)  # LoRa.write expects list of ints in many libs

            try:
                LoRa.beginPacket()
                LoRa.write(data_list, len(data_list))
                LoRa.endPacket()

                # Wait for TX done — try to use timeout-aware wait if available
                tx_ok = True
                try:
                    # many LoRaRF wrappers accept LoRa.wait(ms) or LoRa.wait(timeout=ms)
                    tx_ok = LoRa.wait(2000)  # try with 2s timeout
                except TypeError:
                    try:
                        tx_ok = LoRa.wait(timeout=2000)
                    except Exception:
                        # unknown signature: fallback to small sleep
                        time.sleep(0.1)
                        tx_ok = True
                if not tx_ok:
                    print("[WARN] TX wait timed out -> resetting LoRa")
                    LoRa.end()
                    time.sleep(0.5)
                    LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin)

                # Optionally print TX metrics if available
                try:
                    t_ms = LoRa.transmitTime()
                    rate = LoRa.dataRate()
                    print(f"[SENT] {msg} | TX time {t_ms:.2f} ms | rate {rate:.2f} B/s")
                except Exception:
                    print(f"[SENT] {msg}")
            except Exception as e:
                print("[ERROR] Sending packet:", e)
                # try to recover LoRa if it raises
                try:
                    LoRa.end()
                except Exception:
                    pass
                time.sleep(1)
                try:
                    LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin)
                except Exception:
                    pass

            counter = (counter + 1) % 256
            time.sleep(3)   # throttle transmissions
        else:
            # No valid GNSS fix yet
            print("[GPS] waiting for fix...")
            time.sleep(1)

except KeyboardInterrupt:
    print("\nInterrupted by user, shutting down...")

finally:
    try:
        LoRa.end()
    except Exception:
        pass
    try:
        gps.close()
    except Exception:
        pass
    print("Stopped.")

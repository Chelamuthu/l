#!/usr/bin/env python3
import time, serial
import RPi.GPIO as GPIO
from LoRaRF import SX126x

# ---------------- GPIO FIX ----------------
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# ---------------- GPS SETUP ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

def nmea_to_decimal(raw, hemi, is_lat=True):
    """Convert NMEA raw value to decimal degrees"""
    try:
        deg_len = 2 if is_lat else 3
        deg = float(raw[:deg_len])
        minutes = float(raw[deg_len:])
        val = deg + (minutes / 60.0)
        if hemi in ("S", "W"):
            val = -val
        return val
    except Exception:
        return None

def parse_nmea(line):
    """Parse NMEA line for lat/lon/speed"""
    if not line:
        return None, None, None
    parts = line.split(",")
    try:
        if line.startswith("$GPRMC") and len(parts) > 7 and parts[2] == "A":
            lat = nmea_to_decimal(parts[3], parts[4], True)
            lon = nmea_to_decimal(parts[5], parts[1], False)
            speed_knots = float(parts[6]) if parts[6] else 0.0
            return lat, lon, speed_knots * 1.852  # km/h
        elif (line.startswith("$GPGGA") or line.startswith("$GNGGA")) and len(parts) > 6:
            if parts[1] and int(parts[1]) > 0:
                lat = nmea_to_decimal(parts[2], parts[3], True)
                lon = nmea_to_decimal(parts[4], parts[5], False)
                return lat, lon, None
    except Exception:
        pass
    return None, None, None

# ---------------- LORA SETUP ----------------
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1

LoRa = SX126x()

def init_lora():
    """Initialize LoRa with parameters"""
    if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
        raise SystemExit("LoRa init failed")
    LoRa.setDio2RfSwitch()
    LoRa.setFrequency(865000000)                  # Match RX frequency
    LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)     # 22 dBm
    LoRa.setLoRaModulation(7, 125000, 5)          # SF7, BW125, CR4/5
    LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 0, True)
    LoRa.setSyncWord(0x3444)                      # Must match RX sync word

def hard_reset_lora():
    """Pulse reset pin to recover instantly (<200 ms)"""
    GPIO.setup(resetPin, GPIO.OUT)
    GPIO.output(resetPin, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(resetPin, GPIO.HIGH)
    time.sleep(0.05)
    init_lora()
    print("[RESET] Hard reset done in <0.2s")

# Init once
init_lora()
print("[LoRa] ready, press Ctrl+C to stop.")

# ---------------- MAIN LOOP ----------------
last_send = time.time()
counter = 0
try:
    while True:
        # ---- Read GPS ----
        try:
            raw_line = gps.readline().decode("ascii", "ignore").strip()
            lat, lon, speed = parse_nmea(raw_line)
        except Exception:
            lat, lon, speed = None, None, None
        if gps.in_waiting > 1000:
            gps.reset_input_buffer()
        now = time.time()
        timestamp = time.strftime("%H:%M:%S", time.localtime(now))
        # ---- Build message ----
        send_data = False
        if lat is not None and lon is not None:
            counter += 1
            msg = f"{counter},{timestamp},{lat:.6f},{lon:.6f},{(speed or 0):.1f}km/h"
            last_send = now
            send_data = True
        elif now - last_send > 10:  # no fix >10s → still send
            counter += 1
            msg = f"{counter},{timestamp},NO_FIX"
            last_send = now
            send_data = True
        if not send_data:
            time.sleep(0.01)
            continue

        data = list(msg.encode())
        # ---- Send LoRa instantly ----
        try:
            LoRa.beginPacket()
            LoRa.write(data, len(data))
            LoRa.endPacket(False)
            if not LoRa.wait(10):  # Only wait 10 ms max
                print("[WARN] TX failed → instant HARD RESET")
                hard_reset_lora()
                continue  # Don't wait further, retry next loop
            try:
                tx_time = LoRa.transmitTime()
                rate = LoRa.dataRate()
                print(f"[SENT] {msg} | {len(data)}B | TX {tx_time:.1f} ms | {rate:.2f} B/s")
            except Exception:
                print(f"[SENT] {msg} | {len(data)}B")
        except Exception as e:
            print("[ERROR] send failed:", e)
            hard_reset_lora()
            continue  # Skip further waits when failed
        # ---- Keep precise 1s interval ----
        while time.time() - last_send < 1.0:
            time.sleep(0.01)
except KeyboardInterrupt:
    print("Stopped by user")
    LoRa.end()
    gps.close()
    GPIO.cleanup()

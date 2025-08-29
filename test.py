#!/usr/bin/env python3
import time, serial
import RPi.GPIO as GPIO
from LoRaRF import SX126x

# ---------------- GPIO FIX ----------------
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# ---------------- GPS SETUP ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.1)

def nmea_to_decimal(raw, hemi, is_lat=True):
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
    if not line:
        return None, None, None
    parts = line.split(",")
    try:
        if line.startswith("$GPRMC") and len(parts) > 7 and parts[2] == "A":
            lat = nmea_to_decimal(parts[3], parts[4], True)
            lon = nmea_to_decimal(parts[5], parts[6], False)
            speed_knots = float(parts[7]) if parts[7] else 0.0
            return lat, lon, speed_knots * 1.852
        elif (line.startswith("$GPGGA") or line.startswith("$GNGGA")) and len(parts) > 6:
            if parts[6] and int(parts[6]) > 0:
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
    if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
        raise SystemExit("LoRa init failed")
    LoRa.setDio2RfSwitch()
    LoRa.setFrequency(865000000)
    LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
    LoRa.setLoRaModulation(7, 125000, 5)  # SF7, BW125, CR4/5
    LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 0, True)
    LoRa.setSyncWord(0x3444)

def hard_reset_lora():
    GPIO.setup(resetPin, GPIO.OUT)
    GPIO.output(resetPin, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(resetPin, GPIO.HIGH)
    time.sleep(0.05)
    init_lora()
    print("[RESET] Hard reset done")

init_lora()
print("[LoRa] ready, press Ctrl+C to stop.")

# ---------------- MAIN LOOP ----------------
last_send = time.time()
counter = 0

try:
    while True:
        # keep 1s interval
        now = time.time()
        sleep_time = last_send + 1.0 - now
        if sleep_time > 0:
            time.sleep(sleep_time)
        now = time.time()

        # ---- GPS ----
        try:
            raw_line = gps.readline().decode("ascii", "ignore").strip()
            lat, lon, speed = parse_nmea(raw_line)
        except Exception:
            lat, lon, speed = None, None, None
        if gps.in_waiting > 1000:
            gps.reset_input_buffer()
        timestamp = time.strftime("%H:%M:%S", time.localtime(now))

        # ---- Build message ----
        counter += 1
        if lat is not None and lon is not None:
            msg = f"{counter},{timestamp},{lat:.6f},{lon:.6f},{(speed or 0):.1f}km/h"
        else:
            msg = f"{counter},{timestamp},NO_FIX"

        data = list(msg.encode())

        # ---- Send LoRa ----
        try:
            LoRa.beginPacket()
            LoRa.write(data, len(data))
            LoRa.endPacket(False)

            # expected TX time (ms)
            est_ms = int(LoRa.transmitTime() * 1000) + 20
            if est_ms < 30:
                est_ms = 30  # minimum safe

            if not LoRa.wait(est_ms):
                print(f"[WARN] TX timeout (> {est_ms} ms) → reset")
                hard_reset_lora()
                continue

            try:
                tx_time = LoRa.transmitTime()
                rate = LoRa.dataRate()
                print(f"[SENT] {msg} | {len(data)}B | TX {tx_time:.1f} ms | {rate:.2f} B/s")
            except Exception:
                print(f"[SENT] {msg} | {len(data)}B")

        except Exception as e:
            print("[ERROR] send failed:", e)
            hard_reset_lora()
            continue

        last_send = now

except KeyboardInterrupt:
    print("Stopped by user")
    LoRa.end()
    gps.close()
    GPIO.cleanup()

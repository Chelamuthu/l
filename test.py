#!/usr/bin/env python3
import time, serial
import RPi.GPIO as GPIO
from LoRaRF import SX126x

# ---------------- GPIO ----------------
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# ---------------- GPS -----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.1)
gps.reset_input_buffer()

def nmea_to_decimal(raw, hemi, is_lat=True):
    try:
        if not raw:
            return None
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
    if not line or "," not in line:
        return None, None, None
    parts = line.split(",")
    try:
        # RMC gives position + speed
        if line.startswith("$GPRMC") or line.startswith("$GNRMC"):
            if len(parts) > 7 and parts[2] == "A":
                lat = nmea_to_decimal(parts[3], parts[4], True)
                lon = nmea_to_decimal(parts[5], parts[6], False)
                sp_kn = float(parts[7]) if parts[7] else 0.0
                return lat, lon, sp_kn * 1.852
        # GGA gives position only
        if (line.startswith("$GPGGA") or line.startswith("$GNGGA")) and len(parts) > 6:
            fix = int(parts[6]) if parts[6].isdigit() else 0
            if fix > 0:
                lat = nmea_to_decimal(parts[2], parts[3], True)
                lon = nmea_to_decimal(parts[4], parts[5], False)
                return lat, lon, None
    except Exception:
        pass
    return None, None, None

def get_gps_sample(window_ms=300):
    """Read as many NMEA lines as arrive within window_ms ms and return freshest valid fix."""
    deadline = time.time() + (window_ms / 1000.0)
    best = (None, None, None)
    while time.time() < deadline:
        try:
            line = gps.readline().decode("ascii", "ignore").strip()
        except Exception:
            line = ""
        if line:
            lat, lon, spd = parse_nmea(line)
            if lat is not None and lon is not None:
                best = (lat, lon, spd if spd is not None else best[2])
        if gps.in_waiting > 2000:  # avoid backlog
            gps.reset_input_buffer()
        time.sleep(0.005)
    return best

# ---------------- LORA -----------------
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
    LoRa.setFrequency(865000000)           # adjust to region
    LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
    LoRa.setLoRaModulation(7, 125000, 5)   # SF7, BW125, CR4/5
    LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 0, True)
    LoRa.setSyncWord(0x3444)

def soft_reset_lora():
    try:
        LoRa.reset()
    except Exception:
        pass
    init_lora()
    print("[RESET] Soft reset")

def hard_reset_lora():
    GPIO.setup(resetPin, GPIO.OUT)
    GPIO.output(resetPin, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(resetPin, GPIO.HIGH)
    time.sleep(0.05)
    init_lora()
    print("[RESET] Hard reset")

init_lora()
print("[LoRa] ready, press Ctrl+C to stop.")

# ---------------- MAIN LOOP ----------------
period = 1.0
next_tick = time.time()
counter = 0
fail_count = 0

try:
    while True:
        # keep exact 1 Hz
        now = time.time()
        if now < next_tick:
            time.sleep(next_tick - now)
        tick_time = next_tick
        next_tick += period

        # ---- GPS ----
        lat, lon, speed = get_gps_sample(window_ms=300)
        timestamp = time.strftime("%H:%M:%S", time.localtime(tick_time))

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
            LoRa.endPacket(False)  # non-blocking

            # watchdog: expected airtime + margin
            est_ms = int(LoRa.transmitTime() * 1000) + 50
            if est_ms < 40:
                est_ms = 40

            if not LoRa.wait(est_ms):
                fail_count += 1
                print(f"[WARN] TX timeout (> {est_ms} ms)")
                if fail_count >= 3:
                    hard_reset_lora()
                    fail_count = 0
                else:
                    soft_reset_lora()
                continue
            else:
                fail_count = 0

            try:
                tx_time = LoRa.transmitTime()
                rate = LoRa.dataRate()
                print(f"[SENT] {msg} | {len(data)}B | TX {tx_time:.1f} ms | {rate:.2f} B/s")
            except Exception:
                print(f"[SENT] {msg} | {len(data)}B")

        except Exception as e:
            print("[ERROR] TX failed:", e)
            hard_reset_lora()
            continue

except KeyboardInterrupt:
    print("Stopped by user")
    LoRa.end()
    gps.close()
    GPIO.cleanup()

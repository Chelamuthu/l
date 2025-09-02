#!/usr/bin/env python3
import time, serial
import RPi.GPIO as GPIO
from LoRaRF import SX126x

# ---------------- GPIO ----------------
GPIO.setwarnings(False)
GPIO.cleanup()          # clear any old state
GPIO.setmode(GPIO.BCM)

# ---------------- GPS -----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.2)
gps.reset_input_buffer()

def nmea_to_decimal(raw: str, hemi: str, is_lat=True):
    try:
        if not raw:
            return None
        if is_lat:
            deg = int(raw[:2]); mins = float(raw[2:])
        else:
            deg = int(raw[:3]); mins = float(raw[3:])
        val = deg + mins/60.0
        if hemi in ("S", "W"):
            val = -val
        return val
    except Exception:
        return None

def parse_nmea(line: str):
    """
    Return (lat, lon) if sentence has a valid fix; otherwise (None, None).
    Supports GGA (fix>0) and RMC (status 'A'). Accepts GPS+GNSS variants.
    """
    if not line or line[0] != "$":
        return None, None
    parts = line.split(",")
    tag = parts[0]

    try:
        # --- RMC ---
        if tag in ("$GPRMC", "$GNRMC") and len(parts) > 6:
            status = parts[2]
            if status == "A":  # valid
                lat = nmea_to_decimal(parts[3], parts[4], True)
                lon = nmea_to_decimal(parts[5], parts[6], False)
                return lat, lon

        # --- GGA ---
        if tag in ("$GPGGA", "$GNGGA") and len(parts) > 6:
            fix_q = parts[6]
            if fix_q and int(fix_q) > 0:
                lat = nmea_to_decimal(parts[2], parts[3], True)
                lon = nmea_to_decimal(parts[4], parts[5], False)
                return lat, lon
    except Exception:
        pass

    return None, None

# --------------- LoRa -----------------
busId, csId = 0, 0
resetPin, busyPin, irqPin, txenPin, rxenPin = 18, 20, -1, 6, -1

LoRa = SX126x()

def init_lora():
    if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
        raise SystemExit("LoRa init failed")
    LoRa.setDio2RfSwitch()
    LoRa.setFrequency(865000000)                       # set your region
    LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)          # SX1262 PA
    LoRa.setLoRaModulation(7, 125000, 5)               # SF7, BW125, CR4/5
    LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 0, True)  # CRC on
    LoRa.setSyncWord(0x3444)                           # must match receiver

def hard_reset_lora():
    GPIO.setup(resetPin, GPIO.OUT)
    GPIO.output(resetPin, GPIO.LOW);  time.sleep(0.01)
    GPIO.output(resetPin, GPIO.HIGH); time.sleep(0.05)
    init_lora()
    print("[RESET] LoRa hard reset")

init_lora()
print("[LoRa] ready, press Ctrl+C to stop.")

# ------------- Transmit loop -----------
counter = 0
last_fix = (None, None)
last_fix_time = 0.0
period = 1.0
next_send = time.time()

try:
    while True:
        # keep precise 1 Hz
        now = time.time()
        if now < next_send:
            time.sleep(next_send - now)
        now = next_send
        next_send += period

        # read a few NMEA lines quickly to catch the latest fix
        lat, lon = None, None
        for _ in range(15):  # ~ a few ms total (timeout=0.2 on port)
            line = gps.readline().decode("ascii", "ignore").strip()
            if line:
                lat, lon = parse_nmea(line)
                if lat is not None and lon is not None:
                    last_fix = (lat, lon)
                    last_fix_time = time.time()
                    break

        # build payload
        counter += 1
        ts = time.strftime("%H:%M:%S", time.localtime(now))
        if last_fix[0] is not None and (now - last_fix_time) < 5.0:
            msg = f"{counter},{ts},{last_fix[0]:.6f},{last_fix[1]:.6f}"
        else:
            msg = f"{counter},{ts},NO_FIX"

        data = msg.encode("utf-8")

        # send
        try:
            LoRa.beginPacket()
            LoRa.write(data, len(data))
            LoRa.endPacket(False)  # non-blocking

            # wait briefly for TX-done; fallback to quick reset if stuck
            # For a short payload at SF7/BW125 this should finish well < 200 ms.
            if not LoRa.wait(250):  # milliseconds
                print("[WARN] TX timeout → hard reset")
                hard_reset_lora()
                continue

            # metrics (best-effort)
            try:
                tx_time = LoRa.transmitTime()
                rate = LoRa.dataRate()
                print(f"[SENT] {msg} | {len(data)}B | TX {tx_time:.1f} ms | {rate:.1f} B/s")
            except Exception:
                print(f"[SENT] {msg}")

        except Exception as e:
            print("[ERROR] LoRa send failed:", e)
            hard_reset_lora()
            continue

except KeyboardInterrupt:
    print("Stopping …")
    LoRa.end()
    gps.close()
    GPIO.cleanup()

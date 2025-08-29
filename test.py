#!/usr/bin/env python3
import time, serial
import RPi.GPIO as GPIO
from LoRaRF import SX126x

# ---------------- GPIO ----------------
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# ---------------- GPS -----------------
# Short timeout so we can poll multiple lines per second
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
    """
    Returns (lat, lon, speed_kmh) or (lat, lon, None) or (None, None, None)
    Accepts GPRMC for speed + position, GPGGA/GNGGA for position only.
    """
    if not line or "," not in line:
        return None, None, None
    parts = line.split(",")
    try:
        # RMC: $GxRMC,hhmmss.sss,A,llll.ll,a,yyyyy.yy,a,x.x,...
        if line.startswith("$GPRMC") or line.startswith("$GNRMC"):
            if len(parts) > 7 and parts[2] == "A":
                lat = nmea_to_decimal(parts[3], parts[4], True)
                lon = nmea_to_decimal(parts[5], parts[6], False)
                sp_kn = float(parts[7]) if parts[7] else 0.0
                return lat, lon, sp_kn * 1.852
        # GGA: $GxGGA,hhmmss.sss,llll.ll,a,yyyyy.yy,a,fix,...
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
    """
    Read as many NMEA lines as arrive within window_ms and return
    the freshest valid (lat, lon, speed_kmh). This avoids partial/old lines.
    """
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
        # trim backlog so latency stays low
        if gps.in_waiting > 2000:
            gps.reset_input_buffer()
        # tiny breather to share CPU
        time.sleep(0.005)
    return best

# ---------------- LoRa -----------------
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
    LoRa.setFrequency(865000000)           # adjust to your region/receiver
    LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
    LoRa.setLoRaModulation(7, 125000, 5)   # SF7, 125 kHz, CR 4/5
    LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 0, True)  # CRC ON
    LoRa.setSyncWord(0x3444)

def soft_reset_lora():
    # Fast recovery without GPIO pulse
    try:
        LoRa.reset()
    except Exception:
        pass
    init_lora()
    print("[RESET] Soft reset")

def hard_reset_lora():
    # GPIO pulse reset — still sub-200 ms
    GPIO.setup(resetPin, GPIO.OUT)
    GPIO.output(resetPin, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(resetPin, GPIO.HIGH)
    time.sleep(0.05)
    init_lora()
    print("[RESET] Hard reset")

init_lora()
print("[LoRa] ready, press Ctrl+C to stop.")

# ---------------- MAIN LOOP -------------
period = 1.0          # one packet per second
next_tick = time.time()
counter = 0
consecutive_tx_fail = 0

try:
    while True:
        # precise 1 Hz pacing
        now = time.time()
        if now < next_tick:
            time.sleep(next_tick - now)
        tick_time = next_tick
        next_tick += period

        # --- get fresh GPS within ~300 ms window ---
        lat, lon, speed = get_gps_sample(window_ms=300)
        timestamp = time.strftime("%H:%M:%S", time.localtime(tick_time))

        # --- build message (always send every second) ---
        counter += 1
        if lat is not None and lon is not None:
            msg = f"{counter},{timestamp},{lat:.6f},{lon:.6f},{(speed or 0):.1f}km/h"
        else:
            msg = f"{counter},{timestamp},NO_FIX"

        payload = msg.encode()
        data = list(payload)

        # --- transmit with millisecond watchdog ---
        try:
            LoRa.beginPacket()
            LoRa.write(data, len(data))
            LoRa.endPacket(False)  # non-blocking

            # Tight watchdog loop: up to ~300 ms at SF7 small payloads
            done = False
            t0 = time.time()
            # First a few very short polls (fast path)
            for _ in range(10):  # ~10 * 5 ms = 50 ms
                if LoRa.wait(5):
                    done = True
                    break
                time.sleep(0.005)

            # If not done, extend up to 300 ms total, still millisecond-level
            while not done and (time.time() - t0) < 0.300:
                if LoRa.wait(10):
                    done = True
                    break
                time.sleep(0.005)

            if not done:
                consecutive_tx_fail += 1
                print("[WARN] TX watchdog (300 ms) → soft reset")
                soft_reset_lora()
                # after reset, continue loop; keep 1 Hz schedule
                continue
            else:
                consecutive_tx_fail = 0

            # metrics (guarded)
            try:
                tx_time = LoRa.transmitTime()
                rate = LoRa.dataRate()
                print(f"[SENT] {msg} | {len(data)}B | TX {tx_time:.1f} ms | {rate:.2f} B/s")
            except Exception:
                print(f"[SENT] {msg} | {len(data)}B")

        except Exception as e:
            print("[ERROR] TX exception:", e)
            consecutive_tx_fail += 1
            soft_reset_lora()
            continue

        # escalate if radio is truly wedged
        if consecutive_tx_fail >= 3:
            print("[ERROR] 3 consecutive TX fails → HARD RESET")
            hard_reset_lora()
            consecutive_tx_fail = 0

except KeyboardInterrupt:
    print("Stopped by user")
    LoRa.end()
    gps.close()
    GPIO.cleanup()

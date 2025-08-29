#!/usr/bin/env python3
import time, serial
from LoRaRF import SX126x

# ---------------- GPS SETUP ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

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
    """Return (lat, lon, speed_kmh) from NMEA if valid"""
    if not line: 
        return None, None, None
    parts = line.split(",")
    try:
        if line.startswith("$GPRMC") and len(parts) > 7 and parts[2] == "A":
            lat = nmea_to_decimal(parts[3], parts[4], True)
            lon = nmea_to_decimal(parts[5], parts[6], False)
            speed_knots = float(parts[7]) if parts[7] else 0.0
            speed_kmh = speed_knots * 1.852
            return lat, lon, speed_kmh
        elif (line.startswith("$GPGGA") or line.startswith("$GNGGA")) and len(parts) > 6:
            if parts[6] and int(parts[6]) > 0:
                lat = nmea_to_decimal(parts[2], parts[3], True)
                lon = nmea_to_decimal(parts[4], parts[5], False)
                return lat, lon, None   # no speed in GGA
    except Exception:
        pass
    return None, None, None

# ---------------- LORA SETUP ----------------
busId=0; csId=0; resetPin=18; busyPin=20; irqPin=-1; txenPin=6; rxenPin=-1
LoRa = SX126x()
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise SystemExit("LoRa init failed")
LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)      # adjust for your band
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(7,125000,5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 0, True)
LoRa.setSyncWord(0x3444)

print("[LoRa] ready, press Ctrl+C to stop.")

# ---------------- MAIN LOOP ----------------
last_send = time.time()
counter = 0

try:
    while True:
        try:
            raw_line = gps.readline().decode("ascii","ignore").strip()
            lat, lon, speed = parse_nmea(raw_line)
        except Exception:
            lat, lon, speed = None, None, None

        now = time.time()
        timestamp = time.strftime("%H:%M:%S", time.localtime(now))

        if lat is not None and lon is not None:
            counter += 1
            speed_kmh = speed if speed is not None else 0.0
            msg = f"{counter},{timestamp},{lat:.6f},{lon:.6f},{speed_kmh:.1f}km/h"
            last_send = now
        elif now - last_send > 10:  # no fix for 10s → still send keepalive
            counter += 1
            msg = f"{counter},{timestamp},NO_FIX"
            last_send = now
        else:
            continue   # don’t spam too fast when no fix

        data = list(msg.encode())
        try:
            LoRa.beginPacket()
            LoRa.write(data, len(data))
            LoRa.endPacket()

            # protect against hang
            ok = False
            t0 = time.time()
            while time.time() - t0 < 2:   # max 2s wait
                try:
                    if LoRa.wait(100):   # small timeout step
                        ok = True
                        break
                except Exception:
                    time.sleep(0.05)

            if not ok:
                print("[WARN] TX timeout → resetting LoRa")
                LoRa.end()
                time.sleep(0.5)
                LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin)
                LoRa.setDio2RfSwitch()
                LoRa.setFrequency(865000000)
                LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
                LoRa.setLoRaModulation(7,125000,5)
                LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT,12,0,True)
                LoRa.setSyncWord(0x3444)

            # --- Print metrics ---
            try:
                tx_time = LoRa.transmitTime()   # ms
                rate = LoRa.dataRate()          # B/s
                print(f"[SENT] {msg} | {len(data)}B | TX {tx_time:.1f} ms | {rate:.2f} B/s")
            except Exception:
                print(f"[SENT] {msg} | {len(data)}B")

        except Exception as e:
            print("[ERROR] send failed:", e)
            time.sleep(1)

        time.sleep(1)

except KeyboardInterrupt:
    print("Stopped by user")
    LoRa.end()
    gps.close()

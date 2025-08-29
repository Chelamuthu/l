#!/usr/bin/env python3
import time, serial
from LoRaRF import SX126x

# ---------------- GPS SETUP ----------------
gps = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

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
    if not line: return None, None
    parts = line.split(",")
    try:
        if line.startswith("$GPRMC") and len(parts) > 6 and parts[2] == "A":
            lat = nmea_to_decimal(parts[3], parts[4], True)
            lon = nmea_to_decimal(parts[5], parts[6], False)
            return lat, lon
        elif (line.startswith("$GPGGA") or line.startswith("$GNGGA")) and len(parts) > 6:
            if parts[6] and int(parts[6]) > 0:
                lat = nmea_to_decimal(parts[2], parts[3], True)
                lon = nmea_to_decimal(parts[4], parts[5], False)
                return lat, lon
    except Exception:
        pass
    return None, None

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
            lat, lon = parse_nmea(raw_line)
        except Exception:
            lat, lon = None, None

        now = time.time()
        if lat is not None and lon is not None:
            msg = f"{lat:.6f},{lon:.6f}"
            last_send = now
        elif now - last_send > 10:  # no fix for 10s → still send keepalive
            msg = "NO_FIX"
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

            print(f"[SENT] {msg}")
            counter += 1
        except Exception as e:
            print("[ERROR] send failed:", e)
            time.sleep(1)

        time.sleep(1)

except KeyboardInterrupt:
    print("Stopped by user")
    LoRa.end()
    gps.close()

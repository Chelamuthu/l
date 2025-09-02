import os, sys
import time
import serial
from LoRaRF import SX126x

# ---------- LoRa setup ----------
busId = 0; csId = 0
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1
LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Something wrong, can't begin LoRa radio")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)                 # 865–867 MHz (India)
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 128, True)  # payload bigger for richer text
LoRa.setSyncWord(0x3444)

print("\n-- LoRa Transmitter (GNSS: multi-talker RMC/GGA) --\n")

# ---------- GNSS setup ----------
GNSS_PORT = "/dev/ttyAMA0"       # RPi UART
# GNSS_PORT = "COM5"             # Windows example
GNSS_BAUD = 9600

try:
    gps_serial = serial.Serial(GNSS_PORT, GNSS_BAUD, timeout=1)
    print(f"Opened GNSS on {GNSS_PORT}")
except Exception as e:
    print("Error opening GNSS:", e)
    gps_serial = None

# ---------- Helpers ----------
def convert_lat_lon(lat_str, ns, lon_str, ew):
    """NMEA ddmm.mmmm / dddmm.mmmm -> decimal degrees. Returns (None, None) if invalid."""
    if not lat_str or not lon_str or ns not in ("N","S") or ew not in ("E","W"):
        return None, None
    try:
        # latitude: 2 deg digits
        lat_deg = float(lat_str[:2])
        lat_min = float(lat_str[2:])
        lat = lat_deg + lat_min/60.0
        if ns == "S": lat = -lat
        # longitude: 3 deg digits
        lon_deg = float(lon_str[:3])
        lon_min = float(lon_str[3:])
        lon = lon_deg + lon_min/60.0
        if ew == "W": lon = -lon
        return lat, lon
    except:
        return None, None

def format_utc(hhmmss):
    """hhmmss(.sss) -> hh:mm:ss UTC"""
    if not hhmmss or len(hhmmss) < 6: return "N/A"
    try:
        hh, mm, ss = hhmmss[0:2], hhmmss[2:4], hhmmss[4:6]
        return f"{hh}:{mm}:{ss} UTC"
    except:
        return "N/A"

def is_rmc(sentence):
    # Any talker: $GNRMC, $GPRMC, $GARMC, $BDRMC, ...
    return sentence.startswith("$") and sentence[3:6] == "RMC"

def is_gga(sentence):
    # Any talker: $GNGGA, $GPGGA, $GAGGA, $BDGGA, ...
    return sentence.startswith("$") and sentence[3:6] == "GGA"

counter = 0
last_fix = None   # keep last valid fix to show movement smoothly (optional)

while True:
    latitude = longitude = speed_kmh = course_deg = None
    utc_time = None
    sats_used = hdop = altitude_m = None

    try:
        line = gps_serial.readline().decode("utf-8", errors="ignore").strip() if gps_serial else ""
        if line.startswith("$"):
            print("NMEA:", line)  # keep for debugging; comment out if too chatty

        # -------- RMC: time, status, lat, lon, speed(knots), course, date --------
        if is_rmc(line):
            parts = line.split(",")
            # Expected fields per RMC
            #  1: UTC time, 2: Status (A/V), 3-6: lat,NS,lon,EW, 7: speed(knots), 8: course, 9: date
            if len(parts) >= 10 and parts[2] == "A":   # 'A' = valid fix
                utc_time = format_utc(parts[1])
                lat, lon = convert_lat_lon(parts[3], parts[4], parts[5], parts[6])
                if lat is not None and lon is not None:
                    latitude, longitude = lat, lon
                # speed
                try:
                    if parts[7] != "": speed_kmh = float(parts[7]) * 1.852
                except: pass
                # course over ground
                try:
                    if parts[8] != "": course_deg = float(parts[8])
                except: pass

        # -------- GGA: time, lat/lon, fix quality, sats, HDOP, altitude --------
        elif is_gga(line):
            parts = line.split(",")
            # 1: time, 2-5: lat,NS,lon,EW, 6: fix quality (0=no fix), 7:sats, 8:HDOP, 9:alt(m)
            if len(parts) >= 10 and parts[6] not in ("", "0"):
                utc_time = format_utc(parts[1]) if not utc_time else utc_time
                lat, lon = convert_lat_lon(parts[2], parts[3], parts[4], parts[5])
                if lat is not None and lon is not None:
                    latitude, longitude = lat, lon
                try:
                    if parts[7] != "": sats_used = int(parts[7])
                except: pass
                try:
                    if parts[8] != "": hdop = float(parts[8])
                except: pass
                try:
                    if parts[9] != "": altitude_m = float(parts[9])
                except: pass

    except Exception as e:
        print("GPS parse error:", e)

    # If no fresh fix this loop, optionally fall back to last known fix
    if latitude is None or longitude is None:
        if last_fix:
            latitude, longitude, speed_kmh, course_deg, utc_time, sats_used, hdop, altitude_m = last_fix
    else:
        last_fix = (latitude, longitude, speed_kmh, course_deg, utc_time, sats_used, hdop, altitude_m)

    # -------- Build & send LoRa message --------
    if latitude is not None and longitude is not None:
        spd_txt = f"{speed_kmh:.2f}km/h" if isinstance(speed_kmh, (int,float)) else "N/A"
        crs_txt = f"{course_deg:.1f}°"    if isinstance(course_deg, (int,float)) else "N/A"
        time_txt = utc_time if utc_time else "N/A"
        sats_txt = str(sats_used) if sats_used is not None else "N/A"
        hdop_txt = f"{hdop:.1f}" if isinstance(hdop, (int,float)) else "N/A"
        alt_txt  = f"{altitude_m:.1f}m" if isinstance(altitude_m, (int,float)) else "N/A"

        message = (
            f"GPS:{latitude:.5f},{longitude:.5f} "
            f"Spd:{spd_txt} Crs:{crs_txt} "
            f"Time:{time_txt} Sats:{sats_txt} HDOP:{hdop_txt} Alt:{alt_txt} "
            f"Cnt:{counter}"
        )
    else:
        message = f"No GPS Fix Cnt:{counter}"

    try:
        payload = [ord(c) for c in message]
        LoRa.beginPacket()
        LoRa.write(payload, len(payload))
        LoRa.endPacket()
        LoRa.wait()

        print(f"Sent: {message}")
        print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s"
              .format(LoRa.transmitTime(), LoRa.dataRate()))
    except Exception as e:
        print("LoRa send error:", e)

    time.sleep(1.0)   # faster updates if GNSS is streaming at 1 Hz
    counter = (counter + 1) % 256

import os, sys, time, serial
from datetime import datetime

# ----------------- Import LoRa Library -----------------
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# ----------------- GPS Setup -----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

def nmea_to_decimal(raw, hemi, is_lat=True):
    """Convert NMEA degree format to decimal degrees"""
    try:
        deg_len = 2 if is_lat else 3
        degrees = int(raw[:deg_len])
        minutes = float(raw[deg_len:])
        decimal = degrees + (minutes / 60.0)
        if hemi in ['S', 'W']:
            decimal = -decimal
        return decimal
    except Exception:
        return None

def read_gps():
    """
    Read one line of NMEA GPS data.
    Return (lat, lon, speed, time, fix_status).
    fix_status = True if GPS fix, False if not.
    """
    try:
        line = gps.readline().decode("ascii", errors="ignore").strip()
        if line.startswith("$GPRMC"):
            parts = line.split(",")
            if len(parts) > 7:
                if parts[2] == "A":  # GPS fix
                    raw_lat, hemi_lat = parts[3], parts[4]
                    raw_lon, hemi_lon = parts[5], parts[6]
                    speed_knots = float(parts[7]) if parts[7] else 0.0
                    raw_time = parts[1]

                    lat = nmea_to_decimal(raw_lat, hemi_lat, True)
                    lon = nmea_to_decimal(raw_lon, hemi_lon, False)
                    speed_kmh = speed_knots * 1.852  # knots → km/h
                    utc_time = f"{raw_time[0:2]}:{raw_time[2:4]}:{raw_time[4:6]}"

                    return lat, lon, speed_kmh, utc_time, True
                else:
                    # No GPS fix yet
                    return None, None, None, None, False
    except Exception:
        pass
    return None, None, None, None, False

# ----------------- LoRa Setup -----------------
busId = 0; csId = 0
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1
LoRa = SX126x()

print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Something wrong, can't begin LoRa radio")

LoRa.setDio2RfSwitch()
print("Set frequency to 865 MHz")
LoRa.setFrequency(865000000)
print("Set TX power to +22 dBm")
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
print("Set modulation parameters")
LoRa.setLoRaModulation(7, 125000, 5)
print("Set packet parameters (initial)")
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 64, True)
print("Set sync word")
LoRa.setSyncWord(0x3444)

print("\n-- LoRa GNSS Transmitter --\n")

# ----------------- Main Loop -----------------
counter = 0
while True:
    lat, lon, speed, utc_time, fix = read_gps()

    if fix:
        # GPS fix available
        message = f"{lat:.6f},{lon:.6f},{speed:.2f}km/h,{utc_time}"
    else:
        # No GPS fix → still send status with system time
        system_time = datetime.utcnow().strftime("%H:%M:%S")
        message = f"NO_FIX,{counter},{system_time}"

    # Convert message to bytes
    message_bytes = [ord(c) for c in message]

    # Update payload length dynamically
    LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, len(message_bytes) + 1, True)

    # Transmit
    LoRa.beginPacket()
    LoRa.write(message_bytes, len(message_bytes))
    LoRa.write([counter], 1)   # add counter at end
    LoRa.endPacket()

    # Debug print
    print(f"Sent: {message} | Count: {counter}")

    # Wait until transmit complete
    LoRa.wait()
    print("Transmit time: {0:0.2f} ms | Data rate: {1:0.2f} byte/s"
          .format(LoRa.transmitTime(), LoRa.dataRate()))

    # Send every 1 second
    time.sleep(1)
    counter = (counter + 1) % 256

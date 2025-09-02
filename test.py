import time, serial
from LoRaRF import SX126x

# ---------------- GPS Setup ----------------
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

def parse_nmea(sentence):
    """Extract latitude & longitude from NMEA sentence"""
    try:
        parts = sentence.split(",")
        if parts[0] in ["$GPGGA", "$GPRMC"]:   # Accept both types
            lat_raw = parts[2]
            lat_hemi = parts[3]
            lon_raw = parts[4]
            lon_hemi = parts[5]

            if lat_raw and lon_raw:
                # Convert NMEA to decimal degrees
                lat = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
                lon = float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0

                if lat_hemi == "S":
                    lat = -lat
                if lon_hemi == "W":
                    lon = -lon
                return lat, lon
    except:
        return None
    return None

# ---------------- LoRa Setup ----------------
busId = 0; csId = 0
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1

LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Something wrong, can't begin LoRa radio")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)   # India ISM band
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)

# LoRa parameters
sf = 7; bw = 125000; cr = 5
LoRa.setLoRaModulation(sf, bw, cr)

headerType = LoRa.HEADER_EXPLICIT
preambleLength = 12
payloadLength = 255
crcType = True
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)
LoRa.setSyncWord(0x3444)

print("\n-- LoRa GNSS Transmitter --\n")

# ---------------- Transmit Loop ----------------
try:
    while True:
        line = gps.readline().decode("utf-8", errors="ignore").strip()

        if line.startswith("$GPGGA") or line.startswith("$GPRMC"):
            coords = parse_nmea(line)
            if coords:
                lat, lon = coords

                # Format as plain text
                packet = f"{lat:.6f},{lon:.6f}"
                packet_bytes = packet.encode("utf-8")  # 👈 send as string, not ord()

                # Send LoRa packet
                LoRa.beginPacket()
                LoRa.write(packet_bytes, len(packet_bytes))
                LoRa.endPacket(False)  # non-blocking send

                # Print clean output
                print(f"Latitude: {lat:.6f}, Longitude: {lon:.6f}")

                time.sleep(1)  # GNSS update rate

except KeyboardInterrupt:
    print("\nStopping GNSS transmitter...")
    LoRa.end()
    gps.close()

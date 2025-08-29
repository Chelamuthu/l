import serial
import time
from LoRaRF import SX126x

# ---------------- LoRa Setup ----------------
busId = 0; csId = 0
resetPin = 18; busyPin = 20
irqPin = -1; txenPin = 6; rxenPin = -1

lora = SX126x()
print("Initializing LoRa...")
if not lora.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    print("LoRa init failed!")
    exit(1)

lora.setFrequency(868000000)   # Change to your module band (868/915/433 MHz)
lora.setTxPower(22, 0)
lora.setSpreadingFactor(7)
lora.setBandwidth(125000)
lora.setCodingRate(5)

# ---------------- GNSS Setup ----------------
# GNSS is usually connected via UART (check Waveshare doc for exact port, often /dev/ttyAMA0 or /dev/serial0)
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

def parse_gps(data):
    """Extract lat/lon from NMEA sentence $GPGGA or $GPRMC"""
    if data.startswith("$GPRMC"):
        parts = data.split(",")
        if parts[2] == "A":  # A = valid
            lat_raw = parts[3]
            lat_dir = parts[4]
            lon_raw = parts[5]
            lon_dir = parts[6]

            # Convert from ddmm.mmmm to decimal degrees
            lat = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
            lon = float(lon_raw[:3]) + float(lon_raw[3:]) / 60.0
            if lat_dir == "S": lat = -lat
            if lon_dir == "W": lon = -lon
            return lat, lon
    return None

# ---------------- Main Loop ----------------
while True:
    try:
        line = gps.readline().decode("ascii", errors="replace").strip()
        coords = parse_gps(line)
        if coords:
            lat, lon = coords
            message = f"Lat:{lat:.6f}, Lon:{lon:.6f}"
            print("Sending:", message)
            lora.send(message.encode())
            time.sleep(5)  # send every 5 seconds
    except Exception as e:
        print("Error:", e)

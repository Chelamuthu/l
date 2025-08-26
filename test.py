import spidev
import time
import RPi.GPIO as GPIO
import serial

# -------------------------------
# SX1262 Pins & Commands
# -------------------------------
NSS_PIN   = 8   # Chip Select
RESET_PIN = 25
BUSY_PIN  = 24  # SX1262 busy pin

SET_STANDBY       = 0x80
SET_PACKET_TYPE   = 0x8A
SET_RF_FREQUENCY  = 0x86
SET_BUFFER_BASE   = 0x8F
SET_TX            = 0x83
WRITE_BUFFER      = 0x0E

PACKET_TYPE_LORA  = 0x01

# -------------------------------
# SX1262 Driver (Minimal)
# -------------------------------
class SX1262:
    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0b00

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(NSS_PIN, GPIO.OUT)
        GPIO.setup(RESET_PIN, GPIO.OUT)
        GPIO.setup(BUSY_PIN, GPIO.IN)

        GPIO.output(NSS_PIN, GPIO.HIGH)
        self.reset()
        self.standby()
        self.set_packet_type(PACKET_TYPE_LORA)
        self.set_frequency(866000000)  # ✅ India ISM band
        self.set_buffer_base(0, 0)

    def wait_busy(self):
        while GPIO.input(BUSY_PIN) == 1:
            time.sleep(0.001)

    def reset(self):
        GPIO.output(RESET_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(RESET_PIN, GPIO.HIGH)
        time.sleep(0.01)
        print("SX1262 Reset")

    def spi_write(self, buf):
        GPIO.output(NSS_PIN, GPIO.LOW)
        self.spi.xfer2(buf)
        GPIO.output(NSS_PIN, GPIO.HIGH)

    def command(self, opcode, data=[]):
        self.wait_busy()
        self.spi_write([opcode] + data)
        self.wait_busy()

    def standby(self):
        self.command(SET_STANDBY, [0x00])  # RC standby

    def set_packet_type(self, pkt_type):
        self.command(SET_PACKET_TYPE, [pkt_type])

    def set_frequency(self, freq_hz):
        freq = int((freq_hz / (32e6)) * (1 << 25))
        buf = [(freq >> 24) & 0xFF, (freq >> 16) & 0xFF,
               (freq >> 8) & 0xFF, freq & 0xFF]
        self.command(SET_RF_FREQUENCY, buf)

    def set_buffer_base(self, tx_base, rx_base):
        self.command(SET_BUFFER_BASE, [tx_base, rx_base])

    def write_buffer(self, data):
        self.wait_busy()
        GPIO.output(NSS_PIN, GPIO.LOW)
        self.spi.xfer2([WRITE_BUFFER, 0x00] + data)
        GPIO.output(NSS_PIN, GPIO.HIGH)
        self.wait_busy()

    def send_payload(self, payload):
        self.write_buffer(list(payload))
        self.command(SET_TX, [0x00, 0x00, 0x00])  # TX until done
        print("LoRa TX:", payload)

    def close(self):
        self.spi.close()
        GPIO.cleanup()

# -------------------------------
# GNSS Reader
# -------------------------------
def open_gps():
    try:
        return serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)
    except:
        return serial.Serial("/dev/serial0", baudrate=9600, timeout=1)

def parse_nmea_latlon(parts):
    """Convert NMEA lat/lon to decimal degrees"""
    if parts[2] == "A":  # Status A = valid
        # Latitude
        lat_raw = parts[3]
        lat_deg = float(lat_raw[:2])
        lat_min = float(lat_raw[2:])
        lat = lat_deg + (lat_min / 60.0)
        if parts[4] == "S":
            lat = -lat

        # Longitude
        lon_raw = parts[5]
        lon_deg = float(lon_raw[:3])
        lon_min = float(lon_raw[3:])
        lon = lon_deg + (lon_min / 60.0)
        if parts[6] == "W":
            lon = -lon

        return lat, lon
    return None, None

def read_gps(gps_serial):
    """Read one line from GPS"""
    line = gps_serial.readline().decode("ascii", errors="ignore").strip()
    if line.startswith("$GPRMC"):
        parts = line.split(",")
        return parse_nmea_latlon(parts)
    return None

# -------------------------------
# Main
# -------------------------------
if __name__ == "__main__":
    sx = SX1262()
    gps_serial = open_gps()

    try:
        while True:
            gps_data = read_gps(gps_serial)
            if gps_data and gps_data[0] and gps_data[1]:
                lat, lon = gps_data
                msg = f"Lat:{lat:.6f},Lon:{lon:.6f}"
                sx.send_payload(msg.encode())
            time.sleep(2)

    except KeyboardInterrupt:
        print("Stopped")
    finally:
        sx.close()
        gps_serial.close()

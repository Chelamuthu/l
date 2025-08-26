import serial
import time
import pynmea2
import spidev
import RPi.GPIO as GPIO

# LoRa control pins
NSS_PIN = 8
RESET_PIN = 25
BUSY_PIN = 24

# SX1262 opcodes
SET_STANDBY = 0x80
SET_PACKET_TYPE = 0x8A
SET_RF_FREQUENCY = 0x86
SET_BUFFER_BASE = 0x8F
WRITE_BUFFER = 0x0E
SET_TX = 0x83
PACKET_TYPE_LORA = 0x01

# Initialize SX1262 via SPI + GPIO
class SX1262Custom:
    def __init__(self, freq_hz=866000000):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
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
        self.set_frequency(freq_hz)
        self.set_buffer_base(0, 0)

    def wait_busy(self):
        while GPIO.input(BUSY_PIN):
            time.sleep(0.001)

    def reset(self):
        GPIO.output(RESET_PIN, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(RESET_PIN, GPIO.HIGH)
        time.sleep(0.01)
        print("[LoRa] Reset done")

    def write_command(self, opcode, data=[]):
        self.wait_busy()
        GPIO.output(NSS_PIN, GPIO.LOW)
        self.spi.xfer2([opcode] + data)
        GPIO.output(NSS_PIN, GPIO.HIGH)
        self.wait_busy()

    def standby(self):
        self.write_command(SET_STANDBY, [0x00])

    def set_packet_type(self, pkt_type):
        self.write_command(SET_PACKET_TYPE, [pkt_type])

    def set_frequency(self, freq_hz):
        freq = int((freq_hz / (32e6)) * (1 << 25))
        buf = [(freq >> i) & 0xFF for i in (24, 16, 8, 0)]
        self.write_command(SET_RF_FREQUENCY, buf)

    def set_buffer_base(self, tx, rx):
        self.write_command(SET_BUFFER_BASE, [tx, rx])

    def send_payload(self, payload_bytes):
        self.write_command(WRITE_BUFFER, [0x00] + payload_bytes)
        self.write_command(SET_TX, [0x00, 0x00, 0x00])
        print(f"[LoRa] Sent: {payload_bytes}")

    def cleanup(self):
        self.spi.close()
        GPIO.cleanup()

# Convert NMEA to decimal
def nmea_to_decimal(raw, direction):
    deg = int(float(raw) / 100)
    minutes = float(raw) - deg * 100
    decimal = deg + minutes / 60
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

# Read GPS data
def read_gps(gps_ser):
    line = gps_ser.readline().decode(errors="ignore").strip()
    if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
        parts = line.split(',')
        if parts[2] and parts[4]:
            lat = nmea_to_decimal(parts[2], parts[3])
            lon = nmea_to_decimal(parts[4], parts[5])
            return lat, lon
    return None

if __name__ == "__main__":
    lora = SX1262Custom(freq_hz=866000000)  # India band
    gps_serial = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
    last_fix = None

    try:
        while True:
            gps_data = read_gps(gps_serial)
            if gps_data:
                last_fix = gps_data
                print(f"[GPS] Fix: {gps_data}")
            elif last_fix:
                print(f"[GPS] Using last known: {last_fix}")
            else:
                print("[GPS] No fix yet...")

            if last_fix:
                msg = f"{last_fix[0]:.6f},{last_fix[1]:.6f}"
                lora.send_payload(list(msg.encode()))

            time.sleep(5)

    except KeyboardInterrupt:
        print("Stopping")
    finally:
        lora.cleanup()
        gps_serial.close()

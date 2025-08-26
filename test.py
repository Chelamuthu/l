import spidev
import time
import RPi.GPIO as GPIO

# Pins & Commands
NSS_PIN   = 8
RESET_PIN = 25
BUSY_PIN  = 24

SET_STANDBY       = 0x80
SET_PACKET_TYPE   = 0x8A
SET_RF_FREQUENCY  = 0x86
SET_BUFFER_BASE   = 0x8F
SET_TX            = 0x83
WRITE_BUFFER      = 0x0E
PACKET_TYPE_LORA  = 0x01

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
        self.set_frequency(866000000)  # Adjust for your region
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

    def spi_write(self, data):
        GPIO.output(NSS_PIN, GPIO.LOW)
        self.spi.xfer2(data)
        GPIO.output(NSS_PIN, GPIO.HIGH)

    def command(self, opcode, data=[]):
        self.wait_busy()
        self.spi_write([opcode] + data)
        self.wait_busy()

    def standby(self):
        self.command(SET_STANDBY, [0x00])

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
        self.command(SET_TX, [0x00, 0x00, 0x00])
        print("LoRa TX:", payload)

    def close(self):
        self.spi.close()
        GPIO.cleanup()

if __name__ == "__main__":
    sx = SX1262()
    try:
        while True:
            message = "Hello LoRa SX1262"
            sx.send_payload(message.encode())
            time.sleep(5)
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        sx.close()

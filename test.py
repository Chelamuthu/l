import spidev
import RPi.GPIO as GPIO
import time

# SX1262 pins (adjust as per your wiring / Waveshare HAT defaults)
NSS = 8      # Chip Select
RESET = 17   # Reset pin
BUSY = 25    # Busy pin
DIO1 = 4     # IRQ pin

# Setup SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, CS0
spi.max_speed_hz = 500000

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(NSS, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(RESET, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(BUSY, GPIO.IN)
GPIO.setup(DIO1, GPIO.IN)

def write_command(opcode, data=[]):
    """Send command to SX1262 via SPI"""
    while GPIO.input(BUSY) == 1:
        time.sleep(0.001)
    GPIO.output(NSS, GPIO.LOW)
    spi.xfer2([opcode] + data)
    GPIO.output(NSS, GPIO.HIGH)

def reset():
    GPIO.output(RESET, GPIO.LOW)
    time.sleep(0.05)
    GPIO.output(RESET, GPIO.HIGH)
    time.sleep(0.05)

def send_message(message):
    """Send a LoRa message"""
    data = [ord(c) for c in message]
    write_command(0x0E, [0x00])      # Set buffer base address
    write_command(0x0D, [0x00, 0x00]) # Set buffer pointer
    write_command(0x0E, data)        # Write payload
    write_command(0x83, [0x00, len(data)])  # Set TX params
    write_command(0x03, [0x00])      # TX mode

    print("📡 Sent:", message)

if __name__ == "__main__":
    reset()
    while True:
        send_message("Hello from Pi SX1262")
        time.sleep(5)

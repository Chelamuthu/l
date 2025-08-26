import serial
import spidev
import RPi.GPIO as GPIO
import time

# ----------------------------
# GPIO Pin mapping (adjust if needed)
# ----------------------------
PIN_RESET = 17   # SX1262 Reset pin
PIN_BUSY  = 18   # SX1262 Busy pin
PIN_DIO1  = 23   # SX1262 DIO1 pin
PIN_NSS   = 24   # SX1262 Chip select

# ----------------------------
# SPI Setup for LoRa
# ----------------------------
spi = spidev.SpiDev()
spi.open(0, 0)   # Bus 0, Device 0 (/dev/spidev0.0)
spi.max_speed_hz = 5000000

# ----------------------------
# GPIO Setup
# ----------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_RESET, GPIO.OUT)
GPIO.setup(PIN_BUSY, GPIO.IN)
GPIO.setup(PIN_DIO1, GPIO.IN)
GPIO.setup(PIN_NSS, GPIO.OUT)

# ----------------------------
# GNSS Setup (via UART)
# ----------------------------
try:
    gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)
    print("[INFO] GNSS module connected on /dev/ttyAMA0")
except Exception as e:
    print("[ERROR] Cannot open GNSS UART:", e)
    gps = None

# ----------------------------
# Reset SX1262
# ----------------------------
def reset_lora():
    GPIO.output(PIN_RESET, GPIO.LOW)
    time.sleep(0.05)
    GPIO.output(PIN_RESET, GPIO.HIGH)
    time.sleep(0.05)
    print("[INFO] SX1262 Reset done")

reset_lora()

# ----------------------------
# Send SPI command (basic test)
# ----------------------------
def lora_write_cmd(cmd, data=[]):
    GPIO.output(PIN_NSS, GPIO.LOW)
    spi.xfer2([cmd] + data)
    GPIO.output(PIN_NSS, GPIO.HIGH)

# Example: Get LoRa Chip version
def lora_get_version():
    GPIO.output(PIN_NSS, GPIO.LOW)
    resp = spi.xfer2([0xC0, 0x00])  # Example command (GetStatus / GetVersion may vary)
    GPIO.output(PIN_NSS, GPIO.HIGH)
    return resp

print("[INFO] SX1262 Version/Status:", lora_get_version())

# ----------------------------
# Read GNSS Data
# ----------------------------
def read_gnss():
    if gps:
        line = gps.readline().decode(errors="ignore").strip()
        if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
            parts = line.split(",")
            if len(parts) > 5 and parts[2] != "" and parts[4] != "":
                lat = float(parts[2]) / 100.0
                lon = float(parts[4]) / 100.0
                return (lat, lon)
    return None

# ----------------------------
# Main Loop
# ----------------------------
try:
    while True:
        pos = read_gnss()
        if pos:
            print(f"[GNSS] Lat: {pos[0]}, Lon: {pos[1]}")

            # Send GNSS location via LoRa
            message = f"LAT:{pos[0]:.5f},LON:{pos[1]:.5f}"
            data = [ord(c) for c in message]

            print("[LORA] Sending:", message)
            lora_write_cmd(0x0E, data)  # Example: WriteBuffer command

        else:
            print("[GNSS] No fix yet...")

        time.sleep(2)

except KeyboardInterrupt:
    print("Exit")
    spi.close()
    if gps:
        gps.close()
    GPIO.cleanup()

import spidev
import RPi.GPIO as GPIO
import time
from SX126x import SX126x

# ----------------- LoRa Pins -----------------
NSS = 8
RESET = 17
BUSY = 25
DIO1 = 4

# ----------------- Setup -----------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 5000000

lora = SX126x(spi, NSS, RESET, BUSY, DIO1)

print("Configuring LoRa...")
lora.begin()
lora.setFrequency(865000000)
lora.setTxPower(22, lora.TX_POWER_SX1262)
lora.setSpreadingFactor(7)
lora.setBandwidth(125000)
lora.setCodingRate(5)

print("LoRa Receiver Ready!")

try:
    while True:
        if lora.checkIrq():  # Check IRQ for incoming packet
            payload = lora.readPayload()
            if payload:
                msg = payload.decode("utf-8", errors="ignore")
                print(f"Received: {msg}")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping...")
    GPIO.cleanup()

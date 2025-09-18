import os, sys
import time
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# ==============================
# LORA CONFIGURATION
# ==============================
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1

LoRa = SX126x()
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Cannot start LoRa")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)
LoRa.setRxGain(LoRa.RX_GAIN_POWER_SAVING)

# Modulation parameters
sf = 7
bw = 125000
cr = 5
LoRa.setLoRaModulation(sf, bw, cr)

# Packet parameters
headerType = LoRa.HEADER_EXPLICIT
preambleLength = 12
payloadLength = 15
crcType = True
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)
LoRa.setSyncWord(0x3444)

print("\n-- LoRa Receiver with Antenna Check --\n")

try:
    while True:
        LoRa.request()
        LoRa.wait()

        message = ""
        while LoRa.available() > 1:
            message += chr(LoRa.read())
        counter = LoRa.read() if LoRa.available() else 0

        # Antenna detection based on RSSI
        rssi = LoRa.packetRssi()
        if rssi < -100:  # very weak signal likely no antenna
            antenna_status = "Antenna NOT connected"
        else:
            antenna_status = "Antenna connected"

        # Print received message if any
        if message:
            print(f"Message received: {message}  Counter: {counter}")
        
        print(f"{antenna_status} | RSSI: {rssi:.2f} dBm | SNR: {LoRa.snr():.2f} dB")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n[INFO] Stopped by user")

finally:
    LoRa.end()

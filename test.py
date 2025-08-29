import time
from LoRaRF import SX126x

# ---------------- LoRa Setup ----------------
busId = 0; csId = 0
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1
lora = SX126x()

print("Init LoRa Receiver...")
if not lora.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    print("LoRa init failed")
    exit(1)

lora.setFrequency(865000000)
lora.setTxPower(22, lora.TX_POWER_SX1262)
lora.setLoRaModulation(7, 125000, 5)

print("LoRa Receiver Ready 📡")

# ---------------- Main Loop ----------------
while True:
    if lora.available():
        msg = lora.readString()
        print("Received:", msg)

        try:
            lat, lon = msg.split(",")
            print(f"Latitude: {lat}, Longitude: {lon}")
        except:
            print("Invalid message:", msg)
    time.sleep(1)

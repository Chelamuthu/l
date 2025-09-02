import os, sys, time
from LoRaRF import SX126x

# ---------------- Setup ----------------
busId = 0; csId = 0 
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1 

LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Something wrong, can't begin LoRa radio")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)

# LoRa parameters
sf = 7
bw = 125000
cr = 5
LoRa.setLoRaModulation(sf, bw, cr)

# Payload length set to max for flexibility
headerType = LoRa.HEADER_EXPLICIT
preambleLength = 12
payloadLength = 255    # max length (let us decide per packet)
crcType = True
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)

LoRa.setSyncWord(0x3444)

print("\n-- LoRa Transmitter (Continuous High Speed) --\n")

# ---------------- Transmission Loop ----------------
message = "HeLoRa World!"
counter = 0

try:
    while True:
        # Build message
        packet = f"{message} {counter}"
        packet_bytes = [ord(c) for c in packet]

        # Send packet
        LoRa.beginPacket()
        LoRa.write(packet_bytes, len(packet_bytes))
        LoRa.endPacket()

        # Print transmit info
        print(f"Sent: {packet}")
        print("Transmit time: {:.2f} ms | Data rate: {:.2f} byte/s"
              .format(LoRa.transmitTime(), LoRa.dataRate()))

        counter = (counter + 1) % 256

        # Small delay (adjust for speed vs reliability)
        time.sleep(0.2)   # 200 ms between packets

except KeyboardInterrupt:
    print("\nStopping LoRa transmitter...")
    LoRa.end()

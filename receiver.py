import os, sys, time
from LoRaRF import SX126x

# Setup paths
currentdir = os.path.dirname(os.path.realpath(_file_))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))

# Pin configuration
busId = 0; csId = 0
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1

# Init LoRa
LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Something wrong, can't begin LoRa radio")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(868000000)
LoRa.setRxGain(LoRa.RX_GAIN_POWER_SAVING)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, 15, True)
LoRa.setSyncWord(0x3444)

print("\n-- LoRa Receiver --\n")

try:
    while True:
        LoRa.request()
        LoRa.wait()

        # Read payload
        message = ""
        while LoRa.available() > 0:
            message += chr(LoRa.read())

        print(f"Message: {message}")

        # Packet info
        rssi = LoRa.packetRssi()
        snr = LoRa.snr()

        # Filter unrealistic SNR values
        if snr < -20 or snr > 12:
            snr = "Invalid"

        print(f"Packet status: RSSI = {rssi:.2f} dBm | SNR = {snr} dB")

        # Errors
        status = LoRa.status()
        if status == LoRa.STATUS_CRC_ERR:
            print("CRC error")
        elif status == LoRa.STATUS_HEADER_ERR:
            print("Packet header error")

except KeyboardInterrupt:
    print("\nStopping receiver...")
    LoRa.end()

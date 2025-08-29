import os, sys, time, serial
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# --- LoRa Setup ---
busId = 0; csId = 0
resetPin = 18; busyPin = 20; irqPin = -1; txenPin = 6; rxenPin = -1

LoRa = SX126x()
print("Begin LoRa radio")
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise Exception("Can't begin LoRa radio")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)  # India frequency band
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)

# Dynamic payload (0 = auto size, avoids hangs)
headerType = LoRa.HEADER_EXPLICIT
preambleLength = 12
payloadLength = 0
crcType = True
LoRa.setLoRaPacket(headerType, preambleLength, payloadLength, crcType)
LoRa.setSyncWord(0x3444)

# --- GNSS Setup ---
# On Waveshare board, GNSS is usually at /dev/ttyS0 or /dev/ttyAMA0
try:
    gnss = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
except Exception as e:
    raise Exception(f"Could not open GNSS serial port: {e}")

print("\n-- LoRa GNSS Transmitter --\n")

counter = 0

def get_gnss_sentence():
    """Read one valid GNSS NMEA sentence from module."""
    try:
        line = gnss.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith("$GNRMC") or line.startswith("$GPGGA"):
            return line
    except:
        return None
    return None

try:
    while True:
        nmea = get_gnss_sentence()
        if nmea:
            payload = f"{nmea} CNT:{counter}"
            data = [ord(c) for c in payload]

            try:
                LoRa.beginPacket()
                LoRa.write(data, len(data))
                LoRa.endPacket()

                # Wait with timeout (prevents hang)
                if not LoRa.wait(timeout=2000):
                    print("⚠️ TX timeout, resetting LoRa")
                    LoRa.end()
                    time.sleep(0.5)
                    LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin)

                print(f"Sent GNSS: {payload}")
                print("TX time: {:.2f} ms | Rate: {:.2f} B/s".format(
                    LoRa.transmitTime(), LoRa.dataRate()
                ))

            except Exception as e:
                print(f"❌ Error sending packet: {e}")
                LoRa.end()
                time.sleep(1)
                LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin)

            counter = (counter + 1) % 256
            time.sleep(2)  # don’t overload LoRa
        else:
            # If no GNSS fix/data, just wait
            print("No GNSS data yet...")
            time.sleep(1)

except KeyboardInterrupt:
    print("Stopping transmitter...")
finally:
    LoRa.end()
    gnss.close()

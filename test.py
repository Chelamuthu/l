import serial
import time

# ------------------------------
# LoRa Configuration (India band)
# ------------------------------
LORA_FREQUENCY = 866000000  # 866 MHz
SERIAL_PORT = "/dev/ttyAMA0"  # UART port for Raspberry Pi
BAUDRATE = 115200

# ------------------------------
# Initialize Serial
# ------------------------------
try:
    lora = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print(f"✅ Connected to LoRa module on {SERIAL_PORT} at {BAUDRATE} baud")
except Exception as e:
    print(f"❌ Failed to connect to LoRa module: {e}")
    exit()

# ------------------------------
# Send AT Command to LoRa module
# ------------------------------
def send_command(cmd, delay=0.5):
    try:
        lora.write((cmd + "\r\n").encode())
        time.sleep(delay)
        response = lora.read_all().decode(errors="ignore").strip()
        if response:
            print(f"📡 {cmd} -> {response}")
        else:
            print(f"⚠️ No response for {cmd}")
        return response
    except Exception as e:
        print(f"⚠️ Error sending command {cmd}: {e}")
        return ""

# ------------------------------
# Initialize LoRa
# ------------------------------
def init_lora():
    send_command("AT")  # Check communication
    send_command("AT+RESET")  # Reset LoRa
    send_command(f"AT+FREQ={LORA_FREQUENCY}")  # Set frequency
    send_command("AT+BW=125")  # Bandwidth 125kHz
    send_command("AT+CR=4/5")  # Coding Rate
    send_command("AT+SF=7")    # Spreading Factor 7
    send_command("AT+POWER=22")  # TX Power 22dBm
    send_command("AT+MODE=0")  # Set to LoRa mode

# ------------------------------
# Transmit a Message
# ------------------------------
def transmit_message(message):
    send_command(f"AT+SEND={len(message)},{message}")

# ------------------------------
# GNSS Function
# ------------------------------
def get_gnss_location():
    print("📡 Requesting GNSS location...")
    resp = send_command("AT+GNSS=1", delay=2)  # Start GNSS
    time.sleep(3)
    resp = send_command("AT+GNSS=?", delay=2)  # Request location
    return resp

# ------------------------------
# Main Program
# ------------------------------
if __name__ == "__main__":
    print("🚀 Initializing LoRa SX1262 Node...")
    init_lora()

    while True:
        # Transmit test message
        transmit_message("Hello from India LoRa Node 🚀")
        
        # Get GNSS location
        gps_data = get_gnss_location()
        if gps_data:
            print(f"🌍 GNSS Data: {gps_data}")
        else:
            print("⚠️ No GNSS data received")

        time.sleep(10)  # Wait before next transmission

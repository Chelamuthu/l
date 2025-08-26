import serial
import time

# Change COM port to match your USB dongle (COM8 in your case, or /dev/ttyUSB0 in Linux)
PORT = "COM8"  
BAUD = 9600    

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"✅ Sender connected to {PORT} at {BAUD} baud")

        count = 0
        while True:
            message = f"Hello LoRa {count}"
            ser.write((message + "\n").encode())
            print(f"📤 Sent: {message}")
            count += 1
            time.sleep(2)

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()

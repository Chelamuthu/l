#!/usr/bin/python3
# -- coding: UTF-8 --

import os
import sys
import time

# ==============================
# LoRa Configuration
# ==============================
BUS_ID = 0
CS_ID = 0
RESET_PIN = 18
BUSY_PIN = 20
IRQ_PIN = -1
TX_EN = 6
RX_EN = -1
PAYLOAD_LENGTH = 100
LORA_FREQ = 868000000

# ==============================
# IMPORT LORA LIBRARY
# ==============================
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.dirname(os.path.dirname(currentdir)))
from LoRaRF import SX126x

# ==============================
# INITIALIZE LORA
# ==============================
print("[INFO] Initializing SX1262 LoRa Module for receiving...")
LoRa = SX126x()

if not LoRa.begin(BUS_ID, CS_ID, RESET_PIN, BUSY_PIN, IRQ_PIN, TX_EN, RX_EN):
    raise Exception("Failed to initialize LoRa module!")

LoRa.setDio2RfSwitch()
LoRa.setFrequency(LORA_FREQ)
LoRa.setLoRaModulation(sf=7, bw=125000, cr=5)
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, 12, PAYLOAD_LENGTH, True)
LoRa.setSyncWord(0x3444)
LoRa.setRxContinuous()  # Set continuous receive mode
print("[INFO] LoRa Receiver ready.\n")

# ==============================
# MAIN LOOP
# ==============================
print("[INFO] Waiting for incoming LoRa packets...\n")

try:
    while True:
        if LoRa.available():
            packet = LoRa.read(PAYLOAD_LENGTH)
            try:
                message = bytes(packet).decode('utf-8').rstrip('\x00')
                print(f"[RECEIVED] {message}")
            except Exception as e:
                print(f"[ERROR] Failed to decode packet: {e}")

        time.sleep(0.1)  # Small delay to prevent CPU hog

except KeyboardInterrupt:
    print("\n[INFO] Receiver stopped by user.")

finally:
    LoRa.end()
    print("[INFO] LoRa receiver closed safely.")

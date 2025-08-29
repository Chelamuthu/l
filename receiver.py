#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO
from LoRaRF import SX126x  # same library used on TX

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# --- LoRa pins (BCM) - must match wiring used by TX device/hardware ---
busId = 0
csId = 0
resetPin = 18
busyPin = 20
irqPin = -1
txenPin = 6
rxenPin = -1

LoRa = SX126x()
if not LoRa.begin(busId, csId, resetPin, busyPin, irqPin, txenPin, rxenPin):
    raise SystemExit("LoRa.begin() failed - check wiring and GPIO mode")

# Match exactly the TX config
LoRa.setDio2RfSwitch()
LoRa.setFrequency(865000000)           # MUST match transmitter
LoRa.setTxPower(22, LoRa.TX_POWER_SX1262)
LoRa.setLoRaModulation(7, 125000, 5)   # SF7, BW=125kHz, CR=4/5
LoRa.setLoRaPacket(LoRa.HEADER_EXPLICIT, preambleLength=12, payloadLength=255, crcType=True)
LoRa.setSyncWord(0x3444)

print("[LoRa RX] ready - listening for packets (Ctrl+C to stop)")

# Try to put into continuous RX if the wrapper supports it
try:
    if hasattr(LoRa, "receive"):
        # some wrappers: receive(0) -> continuous
        try:
            LoRa.receive(0)
            print("[LoRa RX] set to continuous receive via receive(0)")
        except TypeError:
            LoRa.receive()   # maybe no arg variant
            print("[LoRa RX] set to receive()")
    elif hasattr(LoRa, "startReceive"):
        LoRa.startReceive()
        print("[LoRa RX] startReceive() called")
    else:
        print("[LoRa RX] no continuous RX API found, will poll available()/read()")
except Exception as e:
    print("[WARN] could not set continuous RX:", e)

def try_get_rssi_snr(lora):
    rssi = None
    snr = None
    # attempt many possible method names that different libs use
    for fn in ("packetRssi", "lastPacketRssi", "getPacketRssi", "rssi", "getRssi"):
        if hasattr(lora, fn):
            try:
                val = getattr(lora, fn)()
                rssi = val
                break
            except Exception:
                pass
    for fn in ("packetSnr", "lastPacketSnr", "getPacketSnr", "snr"):
        if hasattr(lora, fn):
            try:
                val = getattr(lora, fn)()
                snr = val
                break
            except Exception:
                pass
    return rssi, snr

def read_packet_flexible(lora):
    """Try several read methods. Return (text, raw_bytes) or (None,None)."""
    try:
        # preferred API: available() -> getPacketLength() -> read(length)
        if hasattr(lora, "available") and lora.available():
            length = None
            if hasattr(lora, "getPacketLength"):
                try:
                    length = lora.getPacketLength()
                except Exception:
                    length = None
            try:
                if length and hasattr(lora, "read"):
                    raw = lora.read(length)
                elif hasattr(lora, "read"):
                    raw = lora.read()
                elif hasattr(lora, "readString"):
                    txt = lora.readString()
                    return txt, None
                elif hasattr(lora, "readPayload"):
                    raw = lora.readPayload()
                else:
                    return None, None
            except Exception as e:
                # try readString fallback
                if hasattr(lora, "readString"):
                    try:
                        return lora.readString(), None
                    except Exception:
                        return None, None
                return None, None

            # convert raw into text
            if raw is None:
                return None, None
            if isinstance(raw, bytes):
                return raw.decode("ascii", errors="replace"), raw
            if isinstance(raw, str):
                return raw, raw.encode()
            if isinstance(raw, (list, tuple)):
                try:
                    return "".join(chr(b) for b in raw), bytes(raw)
                except Exception:
                    return str(raw), None
            return str(raw), None

        # else: check IRQ/status-based APIs
        if hasattr(lora, "getIrqStatus"):
            try:
                irq = lora.getIrqStatus()
                rx_done_flag = getattr(lora, "IRQ_RX_DONE", 0)
                if irq & rx_done_flag:
                    # read payload by available read methods
                    if hasattr(lora, "read"):
                        raw = lora.read()
                    elif hasattr(lora, "readPayload"):
                        raw = lora.readPayload()
                    elif hasattr(lora, "readString"):
                        txt = lora.readString()
                        lora.clearIrqStatus(rx_done_flag)
                        return txt, None
                    else:
                        raw = None

                    # clear irq
                    try:
                        lora.clearIrqStatus(rx_done_flag)
                    except Exception:
                        pass

                    if raw is None:
                        return None, None
                    if isinstance(raw, bytes):
                        return raw.decode("ascii", errors="replace"), raw
                    if isinstance(raw, str):
                        return raw, raw.encode()
                    if isinstance(raw, (list, tuple)):
                        return "".join(chr(b) for b in raw), bytes(raw)
                    return str(raw), None
            except Exception:
                pass

    except Exception as e:
        print("[WARN] read_packet_flexible exception:", e)
    return None, None

try:
    while True:
        text, raw = read_packet_flexible(LoRa)
        if text:
            text = text.strip()
            rssi, snr = try_get_rssi_snr(LoRa)
            print(f"[RECV] {text}")
            if rssi is not None or snr is not None:
                print(f"       RSSI={rssi} dBm, SNR={snr}")
            # Try parsing expected CSV: counter,timestamp,lat,lon,speed
            if "," in text:
                parts = text.split(",", 4)
                if parts and (parts[0].isdigit() or parts[0].startswith("NO")):
                    print("  parsed:", parts)
            time.sleep(0.05)
        else:
            # poll interval
            time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopping receiver")
finally:
    try:
        LoRa.end()
    except Exception:
        pass
    GPIO.cleanup()

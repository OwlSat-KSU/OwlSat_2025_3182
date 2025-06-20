from digi.xbee.devices import XBeeDevice
import time

PORT = "COM11"
BAUD_RATE = 9600

device = XBeeDevice(PORT, BAUD_RATE)

try:
    device.open()
    print("Sending message to STM32...")

    while True:
        device.send_data_broadcast("Hello STM32!\n")
        print("Message sent.")
        time.sleep(2)

except Exception as e:
    print(f"Error: {e}")

finally:
    if device is not None and device.is_open():
        device.close()

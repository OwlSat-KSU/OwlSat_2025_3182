'''
from digi.xbee.devices import XBeeDevice
import time

# Set correct COM port (change as needed)
PORT = "COM19"  # Replace with the actual COM port connected to the XBee device
BAUD_RATE = 9600

# Initialize XBee device
device = XBeeDevice(PORT, BAUD_RATE)

try:
    # Open the XBee device
    device.open()
    print(f"✅ Connected to XBee device on {PORT}.")

    # Check if the device is operating in API mode
    if device.get_api_mode():
        print("📡 The XBee device is in API Mode.")
    else:
        print("📡 The XBee device is in Transparent Mode.")

    # Optional: Print some basic device information
    print(f"🔧 Firmware version: {device.get_firmware_version()}")
    print(f"🔧 Hardware version: {device.get_hardware_version()}")

    input("Press Enter to exit...\n")  # Keep the script running until the user presses Enter

except Exception as e:
    print(f"❌ Error: {e}")

finally:
    # Close the connection if it was opened
    if device is not None and device.is_open():
        device.close()
        print("🔴 Connection closed.")

'''

'''
from digi.xbee.devices import XBeeDevice
import time

# Set correct COM port (pick one)
PORT = "COM11"  # Change to COM19 if needed
BAUD_RATE = 9600

# Initialize XBee device
device = XBeeDevice(PORT, BAUD_RATE)

try:
    # Open the XBee device
    device.open()
    print(f"✅ Connected to XBee device on {PORT}.")

    # Check if the device is operating in AT mode (Transparent Mode)
    # In AT mode, you don't need to worry about API frames or special settings.
    if not device.get_api_mode():
        print("📡 The XBee device is in AT (Transparent) Mode.")
    else:
        print("📡 The XBee device is in API Mode. You should switch to AT mode for this script.")

    # Set up the data receiving callback (just for demonstration in AT mode)
    def data_receive_callback(xbee_message):
        # Callback for when data is received
        print(f"📥 Received from {xbee_message.remote_device.get_64bit_addr()}: {xbee_message.data.decode()}")

    # Set up the listener for receiving data
    device.add_data_received_callback(data_receive_callback)

    # Send data in AT mode (no special API frame needed)
    def send_data(data):
        # In AT mode, you send data directly using the `send_data` method.
        # The `data` will be sent as-is through the serial interface.
        device.send_data(data.encode())
        print(f"📤 Sent: {data}")

    # Example of sending data
    send_data("Hello from Python in AT Mode")

    input("Press Enter to exit...\n")  # Keep the script running until the user presses Enter

except Exception as e:
    print(f"❌ Error: {e}")

finally:
    # Close the connection if it was opened
    if device is not None and device.is_open():
        device.close()
        print("🔴 Connection closed.")

'''

from digi.xbee.devices import XBeeDevice

PORT = "COM11"  # Change this to match your system
BAUD_RATE = 9600

device = XBeeDevice(PORT, BAUD_RATE)

try:
    device.open()  # Open the XBee connection
    print(f"XBee connected! Operating Mode: {device.get_protocol()}")

except Exception as e:
    print(f"Error: {e}")

finally:
    if device is not None and device.is_open():
        device.close()

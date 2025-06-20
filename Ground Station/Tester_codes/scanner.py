from digi.xbee.devices import XBeeDevice


PORT = "COM11"  # Change to your XBee's serial port
BAUD_RATE = 9600

# Open XBee device
device = XBeeDevice(PORT, BAUD_RATE)
device.open()

print("Scanning for XBee devices...")

# Perform network discovery
xbee_network = device.get_network()
xbee_network.start_discovery_process()

while xbee_network.is_discovery_running():
    pass  # Wait until the process is complete

# List discovered devices and start receiving data from the first one
for remote_device in xbee_network.get_devices():
    print(f"Device found: {remote_device.get_64bit_addr()}")

    # Now that we have found the device, let's receive data
    def data_received_callback(data):
        try:
            # Decode and print the received data
            print(f"Data received: {data.decode()}")
        except Exception as e:
            print(f"Error decoding data: {e}")

    # Register callback for data reception
    device.add_data_received_callback(data_received_callback)

    # Keep the device running to listen for incoming data
    try:
        print("Waiting for data from remote device...")
        while True:
            pass  # This loop will keep the device running, receiving data

    except KeyboardInterrupt:
        print("Stopped by user.")
        break

# Close the device connection when done
device.close()

import spidev
import time

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0, 0)

# Function to receive data from Arduino
def receive_data():
    rx_data = spi.xfer2([0] * 20)  # Assuming maximum message length of 20 bytes
    return ''.join(chr(byte) for byte in rx_data if byte != 0)  # Convert bytes to string

try:
    while True:
        received_message = receive_data()
        if received_message:
            print("Received message from Arduino:", received_message)
        time.sleep(1)

except KeyboardInterrupt:
    spi.close()

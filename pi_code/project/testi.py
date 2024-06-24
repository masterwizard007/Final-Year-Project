import smbus
import time

# Define I2C address of Arduino
address = 0x12

# Create I2C bus
bus = smbus.SMBus(1)  # Raspberry Pi 4 uses bus 1

while True:
    # Read data from Arduino
    data = bus.read_byte(address)
    
    # Print received data
    print("Received data:", data)
    
    # Wait before reading again
    time.sleep(1)

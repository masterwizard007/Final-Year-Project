import spidev
import time

# Define SPI parameters
SPI_CHANNEL = 0
SPI_DEVICE = 0
SPI_SPEED = 1000000

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_DEVICE, SPI_CHANNEL)
spi.max_speed_hz = SPI_SPEED

msb = 0
lsb = 0

# Function to read sensor data from Arduino Nano via SPI
def read_sensor_data():
    # Send dummy byte to request data
    spi.xfer([0x00])
    # Read 2 bytes (MSB and LSB) from Arduino Nano
    msb = spi.xfer([0x00])[0]
    lsb = spi.xfer([0x00])[0]
    # Combine MSB and LSB to get sensor value
    sensor_value = (msb << 8) | lsb
    return sensor_value

# Main loop to read sensor data and control Arduino Nano
while True:
    # Read sensor data from Arduino Nano
    sensor_value = read_sensor_data()
    print("Received MSB:", msb)  # Debug print
    print("Received LSB:", lsb)  # Debug print
    print("Received sensor value:", sensor_value)
    
    # Send control signals to Arduino Nano (Replace this with your control logic)
    # Example: servo_value = ... # Calculate servo value
    #          esc_value = ... # Calculate ESC value
    #          control_signal = f"{servo_value},{esc_value}\n"
    #          ser.write(control_signal.encode())
    
    time.sleep(1)  # Adjust delay as needed

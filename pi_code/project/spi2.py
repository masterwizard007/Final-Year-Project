import spidev
import RPi.GPIO as GPIO

# Define the GPIO pin used for Slave Select (SS) on the Raspberry Pi
SS_PIN = 8  # For example, using GPIO pin 8

# Configure GPIO pin as output
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
GPIO.setup(SS_PIN, GPIO.OUT)

# Create SPI object
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0

# Function to select Arduino (set SS pin low)
def select_arduino():
    GPIO.output(SS_PIN, GPIO.LOW)

# Function to deselect Arduino (set SS pin high)
def deselect_arduino():
    GPIO.output(SS_PIN, GPIO.HIGH)

# Example SPI communication with Arduino
try:
    while True:
        select_arduino()  # Select Arduino for communication
        # Receive data from Arduino
        received_data = spi.readbytes(3)  # Example: receive 3 bytes of data
        print("Received data from Arduino:", received_data)
        deselect_arduino()  # Deselect Arduino
        # Other code...
finally:
    spi.close()  # Close SPI bus
    GPIO.cleanup()  # Clean up GPIO

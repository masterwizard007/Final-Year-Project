# Python Script

import serial

ser = serial.Serial('/dev/ttyUSB4', 9600) # Change '/dev/ttyUSB4' to the appropriate port
# Use ls /dev/tty* to find out the port

while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode().strip() # Read the serial data
        print("Analog value:", data) # Print the received data

import serial

# Define serial port and baudrate
ser = serial.Serial('/dev/ttyUSB1', 9600)  # Adjust port and baudrate as needed

try:
    while True:
        # Read data from serial port
        data = ser.readline().decode().strip()
        
        # Print the received data
        print(data)

except KeyboardInterrupt:
    # Handle Ctrl+C gracefully
    ser.close()

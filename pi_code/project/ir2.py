import RPi.GPIO as GPIO
import time

# Define GPIO pin for IR sensor input
IR_PIN = 14

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN)

# Function to calculate RPM
def calculate_rpm(channel):
    global pulse_start_time
    global rpm
    pulse_end_time = time.time()
    time_diff = pulse_end_time - pulse_start_time
    rpm = 1 / time_diff * 60
    pulse_start_time = pulse_end_time

# Add event detection for rising edge
GPIO.add_event_detect(IR_PIN, GPIO.RISING, callback=calculate_rpm)

try:
    pulse_start_time = time.time()
    rpm = 0
    print("Measuring RPM. Press Ctrl+C to exit.")
    while True:
        print("RPM:", rpm)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    GPIO.cleanup()

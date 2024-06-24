import RPi.GPIO as GPIO
import time

# Define GPIO pin for IR sensor input
IR_PIN = 20

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN)

# Function to calculate RPM
def calculate_rpm(channel):
    global pulse_start_time
    pulse_end_time = time.time()
    time_diff = pulse_end_time - pulse_start_time
    rpm = 1 / time_diff * 60
    print("RPM:", rpm)
    pulse_start_time = pulse_end_time

# Add event detection for rising edge
GPIO.add_event_detect(IR_PIN, GPIO.RISING, callback=calculate_rpm)

try:
    pulse_start_time = time.time()
    while True:
        # Your main program can continue here
        time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()

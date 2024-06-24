import RPi.GPIO as GPIO
import time

# Define GPIO pin to test
TEST_PIN = 20  # Change this to the GPIO pin you want to test

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TEST_PIN, GPIO.OUT)

try:
    print("Testing GPIO pin {}. Press Ctrl+C to exit.".format(TEST_PIN))
    while True:
        # Toggle GPIO pin state
        GPIO.output(TEST_PIN, not GPIO.input(TEST_PIN))
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    # Clean up GPIO
    GPIO.cleanup()

import RPi.GPIO as GPIO
import time

# Define GPIO pin for IR sensor input
IR_PIN = 14

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN)

# Global variables
pulse_times = []
moving = False
timeout_duration = 5  # Adjust timeout duration as needed (in seconds)

# Function to calculate RPM
def calculate_rpm():
    global pulse_times
    if len(pulse_times) < 2:
        return 0  # Not enough data to calculate RPM
    recent_pulse_times = pulse_times[-5:]  # Consider only the 5 most recent pulses
    time_diff = recent_pulse_times[-1] - recent_pulse_times[0]
    if time_diff != 0:
        rpm = 60 / (len(recent_pulse_times) - 1) / time_diff  # RPM = 1 / (time_between_pulses * num_pulses) (in minutes)
        return rpm
    else:
        return 0

try:
    print("Measuring RPM. Press Ctrl+C to exit.")
    while True:
        start_time = time.time()
        GPIO.wait_for_edge(IR_PIN, GPIO.RISING, timeout=int(timeout_duration * 1000))
        end_time = time.time()
        if GPIO.input(IR_PIN):  # If pulse detected
            pulse_times.append(end_time)
            rpm = calculate_rpm()
            moving = True  # Set the flag to indicate that the wheel is moving
            print("RPM:", rpm)
        elif moving:  # If no pulse detected and the wheel was moving
            print("RPM: 0")
            moving = False  # Reset the flag to indicate that the wheel is stopped
            pulse_times = []  # Reset pulse times

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    GPIO.cleanup()

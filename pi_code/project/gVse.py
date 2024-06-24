import evdev
import serial
import time
import concurrent.futures
import math
from colorama import Fore, Style
import RPi.GPIO as GPIO

# Define GPIO pin for IR sensor input
IR_PIN = 23

# Global variables
pulse_times = []
moving = False
timeout_duration = 5  # Adjust timeout duration as needed (in seconds)

# Initialize steering angle and throttle variables with default values
steering_angle = 0
throttle = 0

def loop_task_1(steering_angle, throttle):
    try:
        for event in controller_dev.read_loop():
            # Check for joystick events
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_X:
                    # Steering control
                    steering_angle = -event.value
                elif event.code == evdev.ecodes.ABS_RY:
                    # Forward throttle control (right trigger)
                    throttle = -event.value
           
                # Send control signals to Arduino Nano
                send_control_signals(steering_angle, throttle)
    except Exception as e:
        print("Exception in loop_task_1:", e)
    
    return steering_angle, throttle

def loop_task_2():
    try:
        while True:
            # Read a line from serial
            line = ser.readline().strip().decode('utf-8')
        
            # Split the line into voltage value and delimiter
            voltage_value, delimiter = line.split('|')
            voltage_value = float(voltage_value)*(10/1023)
            # Print the voltage value
            #print("Voltage:", voltage_value)
            start_time = time.time()
            GPIO.wait_for_edge(IR_PIN, GPIO.RISING, timeout=int(timeout_duration * 1000))
            end_time = time.time()
            if GPIO.input(IR_PIN):  # If pulse detected
                pulse_times.append(end_time)
                rpm = calculate_rpm()
                moving = True  # Set the flag to indicate that the wheel is moving
                #print("RPM:", rpm)
            elif moving:  # If no pulse detected and the wheel was moving
                #print("RPM: 0")
                moving = False  # Reset the flag to indicate that the wheel is stopped
                pulse_times = []  # Reset pulse times
            print("\033[H\033[J")  # Clear the terminal
			print(f"RPM : {Fore.GREEN}{rpm:.4f}{Style.RESET_ALL}")
            print(f"Voltage : {Fore.RED}{voltage_value:.4f}{Style.RESET_ALL}")
            time.sleep(0.1)
    except Exception as e:
        print("Exception in loop_task_1:", e)
#    except KeyboardInterrupt:
        # Close the serial port when Ctrl+C is pressed
 #       ser.close()

# Define the serial port for communication with Arduino Nano
SERIAL_PORT = '/dev/ttyUSB7'  # Change this to match your Arduino's serial port
BAUD_RATE = 9600

# Initialize serial communication with Arduino Nano
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# Find the event devices, you may need to change the name depending on the device
devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
for device in devices:
    print(device.fn, device.name)

# Replace '/dev/input/eventX' with the appropriate event device for your controller
# You may need to change this depending on your setup
controller_dev = evdev.InputDevice('/dev/input/event4')

# Map joystick axis ranges to servo angles and motor speeds
def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Function to send control signals to Arduino Nano
def send_control_signals(steering_angle, throttle):
    # Convert steering angle to servo pulse width (PWM)
    servo_pwm = int(map_range(steering_angle, -32768, 32767, 1000, 2000))
    # Convert throttle to PWM signal (50Hz, 1ms to 2ms)
    throttle_pwm = int(map_range(throttle, -32768, 32767, 1200, 1800))
    
    # Send control signals to Arduino Nano
    ser.write(f"{servo_pwm},{throttle_pwm}\n".encode())

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

# Main loop to read controller inputs and send control signals
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
    # Submit each loop task to the executor
    future1 = executor.submit(loop_task_1, steering_angle, throttle)
    future2 = executor.submit(loop_task_2)

    # Wait for all tasks to complete
    concurrent.futures.wait([future1, future2])


# Retrieve the modified values of steering_angle and throttle
steering_angle, throttle = future1.result()


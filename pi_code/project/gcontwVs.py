import evdev
import serial
import time
import concurrent.futures

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
        	line = ser.readline().decode().strip()
        
        	# Split the line into voltage value and delimiter
        	voltage_value, delimiter = line.split('|')
        
        	# Print the voltage value
        	print("Voltage:", voltage_value)
	except KeyboardInterrupt:
    	# Close the serial port when Ctrl+C is pressed
    	ser.close()

# Define the serial port for communication with Arduino Nano
SERIAL_PORT = '/dev/ttyUSB0'  # Change this to match your Arduino's serial port
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

# Main loop to read controller inputs and send control signals
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
    # Submit each loop task to the executor
    future1 = executor.submit(loop_task_1, steering_angle, throttle)
    future2 = executor.submit(loop_task_2)

    # Wait for all tasks to complete
    concurrent.futures.wait([future1, future2])


# Retrieve the modified values of steering_angle and throttle
steering_angle, throttle = future1.result()

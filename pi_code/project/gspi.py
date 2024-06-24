import evdev
import serial
#import smbus
import time
import concurrent.futures
import spidev
import RPi.GPIO as GPIO

# Define the GPIO pin used for Slave Select (SS) on the Raspberry Pi
SS_PIN = 8  # For example, using GPIO pin 8

# Define SPI parameters
SPI_CHANNEL = 0
SPI_DEVICE = 0
SPI_SPEED = 1000000

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_DEVICE, SPI_CHANNEL)
spi.max_speed_hz = SPI_SPEED

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
            # Read sensor data from Arduino Nano
            sensor_value = read_sensor_data()
            print("Received sensor value:", sensor_value)
        
            # Send control signals to Arduino Nano (Replace this with your control logic)
            # Example: servo_value = ... # Calculate servo value
            #          esc_value = ... # Calculate ESC value
            #          control_signal = f"{servo_value},{esc_value}\n"
            #          ser.write(control_signal.encode())
    
            time.sleep(1)  # Adjust delay as needed
    except Exception as e:
        print("Exception in loop_task_2:", e)

    
# Raspberry Pi's I2C bus (depends on which interface you used)
# bus = smbus.SMBus(1)  # Use 0 for I2C0, 1 for I2C1

# Arduino's I2C address
#arduino_address = 8

# Define the serial port for communication with Arduino Nano
SERIAL_PORT = '/dev/ttyUSB3'  # Change this to match your Arduino's serial port
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
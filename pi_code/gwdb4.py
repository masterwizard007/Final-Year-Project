import evdev
import serial
import time
import concurrent.futures
import RPi.GPIO as GPIO
from colorama import Fore, Style
import smbus
import sqlite3
import math

# Define GPIO pin for IR sensor input
IR_PIN = 20

# Initialize variables for RPM calculation
rpm = 0
previous_time = time.time()
pulse_count = 0

# Setup GPIO
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IR_PIN, GPIO.IN)

# Cleanup GPIO
def cleanup_gpio():
    GPIO.cleanup()

# Initialize SQLite database
def initialize_database():
    conn = sqlite3.connect('data.db')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE IF NOT EXISTS SensorData (
                        timestamp INTEGER PRIMARY KEY,
                        rpm REAL,
                        voltage REAL,
                        pitch REAL,
                        yaw REAL,
                        roll REAL
                    )''')
    conn.commit()
    conn.close()


# Read joystick inputs
def read_joystick_inputs(controller_dev):
    steering_angle = 0  # Initialize with default value
    throttle = 0  # Initialize with default value
    
    for event in controller_dev.read_loop():
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_X:
                steering_angle = -event.value
            elif event.code == evdev.ecodes.ABS_RY:
                throttle = -event.value
            yield steering_angle, throttle

# Map joystick axis ranges to servo angles and motor speeds
def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Send control signals to Arduino Nano
def send_control_signals(ser, steering_angle, throttle):
    servo_pwm = int(map_range(steering_angle, -32768, 32767, 1000, 2000))
    throttle_pwm = int(map_range(throttle, -32768, 32767, 1200, 1800))
    ser.write(f"{servo_pwm},{throttle_pwm}\n".encode())

# Print output
def print_output(rpm, voltage_value, pitch, yaw, roll):
    print("\033[H\033[J")  # Clear the terminal
    print(f"RPM : {Fore.GREEN}{rpm:.4f}{Style.RESET_ALL}")
    print(f"Voltage : {Fore.RED}{voltage_value:.4f}{Style.RESET_ALL}")
    print(f"Pitch: {Fore.BLUE}{pitch:.2f}{Style.RESET_ALL}, Yaw: {Fore.BLUE}{yaw:.2f}{Style.RESET_ALL}, Roll: {Fore.BLUE}{roll:.2f}{Style.RESET_ALL}")

# Read IR sensor data for RPM calculation
def read_ir_sensor_rpm():
    global rpm, pulse_count, previous_time
    try:
        while True:
            # Read IR sensor data
            GPIO.wait_for_edge(IR_PIN, GPIO.RISING)
            pulse_count += 1
            current_time = time.time()
            elapsed_time = current_time - previous_time
            if elapsed_time >= 1:  # Calculate RPM every 1 second
                rpm = (pulse_count / elapsed_time) * 60
                pulse_count = 0
                previous_time = current_time
    except Exception as e:
        print("Exception in read IR sensor data for RPM:", e)

# Read IR sensor data for voltage
def read_ir_sensor_voltage(ser):
    time.sleep(3)
    try:
        while True:
            line = ser.readline().strip().decode('utf-8')
            voltage_value, delimiter = line.split('|')
            voltage_value = float(voltage_value) * (10 / 1023)
            time.sleep(1)  # Adjust the sleep time if necessary
            pitch, yaw, roll = read_gyro_values()
            print_output(rpm, voltage_value, pitch, yaw, roll)
            save_to_database(rpm, voltage_value, pitch, yaw, roll)
    except Exception as e:
        print("Exception in read IR sensor data for voltage:", e)

# Read gyro values from MPU6050 and calculate pitch, yaw, and roll
def read_gyro_values():
    bus = smbus.SMBus(1)  # Change to 0 if you're using I2C0
    MPU6050_ADDRESS = 0x68  # MPU6050 I2C address
    MPU6050_GYRO_XOUT_H = 0x43  # Gyroscope data registers

    # Read raw gyro values
    gyro_xout = bus.read_byte_data(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H)
    gyro_yout = bus.read_byte_data(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H + 1)
    gyro_zout = bus.read_byte_data(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H + 2)

    # Convert raw values to degrees per second
    sensitivity = 131.0  # Sensitivity scale factor for gyro
    gyro_x = gyro_xout / sensitivity
    gyro_y = gyro_yout / sensitivity
    gyro_z = gyro_zout / sensitivity

    # Calculate pitch, yaw, and roll angles
    pitch = math.atan2(gyro_y, math.sqrt(gyro_x ** 2 + gyro_z ** 2)) * (180 / math.pi)
    yaw = math.atan2(gyro_x, math.sqrt(gyro_y ** 2 + gyro_z ** 2)) * (180 / math.pi)
    roll = math.atan2(gyro_z, math.sqrt(gyro_x ** 2 + gyro_y ** 2)) * (180 / math.pi)

    return pitch, yaw, roll

# Save data to SQLite database
def save_to_database(rpm, voltage_value, pitch, yaw, roll):
    conn = sqlite3.connect('data.db')
    cursor = conn.cursor()
    timestamp = int(time.time())
    cursor.execute('''INSERT INTO SensorData (timestamp, rpm, voltage, pitch, yaw, roll)
                      VALUES (?, ?, ?, ?, ?, ?)''',
                   (timestamp, rpm, voltage_value, pitch, yaw, roll))
    conn.commit()
    conn.close()

# Main function
def main():
    try:
        # Initialize GPIO
        setup_gpio()

        # Initialize serial communication with Arduino Nano for voltage reading
        SERIAL_PORT = '/dev/ttyUSB0'
        BAUD_RATE = 9600
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

        # Initialize SQLite database
        initialize_database()

        # Find the event devices
        devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
        for device in devices:
            print(device.fn, device.name)

        # Replace '/dev/input/eventX' with the appropriate event device for your controller
        controller_dev = evdev.InputDevice('/dev/input/event4')

        # Main loop to read controller inputs and send control signals
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            future1 = executor.submit(read_joystick_inputs, controller_dev)
            future2 = executor.submit(read_ir_sensor_rpm)
            future3 = executor.submit(read_ir_sensor_voltage, ser)

            for steering_angle, throttle in future1.result():
                send_control_signals(ser, steering_angle, throttle)
    
    except KeyboardInterrupt:
        cleanup_gpio()
        ser.close()

if __name__ == "__main__":
    main()


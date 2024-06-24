import smbus

def scan_i2c():
    bus = smbus.SMBus(0)  # Initialize I2C bus (depends on your Raspberry Pi)
    devices = []

    print("Scanning I2C bus...")
    for address in range(1, 127):
        try:
            bus.read_byte(address)
            devices.append(address)
            print(f"I2C device found at address: 0x{address:02X}")
        except Exception:
            pass

    if not devices:
        print("No I2C devices found.")
    else:
        print("Scan complete.")

    bus.close()

if __name__ == "__main__":
    scan_i2c()

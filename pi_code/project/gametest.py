import evdev

# Find the event devices, you may need to change the name depending on the device
devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
for device in devices:
    print(device.fn, device.name)

# Replace '/dev/input/eventX' with the appropriate event device for your controller
# You may need to change this depending on your setup
dev = evdev.InputDevice('/dev/input/event4')

# Function to decode the input event
def decode_input_event(event):
    if event.type == evdev.ecodes.EV_KEY:
        return "Key event: {}".format(evdev.ecodes.KEY[event.code])
    elif event.type == evdev.ecodes.EV_ABS:
        return "Absolute event: {} - {}".format(evdev.ecodes.ABS[event.code], event.value)
    elif event.type == evdev.ecodes.EV_REL:
        return "Relative event: {} - {}".format(evdev.ecodes.REL[event.code], event.value)
    else:
        return "Unknown event type: {} - {}".format(event.type, event.code)

# Print out events from the controller
for event in dev.read_loop():
    print(decode_input_event(event))

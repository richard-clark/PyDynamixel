"""
The following code can be used to turn on the LED on a connected servo 
(on a POSIX-compliant platform.)
"""

from pydynamixel import dynamixel, registers

# You'll need to change this to the serial port of your USB2Dynamixel
serial_port = '/dev/tty.usbserial-A921X77J'

# You'll need to change this to the ID of your servo
servo_id = 9

# Turn the LED on
led_value = registers.LED_STATE.ON
    
try:
    ser = dynamixel.get_serial_for_url(serial_port)
    dynamixel.set_led(ser, servo_id, led_value)
    print('LED set successfully!')
except Exception as e:
    print('Unable to set LED.')
    print(e)
"""
This example simply moves a specified servo to a specified position. 
"""

from pydynamixel import dynamixel

# You'll need to change this to the serial port of your USB2Dynamixel
serial_port = '/dev/tty.usbserial-A921X77J'

# You'll need to modify these for your setup
servo_id = 9
target_position = 768 # (range: 0 to 1023)

# If this is the first time the robot was powered on, 
# you'll need to read and set the current position.
# (See the documentation above.)
first_move = True

try:
    ser = dynamixel.get_serial_for_url(serial_port)
    
    if first_move == True:
        dynamixel.init(ser, servo_id)
    
    dynamixel.set_position(ser, servo_id, target_position)
    dynamixel.send_action_packet(ser)
    
    print('Success!')
    
except Exception as e:
    print('Unable to move to desired position.')
    print(e)
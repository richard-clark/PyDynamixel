"""
The following moves a servo to a target position and very slow velocity,
and waits for the servo to complete moving before turning on the LED and
printing a "Done moving!" message.

"""

from pydynamixel import dynamixel, registers
import time

# You'll need to change this to the serial port of your USB2Dynamixel
serial_port = '/dev/tty.usbserial-A921X77J'

# You'll need to modify this for your setup
servo_id = 9

target_position = 0
velocity = 20 # very slow

# If this is the first time the robot was powered on, we need to read 
# and set the current position.
# (See the documentation above.)
first_move = True

try:
    ser = dynamixel.get_serial_for_url(serial_port)
    
    # Turn the LED off
    dynamixel.set_led(ser, servo_id, registers.LED_STATE.OFF)
    
    if first_move == True:
        dynamixel.init(ser, servo_id)
    
    # Set the desired position
    dynamixel.set_position(ser, servo_id, target_position)
    
    # Set the velocity
    dynamixel.set_velocity(ser, servo_id, velocity)
    
    # Move to the desired position
    dynamixel.send_action_packet(ser)
    
    # Wait for the arm to stop moving
    print('Waiting...')
    
    # Loop until the robot is done moving.
    while True:
        if dynamixel.get_is_moving(ser, servo_id) == False:
            break
            
        time.sleep(0.1) # Sleep for a short amount of time (100 ms)
        
    # Done moving
    print('Done moving!')
    
    dynamixel.set_led(ser, servo_id, registers.LED_STATE.ON)
    
except Exception as e:
    print('ERROR!')
    print(e)
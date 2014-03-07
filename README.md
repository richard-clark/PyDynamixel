# PyDynamixel

A Python library for controlling Dynamixel servos.

This library contains two modules:

* `dynamixel` allows communication with a single Dynamixel servo connected to the bus. 
* `chain` is for manipulating kinematic chains of multiple Dynamixels.

## Examples

The following example turns on the red LED of a servo with id 1 located (on a POSIX-
compliant system) at `/dev/tty.usbserial-A9C73L15`:

	    from pydynamixel import dynamixel
	
	    try:
	        url = '/dev/tty.usbserial-A9C73L15'
	        servo_id = 1
	        num_attempts = 10
	        led_value = 1 # Red
	        
	        ser = dynamixel.get_serial_for_url(url)
	        packet = get_led_packet(servo_id, led_value)
	        write_and_get_response_multiple(ser, packet, num_attempts, led_value)
	        print('LED set successfully!')
	    except Exception as e:
	        print('Unable to set LED.')
	        print(e)
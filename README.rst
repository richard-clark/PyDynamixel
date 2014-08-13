A Python library for controlling Dynamixel servos.

Installation
============

This package requires pyserial, whicah can be installed using:

::

	pip install pyserial
	
or, using another method described on the documentation page:

	http://pythonhosted.org//pyserial/pyserial.html#installation

The PyDynamixel package itself can be installed using pip:

::

	pip install pydynamixel 

The source code (with examples) is available from the GitHub repository:

	https://github.com/richard-clark/pydynamixel

It can also be installed by cloning into the source and running ``setup.py``:
	
::
	
	git clone https://github.com/richard-clark/pydynamixel
	cd pydynamixel
	python setup.py install

Communication
=============

The ``get_serial_for_url`` method can be used to get a ``serial`` object correctly
configured for the specified url (on POSIX systems; Windows users can use
``get_serial_for_com``).

By default, Dynamixels comminate at a high baud rate (1,000,000 baud), and use a single-wire
protocol. This combination is highly susceptible to noise. For this reason, the
``write_and_get_response`` multiple function is recommended as the preferred way of commincating
with the Dynamixel. This function will make multiple arguments to clear the serial port, send the
packet, and read a valid response before failing.

Servo Initialization
====================

The first time that a servo is instructed to move to a specified position after it has been 
powered up, it will do so at the maximum speed possible, regardless of whether the velocity 
has been set. This is dangerous, as the servos are quite powerful.

This issue can be mitigated by first reading the current position of the servo, and then 
commanding the servo to move to that same position. The ``init()`` method performs this function.

Basic LED Example
=================

The AX-18A servos have integrated LEDs. By default, these LEDs are off. The following code 
can be used to turn on the LED on a connected servo (on POSIX-compliant platforms, 
such as Linux and OSX). 

::

    from pydynamixel import dynamixel, registers

    # You'll need to change this to the serial port of your USB2Dynamixel
    serial_port = '/dev/tty.usbserial-A921X77J'

    # You'll need to change this to the ID of your servo
    servo_id = 9

    # Turn the LED on
    led_value = dynamixel.registers.LED_STATE.ON
    
    try:
        ser = dynamixel.get_serial_for_url(serial_port)
        dynamixel.set_led(ser, servo_id, led_value)
        print('LED set successfully!')
    except Exception as e:
        print('Unable to set LED.')
        print(e)
        
To perform the same function on Windows, use the following:

::

    from pydynamixel import dynamixel, registers

    # You'll need to change this to the serial port of your USB2Dynamixel
    com_port = 'COM5'

    # You'll need to change this to the ID of your servo
    servo_id = 9

    # Turn the LED on
    led_value = dynamixel.registers.LED_STATE.ON
    
    try:
        ser = dynamixel.get_serial_for_url(serial_port)
        dynamixel.set_led(ser, servo_id, led_value)
        print('LED set successfully!')
    except Exception as e:
        print('Unable to set LED.')
        print(e)
        
Motion Example
==============

This example simply moves a specified servo to a specified position.

::

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
       

Chain Module
============

Multiple servos can be controlled more easily using the chain module.

The following example demonstrates the process of manipulating the robot 
using a series of frames using the same velocity for each servo. Each item
in ``pos`` contains a list of positions, each one corresponding with a
servo id. After instructing each joint to move to the specified position,
the program waits until alls ervos have finished moving before moving to the
next frame.

::

    from pydynamixel import chain, dynamixel

    servo_ids = [1, 2, 3, 4, 5, 6, 7]
    velocity = 180
    
    # Initialize the servos
    chain.init(ser, servo_ids, velocity)
    
    # A list of frames each consisting of the target 
    # displacements for each joint
    pos = [[822, 94, 929, 919, 104, 820, 691],
        [822, 632, 391, 919, 104, 523, 561],
        [822, 640, 383, 911, 112, 516, 650],
        [822, 100, 923, 918, 105, 538, 650],
        [818, 100, 923, 918, 105, 538, 650],
        [495, 100, 923, 918, 105, 538, 650],
        [495, 714, 309, 802, 221, 538, 650],
        [495, 723, 300, 791, 232, 538, 569],
        [495, 103, 920, 916, 107, 538, 571],
        [495, 103, 920, 916, 107, 538, 571],
        [495, 723, 300, 791, 232, 538, 569],
        [495, 714, 309, 802, 221, 538, 650],
        [495, 100, 923, 918, 105, 538, 650],
        [818, 100, 923, 918, 105, 538, 650],
        [822, 100, 923, 918, 105, 538, 650],
        [822, 640, 383, 911, 112, 516, 650],
        [822, 632, 391, 919, 104, 523, 561],
        [822, 94, 929, 919, 104, 820, 691]]
  
    # Iterate over the vectors, move to each, 
    # and wait for each move to complete
    for v in pos:
        vector = chain.make_vector_constant_velocity(v, servo_ids, velocity)
        chain.move_to_vector(ser, vector)
        chain.wait_for_move(ser, joints)
      
Fore each vector, after instructing the joints to move to the specified position, 
the program waits until all servos have finished moving before moving to the next frame.

Further Documentation
=====================

For further documentation, see

	http://richard-h-clark.com/tag/pydynamixel

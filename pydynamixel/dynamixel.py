"""

PyDynamixel
===========

A Python library for controlling Dynamixel servos.

Communication
-------------

Communication between Python and Dynamixel servos occurs using the Python
``serial`` library. Data is exchanged using packets.

The ``get_serial_for_url`` method can be used to get a ``serial`` object correctly
configured for the specified url (on POSIX systems; Windows users can use
``get_serial_for_com``).

By default, Dynamixels comminate at a high baud rate (1,000,000 baud), and use a single-wire
protocol. This combination is highly susceptible to noise. For this reason, the
``write_and_get_response`` multiple function is recommended as the preferred way of commincating
with the Dynamixel. This function will make multiple arguments to clear the serial port, send the
packet, and read a valid response before failing.

Servo Initialization [initialization]
-------------------------------------

The first time that a servo is instructed to move to a specified position after it has been 
powered up, it will do so at the maximum speed possible, regardless of whether the velocity 
has been set. This is dangerous, as the servos are quite powerful.

This issue can be mitigated by first reading the current position of the servo, and then 
commanding the servo to move to that same position. The ``init()`` method performs this function.

Basic LED Example
-----------------

The AX-18A servos have integrated LEDs. By default, these LEDs are off. The following code 
can be used to turn on the LED on a connected servo (on POSIX-compliant platforms, 
such as Linux and OSX). 

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
--------------

This example simply moves a specified servo to a specified position.

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

"""

import serial
import struct
import registers
import packets

# The number of times to attempt to send a packet before raising an Exception.
NUM_ERROR_ATTEMPTS = 10

VERBOSE = True

# The ID to use to broadcast to all servos.
BROADCAST = 254

# The default baudrate of the serial interface.
BAUDRATE = 1000000

# The default timeout to use when reading from the serial interface.
TIMEOUT = 0.1

### Serial Helper Functions

def get_serial_for_url(url, baudrate = BAUDRATE, timeout = TIMEOUT):
    """
    Open the servo at the specified path with the correct parameters for the Dynamixel. 
    
    :param url: The path to the ``serial`` port. 
    :param baudrate: The baudrate to to which to configure the servo. (Default: ``BAUDRATE``.)
    :param timeout: The timeout to set for the servo. (Default: ``TIMEOUT``.)
        
    :returns: A ``serial`` object.
    """
    
    ser = serial.serial_for_url(url)
    ser.baudrate = BAUDRATE
    ser.timeout = TIMEOUT
    return ser

def get_serial_for_com(com, baudrate = BAUDRATE, timeout = TIMEOUT):
    """
    Open the servo at the specified path with the correct parameters for the Dynamixel. 
    
    :param url: The path to the ``serial`` port. 
    :param baudrate: The baudrate to to which to configure the servo. (Default: ``BAUDRATE``.)
    :param timeout: The timeout to set for the servo. (Default: ``TIMEOUT``.)
        
    :returns: A ``serial`` object.
    """
    
    ser = serial.Serial(com)
    ser.baudrate = BAUDRATE
    ser.timeout = TIMEOUT
    return ser

def flush_serial(ser):
    """
    Clear any pending bytes from the ``serial`` buffer. 
    
    :param ser: The ``serial`` object to use. 
        
    :returns: ``None``.
    
    """
    while ser.inWaiting() > 0:
        ser.read()

### Response Packet Handling

def get_error_string(error):
    """ 
    Get a string to describe a Dynamixel error.
    
    :param error: A string that represents the error returned in a response packet.
    
    :returns: A string describing an error, or ``None`` if the error is undefined.
    
    """
    errors = []
    
    if error & registers.ERROR_BIT_MASKS.INPUT_VOLTAGE > 0:
        errors.append('input voltage error')
    elif error & registers.ERROR_BIT_MASKS.ANGLE_LIMIT > 0:
        errors.append('angle limit error')
    elif error & registers.ERROR_BIT_MASKS.OVERHEATING > 0:
        errors.append('motor overheating')
    elif error & registers.ERROR_BIT_MASKS.RANGE > 0:
        errors.append('range error')
    elif error & registers.ERROR_BIT_MASKS.SEND_CHECKSUM > 0:
        errors.append('checksum mismatch')
    elif error & registers.ERROR_BIT_MASKS.OVERLOAD > 0:
        errors.append('motor overloaded')
    elif error & registers.ERROR_BIT_MASKS.INSTRUCTION > 0:
        errors.append('instruction error')
    
    if len(errors) == 0:
        return None
    elif len(errors) == 1:
        return errors[0][0].upper() + errors[0][1:]
    else:
        s = errors[0][0].upper() + errors[0][1:]
    
        for i in range(1, len(errors) - 1):
            s += ', ' + errors[i][0].upper() + errors[i][1:]
        
        s += ' and ' + errors[-1][0].upper() + errors[-1][1:]
    
        return s
    
class DynamixelFatalError(Exception):
    def __init__(self, value):
        self.value = value
        
    def __str__(self):
        return self.value

def get_exception(error_code):
    if error_code == registers.ERROR_BIT_MASKS.SEND_CHECKSUM:
        return Exception('Send checksum mismatch.')
    else:
        return DynamixelFatalError(get_error_string(error_code))
    

class Response:
    """
    Holds the response received from the dynamixel
    """
    def __init__(self, servo_id, error, data, checksum_match):
        self.servo_id = servo_id
        self.error = error
        self.data = data
        self.checksum_match = checksum_match
        
    def get_error(self):
        """
        Determines whether the packet indicates an error. 
        
        :returns: A Boolean value indicating ``True`` if an error occured, 
            or ``False`` otherwise.
        """
        return self.error > 0 or self.checksum_match == False
        
    def get_error_str(self):
        """
        Get a text string describing one of the errors that the packet indicates. 
        
        :returns: A string describing an error specified by the packet, or ``None``
            if no error occured. 
        
        """
        
        if self.error > 0:
            return get_error_string(self.error)
        elif self.checksum_match == False:
            return 'Checksum mismatch.'
        else:
            return None

### Send Data and Read Response Functions

def get_response(ser):
    """
    Attempt to read a response packet. 
    
    Throws an exception if a ``serial`` timeout occurs.
    
    :param ser: The ``serial`` object to use. 
    :raises: ``Exception`` if a ``serial`` timeout occurs.
    :returns: A ``Response`` object.
    
    """
    
    data = []
    
    byte_sum = 0
    
    last_byte = None
    while True:
        data = ser.read()
        if data == '':
            raise Exception('Unable to read response header.')
        data_byte = struct.unpack('B', data)[0]
        if data_byte == 0xFF and last_byte == 0xFF:
            break
        last_byte = data_byte
        
    id_str = ser.read()
    if id_str == '':
        raise Exception('Unable to read response id.')
    servo_id = struct.unpack('B', id_str)[0]
    byte_sum += servo_id
    
    length_str = ser.read()
    if length_str == '':
        raise Exception('Unable to read length.')
    length = struct.unpack('B', length_str)[0]
    byte_sum += length
    
    error_str = ser.read()
    if error_str == '':
        raise Exception('Unable to read error.')
    error = struct.unpack('B', error_str)[0]
    byte_sum += error
    
    data = None
    
    if length > 2:
        data_str = ser.read(length-2)
        if data_str == None or len(data_str) < length-2:
            raise Exception('Unable to read response data.')
        data = []
        for d in data_str:
            b = struct.unpack('B', d)[0]
            data.append(b)
            byte_sum += b
    
    calc_checksum = (~byte_sum) & 0xFF
    
    checksum_str = ser.read()
    if checksum_str == None:
        raise Exception('Unable to read response checksum.')
    checksum = struct.unpack('B', checksum_str)[0]
    
    if checksum != calc_checksum:
        raise Exception('Checksum mismatch ({0} vs {1}).'.format(checksum, calc_checksum))
    
    return Response(servo_id, error, data, calc_checksum == checksum)
    

def write_and_get_response_multiple(ser, packet, servo_id = None, verbose = VERBOSE, attempts = NUM_ERROR_ATTEMPTS):
    """
    Send the specified packet, and make up to ``attempts`` attempts to read
    a successful response. 
    
    A successful response is one in which the checksum matches, the ``error`` byte
    is zero, and the ``servo_id`` matches.
    
    :param ser: The ``serial`` object to use. 
    :param packet: The packet to send.
    :param servo_id: The id of the servo, or ``None`` if the id does not matter. (Default: ``None``.)
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
        
    :raises: An ``Exception`` if no packet is successfully read after ``attempts`` attempts, or
    a ``DynamixelFatalError`` if another error occurs.
    
    :returns: A ``Response`` object.
    
    """

    for i in range(attempts):
        try:
            flush_serial(ser)
            ser.write(packet)
            response = get_response(ser)
            
            if servo_id != None and response.servo_id != servo_id:
                raise Exception('Got packet from {0}, expected {1}.'.format(response.servo_id, servo_id))
            
            if response.checksum_match == False:
                raise Exception('Checksum mismatch.')
            
            if response.error > 0:
                raise get_exception(response.error)
                
            return response
                
        except DynamixelFatalError as d:
            raise d
                
        except Exception as e:
            if verbose:
                print('Got exception when waiting for response from {0} on attempt {1}: {2}'.format(servo_id, i, e))
            
    raise Exception('Unable to read response for servo {0}'.format(servo_id))
   
def set_led(ser, servo_id, value, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Set the LED (to either LED_ON or LED_OFF).
    
    :param ser: The ``serial`` object to use. 
    :param servo_id: The id of the servo.
    :param value: The LED value to set (either LED_ON or LED_OFF).
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
        
    :raises: An ``Exception`` if no packet is successfully read after ``attempts`` attempts.
    """
    
    packet = packets.get_write_led_packet(servo_id, value)
    write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)
   
def get_torque(ser, servo_id, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Read the current torque as reported by a servo.
    
    :param ser: The ``serial`` object to use. 
    :param servo_id: The id of the servo.
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
        
    :raises: An ``Exception`` if no packet is successfully read after ``attempts`` attempts.
    
    :returns: An integer indicating the reported torque.
    """
    
    torque_packet = packets.get_read_torque_packet(servo_id)
    resp = write_and_get_response_multiple(ser, torque_packet, servo_id, verbose, num_error_attempts)
    return resp.data[1] * 256 + resp.data[0]

def get_position(ser, servo_id, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Read the current position as reported by a servo.
    
    :param ser: The ``serial`` object to use. 
    :param servo_id: The id of the servo.
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
        
    :raises: An ``Exception`` if no packet is successfully read after ``attempts`` attempts.
    
    :returns: An integer indicating the reported position.
    """
    
    packet = packets.get_read_position_packet(servo_id)
    resp = write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)
    return resp.data[1] * 256 + resp.data[0]

def set_position(ser, servo_id, position, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Set the taget position of a servo (the servo will not move until an action packet is sent).
    
    :param ser: The ``serial`` object to use. 
    :param servo_id: The id of the servo.
    :param position: The position to set, (0, 1023). 
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
        
    :raises: An ``Exception`` if no packet is successfully read after ``attempts`` attempts.
    """
    
    packet = packets.get_write_position_packet(servo_id, position)
    write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)

def set_velocity(ser, servo_id, velocity, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Set the taget velocity of a servo (the servo will not move until an action packet is sent).
    
    :param ser: The ``serial`` object to use. 
    :param servo_id: The id of the servo.
    :param position: The velocity to set, (0, 1023). 
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
        
    :raises: An ``Exception`` if no packet is successfully read after ``attempts`` attempts.
    """
    
    packet = packets.get_write_velocity_packet(servo_id, velocity)
    write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)

def init(ser, servo_id, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    "Initialize" a servo by reading its current position and then writing that position.
    
    :param ser: The ``serial`` object to use. 
    :param servo_id: The id of the servo.
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
        
    :raises: An ``Exception`` if no packet is successfully read after ``attempts`` attempts.
    
    """
    
    position = get_position(ser, servo_id, verbose, num_error_attempts)
    set_position(ser, servo_id, position, verbose, num_error_attempts)
    ser.write(packets.get_action_packet())
    
def send_action_packet(ser):
    """
    Send an action packet, which causes all servos to move to their previously-specified
    target positions.
    
    :param ser: The ``serial`` object to use. 
        
    :raises: An ``Exception`` if no packet is successfully read after ``attempts`` attempts.
    
    """
    ser.write(packets.get_action_packet())

def get_is_moving(ser, servo_id, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Returns a boolean value indicating whether the servo is currently moving.
    
    :param ser: The ``serial`` object to use. 
    :param servo_id: The id of the servo.
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
        
    """
    packet = packets.get_read_is_moving_packet(servo_id)
    resp = write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)
    return resp.data[0] != 0
    
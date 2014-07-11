"""

PyDynamixel
***********

A Python library for controlling Dynamixel servos.

Communication
-------------

Communication between Python and Dynamixel servos occurs using the Python
``serial`` library. Data is exchanged using packets.

The helper method :func:`flush_serial` is used to clear any pending data from
the receive buffer.

The :func:`get_serial_for_url` method can be used to get a ``serial`` object correctly
configured for the specified url.

By defualt, Dynamixels comminicate at a high baud rate (1,000,000 baud), and use a single-wire
protocol. This combination is highly susceptible to noise. For this reason, the 
:func:`write_and_get_response_multiple` function is recommended as the preferred way 
of communicating with the Dynamixel. This function will make multiple attempts to clear
the serial port, send the packet, and read a valid response packet before failing.

Packets
-------

Data is exchaned between the computer and the servo using packets. 

All packets have the following format:

    [0xFF] [0xFF] [data1] [data2] ... [dataN] [checksum]
    
where ``checksum`` is the complement of the lowest byte of the sum of the data bytes.

A number of methods are provided for handling packets.


"""

import serial
import struct

NUM_ERROR_ATTEMPTS = 10
SLEEP_TIME = 0.1
VERBOSE = True

BROADCAST = 254

BAUDRATE = 1000000
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

ERROR_INSTRUCTION = 0x40
ERROR_OVERLOAD = 0x20
ERROR_SEND_CHECKSUM = 0x10
ERROR_RANGE = 0x08
ERROR_OVERHEATING = 0x04
ERROR_ANGLE_LIMIT = 0x02
ERROR_INPUT_VOLTAGE = 0x01

def get_error_string(error):
    """ 
    Get a string to describe a Dynamixel error.
    
    :param error: A string that represents the error returned in a response packet.
    
    :returns: A string describing an error, or ``None`` if the error is undefined.
    
    """
    errors = []
    
    if error & ERROR_INPUT_VOLTAGE > 0:
        errors.append('input voltage error')
    elif error & ERROR_ANGLE_LIMIT > 0:
        errors.append('angle limit error')
    elif error & ERROR_OVERHEATING > 0:
        errors.append('motor overheating')
    elif error & ERROR_RANGE > 0:
        errors.append('range error')
    elif error & ERROR_SEND_CHECKSUM > 0:
        errors.append('checksum mismatch')
    elif error & ERROR_OVERLOAD > 0:
        errors.append('motor overloaded')
    elif error & ERROR_INSTRUCTION > 0:
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
    if error_code == ERROR_SEND_CHECKSUM:
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
        
### Packet-Formatting Functions

def get_packet(bytes_in):
    """
    Get a properly formatted Dynamixel packet with header and checksum. 
    
    :param bytes_in: A list of bytes in the form ``[b0, b1, b2, ..., bn]``
        
    :returns: A packet in the form of a list of bytes.
    """
    bytes = [0xFF, 0xFF] + bytes_in

    sum = 0
    
    for i in range(2,len(bytes)):
        b = bytes[i]
        sum += b
    
    sum = (~sum) & 0xFF
    bytes.append(sum)
    
    return bytes

def get_position_packet(id, angle):
    """
    Get a packet to set the position of a servo. 
    
    An action packet must be sent to actually move the servo.
    
    :param id: The id of the servo. 
    :param angle: The angle to which to set the servo. 
    :returns: A packet, in the form of a list of bytes. 
    
    """

    angle_msb = (angle >> 8) & 0xFF
    angle_lsb = angle & 0xFF
    
    bytes = [id, 0x05, 0x04, 0x1E, angle_lsb, angle_msb]
    packet = get_packet(bytes)
    return packet

def get_velocity_packet(id, angular_velocity):
    """
    Get a packet to set the position of a servo. 
    
    An action packet must be sent to actually move the servo.
    
    :param id: The id of the servo. 
    :param velocity: The velocity at which to move the servo. 
        
    :returns: A packet, in the form of a list of bytes. 
    
    """
    
    vel_msb = (angular_velocity >> 8) & 0xFF
    vel_lsb = angular_velocity & 0xFF
    
    bytes = [id, 0x05, 0x04, 0x20, vel_lsb, vel_msb]
    packet = get_packet(bytes)
    return packet

def get_position_and_velocity_packet(id, angle, velocity):
    """
    Get a packet to set the position of a servo at a specified velocity. 
    
    An action packet must be sent to actually move the servo.
    
    :param id: The id of the servo. 
    :param angle: The angle to which to move the servo. 
    :param velocity: The velocity at which to move the servo. 
    :returns: A packet, in the form of a list of bytes. 
    
    """
    
    angle_msb = (angle >> 8) & 0xFF
    angle_lsb = angle & 0xFF
    
    vel_msb = (velocity >> 8) & 0xFF
    vel_lsb = velocity & 0xFF
    
    bytes = [id, 0x05, 0x04, 0x20, angle_lsb, angle_msb, vel_lsb, vel_msb]
    packet = get_packet(bytes)
    return packet

def get_led_packet(id, value):
    """
    Get a packet to set the LED state of a servo. 
    
    :param id: The id of the servo. 
    :param value: The value for the LED register (0 is off, 1 is on).
    :returns: A packet, in the form of a list of bytes. 
    
    """
    
    return get_write_packet_1b(id, 0x19, value)

def get_action_packet():
    """
    Get a packet which is sent to all servos to cause the servos to
    move to the specified position. 
    
    :returns: A packet, in the form of a list of bytes. 
    
    """
    
    bytes = [0xFE, 0x02, 0x05]
    packet = get_packet(bytes)
    return packet

def get_write_packet_1b(id, register, data):
    """Return a packet which will write two bytes to a register.
    
    :param id: The identifier of the servo to address. 
    :param register: The address of the register to write.
    :param data: The data to write.
    :returns: A command packet in the form of a list of bytes. 
    
    """
    
    bytes = [id, 0x03 + 1, 0x03, register] + [data]
    packet = get_packet(bytes)
    return packet


def get_write_packet_2b(id, register, data):
    """Return a packet which will write two bytes to a register.
    
    :param id: The identifier of the servo to address. 
    :param register: The address of the register to write.
    :param data: The data to write.
    :returns: A command packet in the form of a list of bytes. 
    
    """
    
    msb = (data >> 8) & 0xFF
    lsb = data & 0xFF
    
    bytes = [id, 0x03 + 2, 0x03, register] + [lsb, msb]
    packet = get_packet(bytes)
    return packet

def get_read_packet(id, register, num_bytes = 2):
    """Return a read register packet. 
    
    :param id: The identifier of the servo to address.
    :param register: The address of the register to read. 
    :param num_bytes: The number of bytes to read. (The default is 2.)
    :returns: A command packet in the form of a list of bytes.
    
    """
    bytes = [id, 0x02 + num_bytes, 0x02, register, num_bytes]
    packet = get_packet(bytes)
    return packet

### Send Data and Read Response Functions

def get_response(ser):
    """
    Attempt to read a response packet. 
    
    Throws an exception if a ``serial`` timeout occurs.
    
    :param ser: The ``serial`` object to use. 
    :raises: ``Exception`` if a ``serial`` timeout occurs.
    :returns: A ``Response`` object.
    
    """
    
    id = None
    data = []
    
    sum = 0
    
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
    id = struct.unpack('B', id_str)[0]
    sum += id
    
    length_str = ser.read()
    if length_str == '':
        raise Exception('Unable to read length.')
    length = struct.unpack('B', length_str)[0]
    sum += length
    
    error_str = ser.read()
    if error_str == '':
        raise Exception('Unable to read error.')
    error = struct.unpack('B', error_str)[0]
    sum += error
    
    data = None
    
    if length > 2:
        data_str = ser.read(length-2)
        if data_str == None or len(data_str) < length-2:
            raise Exception('Unable to read response data.')
        data = []
        for d in data_str:
            b = struct.unpack('B', d)[0]
            data.append(b)
            sum += b
    
    calc_checksum = (~sum) & 0xFF
    
    checksum_str = ser.read()
    if checksum_str == None:
        raise Exception('Unable to read response checksum.')
    checksum = struct.unpack('B', checksum_str)[0]
    
    if checksum != calc_checksum:
        raise Exception('Checksum mismatch ({0} vs {1}).'.format(checksum, calc_checksum))
    
    return Response(id, error, data, calc_checksum == checksum)
    

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
        
    :raises: An ``Exception`` if no packet is successfully read after ``attempts`` attempts.
    
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
   
LED_ON = 1
LED_OFF = 0
   
def set_led(ser, servo_id, value, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    packet = get_led_packet(servo_id, value)
    write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)
   
def get_torque(ser, servo_id, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    torque_packet = get_read_packet(servo_id, 0x28, 2)
    resp = write_and_get_response_multiple(ser, torque_packet, servo_id, verbose, num_error_attempts)
    return resp.data[1] * 256 + resp.data[0]

def get_position(ser, servo_id, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    packet = get_read_packet(servo_id, 0x24, 2)
    resp = write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)
    return resp.data[1] * 256 + resp.data[0]

def set_position(ser, servo_id, position, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    packet = get_write_packet_2b(servo_id, 0x1E, position)
    write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)

def set_velocity(ser, servo_id, velocity, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    packet = get_write_packet_2b(servo_id, 0x20, velocity)
    write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)

def init(ser, servo_id, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    position = get_position(ser, servo_id, verbose, num_error_attempts)
    set_position(ser, servo_id, position, verbose, num_error_attempts)
    ser.write(get_action_packet())
    
def send_action_packet(ser):
    ser.write(get_action_packet())

def get_is_moving(ser, servo_id, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    packet = get_read_packet(servo_id, 0x2E, 2)
    resp = write_and_get_response_multiple(ser, packet, servo_id, verbose, num_error_attempts)
    return resp.data[0] != 0
    
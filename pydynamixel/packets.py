"""
Packets
=======

Contains utilities for creating packets for reading and writing to and from
Dynamixels.  

Data is exchaned between the computer and the servo using packets. 

All packets have the following format:

    [0xFF] [0xFF] [data1] [data2] ... [dataN] [checksum]
    
where ``checksum`` is the complement of the lowest byte of the sum of the data bytes.

"""

import registers
        
### Packet-Formatting Functions

def get_packet(bytes_in):
    """
    Get a properly formatted Dynamixel packet with header and checksum. 
    
    :param bytes_in: A list of bytes in the form ``[b0, b1, b2, ..., bn]``
        
    :returns: A packet in the form of a list of bytes.
    """
    packet_bytes = [0xFF, 0xFF] + bytes_in

    checksum = 0
    
    for i in range(2,len(packet_bytes)):
        b = packet_bytes[i]
        checksum += b
    
    checksum = (~checksum) & 0xFF
    bytes.append(checksum)
    
    return packet_bytes

def get_write_position_packet(servo_id, position):
    """
    Get a packet to set the position of a servo. 
    
    An action packet must be sent to actually move the servo.
    
    :param servo_id: The id of the servo.
    :param position: The position to set, (0, 1023). 
    :returns: A packet, in the form of a list of bytes. 
    
    """

    return get_write_packet_2b(servo_id, registers.GOAL_POSITION, position)

def get_write_velocity_packet(servo_id, velocity):
    """
    Get a packet to set the position of a servo. 
    
    An action packet must be sent to actually move the servo.
    
    :param servo_id: The id of the servo.
    :param position: The velocity to set, (0, 1023). 
        
    :returns: A packet, in the form of a list of bytes. 
    
    """
    
    return get_write_packet_2b(servo_id, registers.MOVING_SPEED, velocity)

def get_write_led_packet(servo_id, value):
    """
    Get a packet to set the LED state of a servo. 
    
    :param id: The id of the servo. 
    :param value: The value for the LED register (0 is off, 1 is on).
    :returns: A packet, in the form of a list of bytes. 
    
    """
    
    return get_write_packet_1b(servo_id, registers.LED, value)

def get_action_packet():
    """
    Get a packet which is sent to all servos to cause the servos to
    move to the specified position. 
    
    :returns: A packet, in the form of a list of bytes. 
    
    """
    
    packet_bytes = [0xFE, 0x02, 0x05]
    packet = get_packet(packet_bytes)
    return packet

def get_write_packet_1b(servo_id, register, data):
    """Return a packet which will write two bytes to a register.
    
    :param id: The identifier of the servo to address. 
    :param register: The address of the register to write.
    :param data: The data to write.
    :returns: A command packet in the form of a list of bytes. 
    
    """
    
    packet_bytes = [servo_id, 0x03 + 1, 0x03, register] + [data]
    packet = get_packet(packet_bytes)
    return packet


def get_write_packet_2b(servo_id, register, data):
    """Return a packet which will write two bytes to a register.
    
    :param id: The identifier of the servo to address. 
    :param register: The address of the register to write.
    :param data: The data to write.
    :returns: A command packet in the form of a list of bytes. 
    
    """
    
    msb = (data >> 8) & 0xFF
    lsb = data & 0xFF
    
    packet_bytes = [servo_id, 0x03 + 2, 0x03, register] + [lsb, msb]
    packet = get_packet(packet_bytes)
    return packet

def get_read_packet(servo_id, register, num_bytes = 2):
    """Return a read register packet. 
    
    :param id: The identifier of the servo to address.
    :param register: The address of the register to read. 
    :param num_bytes: The number of bytes to read. (The default is 2.)
    :returns: A command packet in the form of a list of bytes.
    
    """
    packet_bytes = [id, 0x02 + num_bytes, 0x02, register, num_bytes]
    packet = get_packet(packet_bytes)
    return packet

def get_read_position_packet(servo_id):
    """Return a read position packet. 
    
    :param id: The identifier of the servo to address.

    :returns: A read position packet in the form of a list of bytes.
    
    """
    return get_read_packet(servo_id, registers.PRESENT_POSITION, 2)
    
def get_read_torque_packet(servo_id):
    """Return a read torque (load) packet. 
    
    :param id: The identifier of the servo to address.

    :returns: A read torque packet in the form of a list of bytes.
    
    """
    return get_read_packet(servo_id, registers.PRESENT_LOAD, 2)

def get_read_is_moving_packet(servo_id):
    """Return a read is moving packet. 
    
    :param id: The identifier of the servo to address.

    :returns: A read is moving packet in the form of a list of bytes.
    
    """
    
    return get_read_packet(servo_id, registers.MOVING, 2)

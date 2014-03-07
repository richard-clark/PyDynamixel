"""

Dynamixel Chain
***************

This library builds upon the :mod:`dynamixel` library, and adds functions for 
interating with systems containing kinematic chains of multiple Dynamixels, 
such as robotic arms.

This library operates using vectors in joint space. Each vector is in the form
of a list of tuples. That is,

    V = [(id1, angle1, vel1),
         (id2, angle2, vel2),
         ...
         (idN, angleN, velN)]

where ``id`` is the id of the associated servo, ``angle`` is the angle to which 
the servo should be positioned, and ``vel`` is the velocity at which the servo
should rotate to achieve that position.

A typical command sequence would be to command the chain to a specified vector,
using :func:`move_to_vector`, and then wait for the move to complete, by invoking
:func:`wait_for_move`.

The :func:`read_position` will return a vector of the angles of all the specified
Dynamixels.

note:: After the Dynamixel is powered on, the first angle to which it should be set
is its current value. If the first angle is not set to the current value, it will
move at maxium speed to whatever next value is set. The current position can be read
using func:`read_position`.

"""


import dynamixel
import time

NUM_ERROR_ATTEMPTS = 10
SLEEP_TIME = 0.1
VERBOSE = True

def wait_for_move(ser, joints, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Blocks until all of the servos with IDs specified by `joints` have stopped moving.
    
    :param ser: The ``serial`` port to use. 
    :param joints: A list of servo IDs. 
    :param verbose: If True, status information will be printed. Default: ``VERBOSE``.
    :param num_error_attempts: The number of attempts to make to send the packet when an error is encountered.
    
    :returns: ``None``.
    
    """
       
    # Iterate over the joints 
    for j in joints:
        # Get the packet which will be used to read the moving status of the joint
        packet = dynamixel.get_read_packet(j, 0x2E, 2)
                
        # Loop until the move status indicates that the joint is no longer moving
        while True:
            resp = dynamixel.write_and_get_response_multiple(ser, packet, j, verbose, num_error_attempts)
            
            moving = resp.data[0]
            
            if moving == 0:
                break
                        
            time.sleep(SLEEP_TIME)

def move_to_vector(ser, vector, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Move multiple servos concurrently. 
    
    :param ser: The ``serial`` object to use. 
    :param vector: A list of vectors in the form (id, position, velocity).  
    :param verbose: If True, status information will be printed. Default: ``VERBOSE``.
    :param num_error_attempts: The number of attempts to make to send the packet when an error is encountered.
    
    :returns: ``None``.
    
    """
    
    for i in range(0,len(vector)):
        
        id = vector[i][0]
        angle = vector[i][1]
        velocity = vector[i][2]
        
        if verbose:
            print('Setting angle for {0} to {1}...'.format(id, angle))
        packet = dynamixel.get_write_packet_2b(id, 0x1E, angle)
        dynamixel.write_and_get_response_multiple(ser, packet, id, verbose, num_error_attempts)
        
        if verbose:
            print('Setting velocity for {0} to {1}...'.format(id, velocity))
        packet = dynamixel.get_write_packet_2b(id, 0x20, velocity)
        dynamixel.write_and_get_response_multiple(ser, packet, id, verbose, num_error_attempts)
        
    if verbose:
        print('Sending action packet.')

    packet = dynamixel.get_action_packet()
    ser.write(packet)
     
     
def read_position(ser, joints, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Read the current position of each joint. 
    
    Move each joint to this position so that servo
    does not jerk into position when moved later.
    
    :param ser: The ``serial`` object to use. 
    :param joints: A list of servo IDs.
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param num_error_attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
    
    :returns: An array of values which correspond to the initial positions of the joints. 
    
    """
    
    # Create a list to hold the positions
    init_positions = []
    
    # Iterate over the joints and read the position of each
    for j in joints:
        
        if verbose:
            print('Reading initial position for joint {0}...'.format(j))
        
        packet = dynamixel.get_read_packet(j, 0x24, 2)
        
        resp = dynamixel.write_and_get_response_multiple(ser, packet, j, verbose, num_error_attempts)
        
        pos = resp.data[1] * 256 + resp.data[0]
        init_positions.append(pos)
        
    return init_positions

def make_vector(position, joints, velocity):
    """
    A convenience method to make a vector from a list of positions, a list of joints,
    and the same velocity for all joints. 
    
    :param position: A list of servo positions. 
    :param joints: A list of servo ids. 
    :param velocity: The velocity at which to move all joint.
    
    :returns: A vector.
    
    """
    vector = []
    
    for i in range(len(joints)):
        vector.append((joints[i], position[i], velocity))
        
    return vector

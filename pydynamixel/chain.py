"""

Chain
=====

This module is designed to make it easy to control multiple servos in a synchronized manner 
using less code.

This module further implements functionality which is important in applications such as 
kinematic chains, which contain a number of interconnected servos. In a robotic arm, 
for example, it is important to ensure that all of the joints have completed moving to 
the first target position before the arm starts moving to a second position.

This library need not only be used for kinetic chains; it can be used in any 
application where it is required to control multiple servos in a synchronized manner.


The position of the servos is controlled using vectors. Each vector contains a list of 
tuples, each containing a servo id, the target position, and the velocity at which the 
servo should move. For example:

    servo_id = 1
    target_position = 512
    velocity = 180
    
    vector_tuple = (servo_id, target_position, velocity)
    
Given multiple vectors, the ``move_to_vector()`` method can be used to move every servo 
for every tuple at once. For example:

    from pydynamixel import chain, dynamixel

    # You'll need to change this to the serial port of your USB2Dynamixel
    serial_port = '/dev/tty.usbserial-A921X77J'
    ser = dynamixel.get_serial_for_url(serial_port)

    vector = [
        (1, 822, 180),
        (2, 94, 140),
        (3, 929, 100)
    ]
    
    chain.move_to_vector(ser, vector)
    
Moving multiple servos through multiple frames using this vector syntax is 
quite combersome. If the servo ids and velocities are constant, the ``make_vector()`` 
convenience method can be used:

    from pydynamixel import chain, dynamixel

    # You'll need to change this to the serial port of your USB2Dynamixel
    serial_port = '/dev/tty.usbserial-A921X77J'
    ser = dynamixel.get_serial_for_url(serial_port)

    servo_ids = [1,2,3]
    velocities = [180,140,100]
    
    vector_1 = make_vector(servo_ids, [822, 94, 929], velocities)
    vector_2 = make_vector(servo_ids, [822, 632, 391], velocities)

The following example demonstrates the process of manipulating the robot using a series of frames.

    from pydynamixel import chain, dynamixel

    joints = [1, 2, 3, 4, 5, 6, 7]
    velocity = 180
    
    # Initialize the servos
    chain.init(ser, joints, velocity)
    
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
        vector = chain.make_vector_constant_velocity(v, joints, velocity)
        chain.move_to_vector(ser, vector)
        chain.wait_for_move(ser, joints)
      
Fore each vector, after instructing the joints to move to the specified position, 
the program waits until all servos have finished moving before moving to the next frame.

"""

import dynamixel
import packets
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
        packet = packets.get_read_packet(j, 0x2E, 2)
                
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
        
        servo_id = vector[i][0]
        angle = vector[i][1]
        velocity = vector[i][2]
        
        if verbose:
            print('Setting angle for {0} to {1}...'.format(id, angle))
        packet = packets.get_write_position_packet(servo_id, angle)
        dynamixel.write_and_get_response_multiple(ser, packet, id, verbose, num_error_attempts)
        
        if verbose:
            print('Setting velocity for {0} to {1}...'.format(id, velocity))
        packet = packets.get_write_velocity_packet(servo_id, velocity)
        dynamixel.write_and_get_response_multiple(ser, packet, id, verbose, num_error_attempts)
        
    if verbose:
        print('Sending action packet.')
    
    packet = packets.get_action_packet()
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
        
        pos = packets.get_read_position_packet(j)
        init_positions.append(pos)
        
    return init_positions

def make_vector_constant_velocity(position, joints, velocity):
    """
    A convenience method to make a vector from a list of positions, a list of joints,
    and the same velocity for all joints. 
    
    :param position: A list of servo positions. 
    :param joints: A list of servo ids. 
    :param velocity: The velocity at which to move all joints.
    
    :returns: A vector.
    
    """
    vector = []
    
    for i in range(len(joints)):
        vector.append((joints[i], position[i], velocity))
        
    return vector

def make_vector(position, joints, velocity):
    """
    A convenience method to make a vector from a list of positions, a list of joints,
    and a list of velocities.
    
    :param position: A list of servo positions. 
    :param joints: A list of servo ids. 
    :param velocity: The velocities at which to move each joint.
    
    :returns: A vector.
    
    """
    vector = []
    
    for i in range(len(joints)):
        vector.append((joints[i], position[i], velocity[i]))
        
    return vector

def init_constant_velocity(ser, joints, velocity, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Read the current position of the servos and then set that position. Used to mitigate issues
    with the servos moving abrubtly when first powered on.
    
    :param ser: The ``serial`` object to use. 
    :param joints: A list of servo IDs.
    :param velocity: The velocity at which to move all joints.
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param num_error_attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
    
    :returns: A vector which corresponds to the initial positions of the joints. 
    
    """
    
    init_pos = read_position(ser, joints, verbose, num_error_attempts)
    vector = make_vector_constant_velocity(init_pos, joints, velocity)
    move_to_vector(ser, vector, verbose, num_error_attempts)
    wait_for_move(ser, joints, verbose, num_error_attempts)
    
    return vector
    

def init(ser, joints, velocity, verbose = VERBOSE, num_error_attempts = NUM_ERROR_ATTEMPTS):
    """
    Read the current position of the servos and then set that position. Used to mitigate issues
    with the servos moving abrubtly when first powered on.
    
    :param ser: The ``serial`` object to use. 
    :param joints: A list of servo IDs.
    :param velocity: The velocities at which to move each joint.
    :param verbose: If True, status information will be printed. (Default: ``VERBOSE``).
    :param num_error_attempts: The number of attempts to make to send the packet when an error is encountered. (Default: ``NUM_ERROR_ATTEMPTS``.)
    
    :returns: A vector which corresponds to the initial positions of the joints. 
    
    """
    
    init_pos = read_position(ser, joints, verbose, num_error_attempts)
    vector = make_vector(init_pos, joints, velocity)
    move_to_vector(ser, vector, verbose, num_error_attempts)
    wait_for_move(ser, joints, verbose, num_error_attempts)
    
    return vector
    

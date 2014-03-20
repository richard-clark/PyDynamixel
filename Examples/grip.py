
from pydynamixel import dynamixel, chain

def grip(ser, joint, incr, limit, velocity, verbose, num_error_attempts):
    """
    This function will modify the angular position of the servo with the specified ID until
    its measured torque exceeds a specified value. This is especailly useful for gripping objects, 
    as it allows objects of all sizes to be gripped.
    
    :param joint: The servo ID of the joint to manipulate.
    :param incr: The amount by which to increment (or decrement) the current angular position.
    :param limit: The torque limit, in Dynamixel units. 
    :param velocity: The velocity at which to change the position.
    :param verbose: If True, status information will be printed. Default: ``VERBOSE``.
    :param num_error_attempts: The number of attempts to make to send the packet when an error is encountered.
    
    :returns: The angular position at which the torque was exceeded.
    """
    # The packet used to read the torque.
    # This is the same each time it is set.
    torque_packet = dynamixel.get_read_packet(joint, 0x28, 2)

    # The initial angular position
    val = chain.read_position(ser, [joint], verbose, num_error_attempts)[0]

    # Loop forever
    while True:
        # Get the torque
        resp = dynamixel.write_and_get_response_multiple(ser, torque_packet, joint, verbose, num_error_attempts)
        torque = resp.data[1] * 256 + resp.data[0]
        
        if verbose:
            print('Torque: {0}'.format(torque))

        # Check the torque. If greater than the limit, return.
        if torque >= limit:
            if verbose:
                print('Torque at limit!')
                
            return val
        
        # Otherwise, increment the angular position. 
        val += incr
        if verbose:
            print('Setting val to {0}.'.format(val))
        
        vector = chain.make_vector([val], [joint], velocity)
        chain.move_to_vector(ser, vector, verbose, num_error_attempts)
        chain.wait_for_move(ser, [joint], verbose, num_error_attempts)
    
if __name__ == '__main__':
    url = '/dev/tty.usbserial-A9SFBTPX'
    ser = dynamixel.get_serial_for_url(url)
    
    verbose = False
    num_error_attempts = 10
    joint = 7
    limit = 1200
    velocity = 100
    incr = 1
    
    grip(ser, joint, incr, limit, velocity, verbose, num_error_attempts)
    
#!/usr/bin/env python3

import sys
from optparse import OptionParser

from dynamixel_driver import dynamixel_io

def main(args=sys.argv):
    usage_msg = 'Usage: %prog [options] ID [On|Off]'
    desc_msg = 'Turns the torque of specified Dynamixel servo motor on or off.'
    epi_msg = 'Example: %s --port=/dev/ttyUSB1 --baud=57600 1 Off' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
                      
    (options, args) = parser.parse_args(args)
    
    if len(args) < 3:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    motor_id = int(args[1])
    torque_on = args[2]

    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError as soe:
        print('ERROR: {}'.format(soe))
    else:
        print('Turning torque %s for motor %d' % (torque_on, motor_id))
        if dxl_io.ping(motor_id):
            if torque_on.lower() == 'off':
                torque_on = False
            elif torque_on.lower() == 'on':
                torque_on = True
            else:
                parser.print_help()
                exit(1)
            dxl_io.set_torque_enabled(motor_id, torque_on)
            print('done')
        else:
            print('ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id)

if __name__ == '__main__':
    main()


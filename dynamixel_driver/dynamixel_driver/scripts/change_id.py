#!/usr/bin/env python3

import sys
from optparse import OptionParser

from dynamixel_driver import dynamixel_io

def main(args=sys.argv):
    parser = OptionParser(usage='Usage: %prog [options] OLD_ID NEW_ID', description='Changes the unique ID of a Dynamixel servo motor.')
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
    old_id = int(args[1])
    new_id = int(args[2])
    
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError as soe:
        print('ERROR:{}'.format(soe))
    else:
        print('Changing motor id from %d to %d...' % (old_id, new_id), end='')
        if dxl_io.ping(old_id):
            dxl_io.set_id(old_id, new_id)
            print(' done')
            print('Verifying new id...', end='') 
            if dxl_io.ping(new_id):
                print(' done')
            else:
                print('ERROR: The motor did not respond to a ping to its new id.')
        else:
            print('ERROR: The specified motor did not respond to id %d. Make sure to specify the correct baudrate.' % old_id)

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import sys
from optparse import OptionParser

from dynamixel_driver import dynamixel_io
from dynamixel_driver.dynamixel_const import *

def print_data(values):
    ''' Takes a dictionary with all the motor values and does a formatted print.
    '''
    if values['freespin']:
        print('''\
    Motor %(id)d is connected:
        Freespin: True
        Model ------------------- %(model)s
        Current Speed ----------- %(speed)d
        Current Temperature ----- %(temperature)d%(degree_symbol)sC
        Current Voltage --------- %(voltage).1fv
        Current Load ------------ %(load)d
        Moving ------------------ %(moving)s
''' %values)
    else:
        print('''\
    Motor %(id)d is connected:
        Freespin: False
        Model ------------------- %(model)s
        Min Angle --------------- %(min)d
        Max Angle --------------- %(max)d
        Current Position -------- %(position)d
        Current Speed ----------- %(speed)d
        Current Temperature ----- %(temperature)d%(degree_symbol)sC
        Current Voltage --------- %(voltage).1fv
        Current Load ------------ %(load)d
        Moving ------------------ %(moving)s
''' %values)

def main(args=sys.argv):
    usage_msg = 'Usage: %prog [options] IDs'
    desc_msg = 'Prints the current status of specified Dynamixel servo motors.'
    epi_msg = 'Example: %s --port=/dev/ttyUSB1 --baud=57600 1 2 3 4 5' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
                      
    (options, args) = parser.parse_args(args)
    
    if len(args) < 2:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    motor_ids = args[1:]
    
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError as soe:
        print('ERROR: {}'.format(soe))
    else:
        responses = 0
        print('Pinging motors:')
        for motor_id in motor_ids:
            motor_id = int(motor_id)
            print('%d ...' % motor_id, end='')
            p = dxl_io.ping(motor_id)
            if p:
                responses += 1
                values = dxl_io.get_feedback(motor_id)
                angles = dxl_io.get_angle_limits(motor_id)
                model = dxl_io.get_model_number(motor_id)
                firmware = dxl_io.get_firmware_version(motor_id)
                values['model'] = '%s (firmware version: %d)' % (DXL_MODEL_TO_PARAMS[model]['name'], firmware)
                values['degree_symbol'] = u"\u00B0"
                values['min'] = angles['min']
                values['max'] = angles['max']
                values['voltage'] = values['voltage']
                values['moving'] = str(values['moving'])
                print(' done')
                if angles['max'] == 0 and angles['min'] == 0:
                    values['freespin'] = True
                else:
                    values['freespin'] = False
                print_data(values)
            else:
                print(' error')
        if responses == 0:
            print('ERROR: None of the specified motors responded. Make sure to specify the correct baudrate.')

if __name__ == '__main__':
    main()


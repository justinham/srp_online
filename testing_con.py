import asyncio
import can
import cantools
import sys


import cantools
import can
import sys
import time
import math





"""
General Notes:
    1.  Currently using the time at message creation for timestamp, which may result in some in-accuracy
        This may need to be changed for using Aquisition timestamp, but I have no idea as of now how it works as it is reported
        in ms, and ranges from 0-2047 and is only reported in a single frame per burst
    2. Sensor Ids: 0 - Infra, 1 - LRR, 2 - FCM, 3- SRRLF, 4 - SRRLR

"""


def main(args=None):

    can.rc['interface'] = 'socketcan'
    can.rc['channel'] = "can0"
    can.rc['bitrate'] = 500000
    
    db = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/Mapless_PoC 2.dbc')

    while(True):
            with can.Bus() as bus:
                for msg in bus:
                    decoded = db.decode_message(msg.arbitration_id, msg.data)
                    print(decoded)
                    print("*******************************************")
    

if __name__ == '__main__':
    main(sys.argv)

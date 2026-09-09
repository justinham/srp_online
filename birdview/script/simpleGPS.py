import serial

import asyncio
import sys
import datetime

import sys
import time
import math


port = "/dev/ttyUSB0"
baud_rate = 115200

class SerialGPSReader():
    def __init__(self):
        super().__init__()


        try:
            ser = serial.Serial(port, baud_rate)
            print(f"Connected to: {ser.portstr}")
        except: 
            print(f"Error: Could not open serial port")
            exit()

        # GPS Variables
        self.gps_lat = 0
        self.gps_lon = 0
        self.first_start_tag = True

        
        self.read_serial(ser)
        
    def ddmm_to_dd(self, coordinate):
   
        if coordinate.find('.') == 4:
            coordinate = '0'+coordinate
        # print(f"coordinate is: {coordinate}")
        degrees = int(coordinate[:3])
        minutes = float(coordinate[3:])
        decimal_degrees = degrees + (minutes / 60.0)
        return decimal_degrees

    def read_serial(self, ser):
        # Read data from the serial port
        while True:
            try:
                # Read a line of data from the serial port
                data = ser.readline().decode('utf-8').strip()
                #self.raw_log.write(data + "\n")
                if "$GPRMC" in data:
                    splits = data.split(',')[1:]
                    # print(splits)
                    if splits[1] == "V":
                        #print("Invalid GPS")
                        valid = False
                    else:
                        valid = True
                    if 1:
                        self.gps_lat = self.ddmm_to_dd(splits[2]) if splits[3] == 'N' else -self.ddmm_to_dd(splits[2])
                        self.gps_lon = self.ddmm_to_dd(splits[4]) if splits[5] == 'E' else -self.ddmm_to_dd(splits[4])
                        
                        print(self.gps_lat, self.gps_lon)
                        gps_log = open("./hummer_path/gps.txt", "w")
                        gps_log.write("[%f,%f]"%(self.gps_lat, self.gps_lon))
                        gps_log.close()
                        
                        ## no need to get the ini gps location here
                        ## change to: when path uploaded, copy the current location to the init loc
                        
                        # if self.first_start_tag:
                        #     gps_log = open("./hummer_path/gps_ref_p.txt", "w")
                        #     gps_log.write("[%f,%f]"%(self.gps_lat, self.gps_lon))
                        #     gps_log.close()
                        #     self.first_start_tag = False    
                    
            except serial.SerialException as e:
                print(f"Error reading from serial port: {e}")
                break
            except KeyboardInterrupt:
                print("Exiting program")
                break
            except UnicodeDecodeError:
                print("Error Decoding Serial Data")

        # Close the serial port
        ser.close()
        print("Serial port closed")


SerialGPSReader()

# def main(args=None):
#     SerialGPSReader(sys.argv)

# if __name__ == '__main__':

#     main(sys.argv)

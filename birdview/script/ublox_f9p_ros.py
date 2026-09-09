import serial

import asyncio
import sys
import datetime

import rclpy
from rclpy.node import Node

from fusion.msg import GPS
import sys, json
import time
import math

from .can_to_message import *

import rclpy
import rclpy.duration


port = "/dev/ttyACM1"
# port = "/dev/ttyACM1" #f9p
baud_rate = 115200

class SerialGPSReader(Node):
    def __init__(self, args):
        super().__init__('SerailGPSReader')

        self.publisher_gps = self.create_publisher(GPS, 'gps', 100)
        
        try:
            ser = serial.Serial(port, baud_rate)
            print(f"Connected to: {ser.portstr}")
        except serial.SerialException as e:
            print(f"Error: Could not open serial port: {e}")
            exit()

        self.i = 0
        
        file_path = "log"
        self.gps_file = open(file_path + "_ublox_gps.csv", 'w+')
        self.gps_file.write("timestamp (s) \{PC Timestamp\},lat (deg),lon (deg),heading (rad),calculated speed (km/h),year - gps,day of year - gps,time of day in ms - gps UTC, gps valid, heading valid\n")
        self.raw_log = open("RAW-GPS.csv", 'w+')


        # GPS Variables
        self.first_gps_time = 0
        self.gps_msg_recv = 0
        self.avg_gps_time = 0
        self.gps_lat = 0
        self.gps_lon = 0
        self.gps_heading = 0
        self.gps_speed = 0
        self.prev_gps_speed = 0
        self.gps_year = 0
        self.gps_day = 0
        self.gps_ms = 0

        # Filters
        self.alpha_speed   = 0.3        # EMA for speed
        self.ego_gps_speed_filtered = 0.0
        self.alpha_heading = 0.3        # EMA for heading vector
        self.ego_heading_filtered_x = 0.0
        self.ego_heading_filtered_y = 0.0

        self.read_serial(ser)
        
    def ddmm_to_dd(self, coordinate):
        """Converts GPS coordinates from DDMM.MM format to decimal degrees.

        Args:
            coordinate: A string representing the coordinate in DDMM.MM format.

        Returns:
            A float representing the coordinate in decimal degrees.
        """
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
                self.raw_log.write(data + "\n")
                # print(data)
                if "$GPRMC" in data:
                    splits = data.split(',')[1:]
                    #print(splits)
                    if splits[1] == "V":
                        #print("Invalid GPS")
                        valid = False
                    else:
                        valid = True
                    if 1:
                        try:
                            self.gps_lat = self.ddmm_to_dd(splits[2]) if splits[3] == 'N' else -self.ddmm_to_dd(splits[2])
                            self.gps_lon = self.ddmm_to_dd(splits[4]) if splits[5] == 'E' else -self.ddmm_to_dd(splits[4])
                            self.gps_speed = float(splits[6])
                            if len(splits[7]) != 0:
                                self.gps_heading = float(splits[7]) / 180 * math.pi
                                heading_valid = True
                            else:
                                #print("Invalid Heading")
                                self.gps_heading = 0.0
                                heading_valid = False

            

                            ## add by Justin to support the in-vehicle visulization app
                            print("j", self.gps_lat, self.gps_lon)
                            gps_log = open("/home/connau/srp/birdview/hummer_path/gps.txt", "w")
                            gps_log.write("[%f,%f]"%(self.gps_lat, self.gps_lon))
                            gps_log.close()
                            #########################
                            


                            month_day = int(splits[8][:2])
                            splits[8] = splits[8][2:]
                            month = int(splits[8][:2])
                            splits[8] = splits[8][2:]
                            year = 2000 + int(splits[8][:2])
                            date_object = datetime.datetime(year, month, month_day)
                            day_of_year = date_object.timetuple().tm_yday
                            self.gps_day = day_of_year
                            self.gps_year = year

                            hour = splits[0][:2]
                            minutes = int(splits[0][2:4]) + int(hour) * 60
                            seconds = float(splits[0][4:]) + minutes * 60.0

                            self.gps_ms = seconds * 1000

                            self.gps_file.write(f"{self.get_clock().now().nanoseconds/1e9}, {self.gps_lat}, {self.gps_lon}, {self.gps_heading}, {self.gps_speed}, {self.gps_year}, {self.gps_day}, {self.gps_ms}, {valid}, {heading_valid}\n")
                            g = GPS()
                            g.timestamp = self.get_clock().now().to_msg()
                            g.latitude = self.gps_lat
                            g.longitude = self.gps_lon
                            g.heading = self.gps_heading
                            g.speed = float(self.gps_speed)*0.5144
                            g.valid = valid
                            g.heading_valid = heading_valid
                            self.ego_gps_speed_filtered = (
                                self.alpha_speed*g.speed + (1.0-self.alpha_speed)*self.ego_gps_speed_filtered
                            )
                            g.speed_f = float(self.ego_gps_speed_filtered)

                            hx, hy = np.cos(self.gps_heading), np.sin(self.gps_heading)
                            self.ego_heading_filtered_x = self.alpha_heading*hx + (1.0-self.alpha_heading)*self.ego_heading_filtered_x
                            self.ego_heading_filtered_y = self.alpha_heading*hy + (1.0-self.alpha_heading)*self.ego_heading_filtered_y
                            heading_f = np.arctan2(self.ego_heading_filtered_y, self.ego_heading_filtered_x)
                            if heading_f < 0:
                                heading_f += 2*np.pi
                            g.heading_f = heading_f

                            if g.speed_f < 1.5:
                                if g.speed > 1.0 and (g.speed_f < self.prev_gps_speed):
                                    h_log = open("/home/connau/connau_aux_files/ego_ublox_heading_hysteresis.txt", "w")
                                    h_log.write("%f"%(self.gps_heading))
                                    h_log.close()
                                else:
                                    h_log = open("/home/connau/connau_aux_files/ego_ublox_heading_hysteresis.txt", "r")
                                    line = h_log.readline()
                                    g.heading = json.loads(line)
                                    self.get_logger().info(f"val from residual: {g.heading}")
                                    h_log.close()

                            self.prev_gps_speed = g.speed_f

                            print(self.gps_lat, self.gps_lon, self.gps_heading, heading_f)

                            self.publisher_gps.publish(g)
                        except:
                            print("ublox decoding fail")

                    
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

def main(args=None):
    rclpy.init(args=args)
    
    publisher = SerialGPSReader(sys.argv)

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
    

'''
import serial
import time
import pynmea2

def process_gps(sentence):
    lat = 0.0
    lon = 0.0
    valid = 0

    # GNRMC provides the recommended minimum navigation data, including time, date, position, speed, and course
    if sentence.startswith('$GNRMC'):
        try:
            msg = pynmea2.parse(sentence)
            # print(f"RMC Sentence - Status: {msg.status}")
            if msg.status == 'A':
                # This block will run only if the fix is valid
                # print(f"  Valid Fix! Latitude: {msg.latitude}°")
                # print(f"  Valid Fix! Longitude: {msg.longitude}°")
                # The library handles conversion from ddmm.mmmm to decimal degrees
                lat = round(float(msg.latitude),7)
                lon = round(float(msg.longitude),7)
                valid = 1
            else:
                print("GPS not fix yet")
        except pynmea2.ParseError as e:
            print(f"RMC Parse error: {e}")

    # GNGGA contains essential fix data like time, position, and fix quality, along with altitude and geoid separation
    elif sentence.startswith('$GNGGA'):
        try:
            msg = pynmea2.parse(sentence)
            # print(f"GGA Sentence - Fix Quality: {msg.gps_qual}")
            if msg.gps_qual > 0:
                # This block will run if fix quality is valid (1 or higher)
                # print(f"  Valid Fix! Latitude: {msg.latitude}°")
                # print(f"  Valid Fix! Longitude: {msg.longitude}°")
                lat = round(float(msg.latitude),7)
                lon = round(float(msg.longitude),7)
                valid = 1
        except pynmea2.ParseError as e:
            print(f"GGA Parse error: {e}")

    else:
        valid = 0

    return (lat,lon,valid)



def gps2file(lat,lon):
    path = "/home/connau/srp/birdview/hummer_path/gps.txt"
    # path = "a.txt"
    try:
        with open(path, "w") as gps_log:
            gps_log.write("[%f,%f]"%(lat, lon))
    except:
        pass









################## main ################
try:
    ser = serial.Serial(
        port='/dev/ttyACM1', 
        baudrate=115200,
        bytesize=8,
        parity='N',
        stopbits=1,
        timeout=1  # Read timeout in seconds
    )

    print(f"Connected to {ser.portstr}")

    while True:
        if ser.in_waiting > 0:
            # Read a line from the serial port
            line = ser.readline()
            # Decode bytes to string, assuming UTF-8 encoding
            try:
                decoded_line = line.decode('utf-8', errors='replace').strip()
                print(f"Received: {decoded_line}")
                lat,lon,valid = process_gps(decoded_line)
          
                if valid==1:
                    print("valid ublox gps:", lat,lon)
                    gps2file(lat,lon)
                    
            except UnicodeDecodeError:
                print(f"Received raw bytes (could not decode): {line}")
        else:
            # print("Waiting for data...")
            time.sleep(0.1) # Small delay to prevent high CPU usage

except serial.SerialException as e:
    print(f"Error opening or reading from serial port: {e}")
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    if 'ser' in locals() and ser.isOpen():
        ser.close()
        print("Serial port closed.")
'''
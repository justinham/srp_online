import sys
import time
import math
import datetime
import serial
import numpy as np

# Port Configurations
port = "/dev/ttyACM0"  # f9p (connect to usb hub)
baud_rate = 115200

class SerialGPSReader:
    def __init__(self, args):
        # 1. Initialize Serial Communication directly
        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1.0)
            print(f"Connected to serial port: {self.ser.portstr}")
        except serial.SerialException as e:
            print(f"Error: Could not open serial port: {e}")
            sys.exit(1)

        self.i = 0
        
        # 2. Setup standard logging files
        file_path = "log"
        self.gps_file = open(file_path + "_ublox_gps.csv", 'w+')
        self.gps_file.write("timestamp (s) {PC Timestamp},lat (deg),lon (deg),heading (rad),calculated speed (km/h),year - gps,day of year - gps,time of day in ms - gps UTC, gps valid, heading valid\n")
        self.raw_log = open("RAW-GPS.csv", 'w+')

        # GPS Base Variables
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

        # Signal Filters (Exponential Moving Averages)
        self.alpha_speed = 0.3        
        self.ego_gps_speed_filtered = 0.0
        self.alpha_heading = 0.3        
        self.ego_heading_filtered_x = 0.0
        self.ego_heading_filtered_y = 0.0
        self.heading_latcher = False

        # 3. Enter processing loop
        self.read_serial()
        
    def ddmm_to_dd(self, coordinate):
        if not coordinate:
            return 0.0
        if coordinate.find('.') == 4:
            coordinate = '0' + coordinate
        degrees = int(coordinate[:3])
        minutes = float(coordinate[3:])
        decimal_degrees = degrees + (minutes / 60.0)
        return decimal_degrees

    def read_serial(self):
        print("Starting NMEA loop parser. Press Ctrl+C to terminate cleanly...")
        while True:
            try:
                # Read line from device interface safely
                line_bytes = self.ser.readline()
                if not line_bytes:
                    continue
                
                data = line_bytes.decode('utf-8', errors='ignore').strip()
                self.raw_log.write(data + "\n")
                
                # Check for standard Recommended Minimum Navigation sentence ($GNRMC)
                if "$GNRMC" in data:
                    splits = data.split(',')[1:]
                    
                    # Boundary condition guard to ensure a full payload array frame exists
                    if len(splits) < 9:
                        continue
                        
                    valid = False if splits[1] == "V" else True

                    try:
                        # Extract basic position coordinates
                        self.gps_lat = self.ddmm_to_dd(splits[2]) if splits[3] == 'N' else -self.ddmm_to_dd(splits[2])
                        self.gps_lon = self.ddmm_to_dd(splits[4]) if splits[5] == 'E' else -self.ddmm_to_dd(splits[4])
                        self.gps_speed = float(splits[6]) if splits[6] else 0.0
                        
                        if len(splits[7]) != 0:
                            self.gps_heading = float(splits[7]) / 180 * math.pi
                            heading_valid = True
                        else:
                            self.gps_heading = 0.0
                            heading_valid = False

                        # Local write-out to human-readable path tracker script dependency 
                        try:
                            with open("/home/connau/srp/birdview/hummer_path/gps.txt", "w") as gps_log:
                                gps_log.write("[%f,%f]" % (self.gps_lat, self.gps_lon))
                        except IOError:
                            pass

                        # Extract Data Time parameters
                        month_day = int(splits[8][:2])
                        rem_date = splits[8][2:]
                        month = int(rem_date[:2])
                        year = 2000 + int(rem_date[2:4])
                        
                        date_object = datetime.datetime(year, month, month_day)
                        day_of_year = date_object.timetuple().tm_yday
                        self.gps_day = day_of_year
                        self.gps_year = year

                        # Parse time offsets
                        hour = splits[0][:2]
                        minutes = int(splits[0][2:4]) + int(hour) * 60
                        seconds = float(splits[0][4:]) + minutes * 60.0
                        self.gps_ms = seconds * 1000

                        # Get current local UNIX system processing snapshot time
                        now_seconds = time.time()

                        # Write row updates directly to local tracking log
                        self.gps_file.write(
                            f"{now_seconds}, {self.gps_lat}, {self.gps_lon}, {self.gps_heading}, "
                            f"{self.gps_speed}, {self.gps_year}, {self.gps_day}, {self.gps_ms}, "
                            f"{valid}, {heading_valid}\n"
                        )
                        self.gps_file.flush()

                        # Operational unit speed calculation conversions (Knots to m/s)
                        speed_meters_per_sec = float(self.gps_speed) * 0.5144
                        
                        # Apply Exponential Moving Average filtering logic
                        self.ego_gps_speed_filtered = (
                            self.alpha_speed * speed_meters_per_sec + 
                            (1.0 - self.alpha_speed) * self.ego_gps_speed_filtered
                        )

                        hx, hy = np.cos(self.gps_heading), np.sin(self.gps_heading)
                        self.ego_heading_filtered_x = self.alpha_heading * hx + (1.0 - self.alpha_heading) * self.ego_heading_filtered_x
                        self.ego_heading_filtered_y = self.alpha_heading * hy + (1.0 - self.alpha_heading) * self.ego_heading_filtered_y
                        
                        heading_f = np.arctan2(self.ego_heading_filtered_y, self.ego_heading_filtered_x)
                        if heading_f < 0:
                            heading_f += 2 * np.pi

                        # Low-speed directional orientation state locks
                        if self.ego_gps_speed_filtered < 0.6:
                            self.heading_latcher = True
                        elif self.ego_gps_speed_filtered > 0.9:
                            self.heading_latcher = False

                        self.prev_gps_speed = self.ego_gps_speed_filtered

                        # Print clean diagnostics directly to execution terminal window
                        print(f"Lat: {self.gps_lat:.6f} | Lon: {self.gps_lon:.6f} | Hdg Raw: {self.gps_heading:.4f} | Hdg Filt: {heading_f:.4f}")

                    except Exception as parse_err:
                        print(f"ublox decoding fail: {parse_err}")
                        
            except serial.SerialException as e:
                print(f"Error reading from serial port: {e}")
                break
            except UnicodeDecodeError:
                print("Error Decoding Serial Data bytes")
            except KeyboardInterrupt:
                print("\nShutdown interrupt received.")
                break

        self.cleanup()

    def cleanup(self):
        print("Closing file records and releasing serial port interfaces...")
        try:
            self.gps_file.close()
            self.raw_log.close()
            if self.ser and self.ser.is_open:
                self.ser.close()
            print("Cleanup finished successfully.")
        except Exception as e:
            print(f"Error during file/port cleanup: {e}")

if __name__ == '__main__':
    SerialGPSReader(sys.argv)

import sys
import time
import math
import numpy as np
import can
import cantools
import cantools.database

def convert_xy_to_lat_lon(ref_lat_rad, ref_lon_rad, ref_heading_rad, x_m, y_m):
    f = 0.003353
    a = 6378137
    f1 = np.sqrt(f * (2 - f))
    sin_lat = np.sin(ref_lat_rad)
    denom = np.sqrt(1 - (f1 ** 2) * (sin_lat ** 2))
    f2 = a * (1 - f1 ** 2) / (denom ** 3)
    f3 = a / denom
    N = x_m * np.cos(ref_heading_rad) - y_m * np.sin(ref_heading_rad)
    E = x_m * np.sin(ref_heading_rad) + y_m * np.cos(ref_heading_rad)
    lat_rad = ref_lat_rad + N / f2
    lon_rad = ref_lon_rad + E / (f3 * np.cos(ref_lat_rad))
    return lat_rad, lon_rad

class CanGPSReader:
    def __init__(self, args):
        # 1. Initialize Python-CAN Environment directly
        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = "can2"
        can.rc['bitrate'] = 5000000
        self.can_Bus = can.Bus()

        self.i = 0
        self.f_in = None

        # 2. Configure Vehicle Profiles based on runtime script arguments
        if (len(args) > 2 and (args[2] is None or args[2] == "Lyriq")) or len(args) < 3:
            self.db_can5 = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/MY24CAN5.dbc')
            self.speed_heading = 727  # '2D7'
            self.time_day = 726       # '2D6'
            self.lat_lon = 725        # '2D5'
        elif (len(args) > 2 and (args[2] is None or args[2] == "MY22")):
            self.db_can5 = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/MY22CAN5.dbc')
            self.speed_heading = 620  # '26C'
            self.time_day = 619       # '26B'
            self.lat_lon = 618        # '26A'

        # 3. Setup tracking log files
        file_path = "log"
        self.gps_file = open(file_path + "_can_gps.txt", 'w+')
        self.gps_file.write("timestamp (s),lat (ms arc),lon (msg arc),heading (deg),calculated speed (km/h),year,day of year,time of day in ms\n")

        # Local Variables
        self.first_gps_time = 0
        self.gps_msg_recv = 0
        self.avg_gps_time = 0
        self.gps_lat = 0
        self.gps_lon = 0
        self.gps_heading = 0
        self.gps_speed = 0
        self.gps_year = 0
        self.gps_day = 0
        self.gps_ms = 0

        self.rwa = 0.0
        self.prev_heading_raw = None
        self.prev_heading_time = None
        self.heading_rate = 0.0
        self.heading_predict_dt = 0.1
        self.turn_signal = 'S'

        # 4. Start the listener loop
        self.read_can()

    def publish_gps(self, msg):
        frame_id = msg.arbitration_id
        
        # Skip diagnostic/multiframe/unmapped IDs that cause decode errors
        if hex(frame_id)[2:].capitalize() in ['788', '7ED', '7DF', '7EA', '40E', '40C', '7E6', '41B', '7E9', '7EE', '14DA97F4x', '14DAF497x', '14DA80F3x']:
            return

        try:
            decoded = self.db_can5.decode_message(frame_id, msg.data)
        except KeyError:
            return  # Ignore messages not explicitly in the active DBC configuration

        # Turn Signals
        if frame_id == 1490:
            turn_signal = 'S'
            is_left_turn_signal_on = decoded.get('TrnSigSwLtActv')
            is_right_turn_signal_on = decoded.get('TrnSigSwRtActv')
            if is_left_turn_signal_on == 'TRUE':
                turn_signal = 'L'
            elif is_right_turn_signal_on == 'TRUE':
                turn_signal = 'R'
            if turn_signal != self.turn_signal:
                print(f"[Turn Signal Change]: {turn_signal}")
                self.turn_signal = turn_signal

        # Front Wheel Road Wheel Angle
        elif frame_id == 936:
            self.rwa = decoded.get('RdWhlAng', 0.0)
          
        # Rear Steering Angle
        elif frame_id == 1470:
            rear_rwa = decoded.get('RrStrgRdWhlAngAuth', 0.0)

        # Timeout checking to reset tracking window if data gaps occur
        now_sec = time.time()
        if now_sec - self.first_gps_time > 0.5:
            self.gps_msg_recv = 0
            self.avg_gps_time = 0
            self.first_gps_time = 0

        # Group 1: Speed and Heading
        if frame_id == self.speed_heading:
            if self.first_gps_time == 0:
                self.first_gps_time = now_sec
            
            self.avg_gps_time += now_sec
            self.gps_heading = decoded.get('GPSV_PPSHdg', 0.0)
            self.gps_speed = decoded.get('GPSV_PPSCalcdSpd', 0.0)
            print(now_sec, self.gps_speed)
            self.gps_msg_recv += 1

            raw_heading = self.gps_heading / 180 * math.pi

            if self.prev_heading_time is None:
                self.prev_heading_raw = raw_heading
                self.prev_heading_time = now_sec
            else:
                dt = now_sec - self.prev_heading_time
                if self.prev_heading_raw != raw_heading or dt >= 1.0:
                    if dt > 1e-3:
                        dtheta = math.atan2(
                            math.sin(raw_heading - self.prev_heading_raw),
                            math.cos(raw_heading - self.prev_heading_raw)
                        )
                        self.heading_rate = dtheta / dt
                    else:
                        self.heading_rate = 0.0
                    self.prev_heading_raw = raw_heading
                    self.prev_heading_time = now_sec

        # Group 2: Time and Day
        elif frame_id == self.time_day:
            if self.first_gps_time == 0:
                self.first_gps_time = now_sec
            self.avg_gps_time += now_sec
            self.gps_year = decoded.get('GPST_PPSCldrYr', 0)
            self.gps_day = decoded.get('GPST_PPSCldrDy', 0)
            self.gps_ms = decoded.get('GPST_PPSTmOfDy', 0)
            self.gps_msg_recv += 1
      
        # Group 3: Latitude and Longitude
        elif frame_id == self.lat_lon:
            if self.first_gps_time == 0:
                self.first_gps_time = now_sec
            self.avg_gps_time += now_sec
            self.gps_lat = decoded.get('GPSC_PPSLat', 0.0)
            self.gps_lon = decoded.get('GPSC_PPSLong', 0.0)
            self.gps_msg_recv += 1

        # Triggered execution block when all 3 elements of the GPS sequence are updated
        if self.gps_msg_recv == 3:
            now_sec = time.time()
            pred_heading = self.gps_heading / 180 * math.pi + self.heading_rate * (now_sec - self.prev_heading_time)
            pred_heading = (math.atan2(math.sin(pred_heading), math.cos(pred_heading))) % (2 * math.pi)

            # Format fields to decimal values
            lat_deg = self.gps_lat / 3600000.0
            lon_deg = self.gps_lon / 3600000.0
            heading_rad = self.gps_heading / 180.0 * math.pi
            gps_heading_deg = heading_rad * 180 / math.pi

            # Write rows directly to local tracker log
            self.gps_file.write(
                f"{round((self.avg_gps_time / 3.0), 6)}, {lat_deg}, {lon_deg}, "
                f"{heading_rad}, {self.gps_speed}, {self.gps_year}, {self.gps_day}, {self.gps_ms}\n"
            )
            self.gps_file.flush()

            # Local write-out to custom visualization pipeline tracker
            try:
                with open("/home/connau/srp/birdview/hummer_path/can_heading.txt", "w") as srp_can_h_log:
                    srp_can_h_log.write("[%f,%f,%f]" % (lat_deg, lon_deg, gps_heading_deg))
            except IOError as io_err:
                print(f"Failed to update localized srp log: {io_err}")

            # Print diagnostics directly to terminal window
            print(f"Processed Frame -> Lat: {lat_deg:.6f} | Lon: {lon_deg:.6f} | Pred Hdg: {pred_heading:.4f}")

            # Reset state windows
            self.gps_msg_recv = 0
            self.avg_gps_time = 0
            self.first_gps_time = 0

    def read_can(self):
        print(f"Listening to channel: {can.rc['channel']} at {can.rc['bitrate']} bps...")
        try:
            while True:
                msg = self.can_Bus.recv(timeout=0.1)
                if msg is not None:
                    self.publish_gps(msg)
        except KeyboardInterrupt:
            print("\nExiting and wrapping up tracking logs safely...")
        finally:
            self.gps_file.close()
            self.can_Bus.shutdown()

if __name__ == '__main__':
    CanGPSReader(sys.argv)

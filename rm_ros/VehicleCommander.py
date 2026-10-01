import sys
import time
import can
import cantools
import cantools.database

class VehicleCommander:

    def __init__(self):
        # 1. Initialize Python-CAN Interface Directly
        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = "can0"
        can.rc['bitrate'] = 500_000
        self.can_Bus = can.Bus()

        self.start_time = time.time()

        # 2. Load DBC definitions
        self.dbc = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/Mapless_PoC 2.dbc')
        self.output_MSG = self.dbc.get_message_by_frame_id(225)
        
        # Initial target command dictionary
        self.data_dict = {"LongDir_Rq": 1, "StrgWhlAng_Rq": 0.0, "Velocity_Rq": 0.0, "Brake_Rq": 0.0, "BrakeHoldReq": False}
        
        # State tracking (useful if you implement the cleanup method)
        self.curr_steering = 0.0
        self.curr_speed = 0.0
        self.curr_long_dir_rq = 0.0
        self.curr_brake_rq = 0.0
        self.curr_brake_hold_rq = 0.0

    def cleanup(self):
        print("*****************************************, on cleanup")
        decel_rate = -0.3
        iter_count = 0
        while self.curr_speed > 0.0:
            self.curr_speed = max(0.0, self.curr_speed + decel_rate * 0.02)
            self.data_dict = {
                "LongDir_Rq": self.curr_long_dir_rq, 
                "StrgWhlAng_Rq": self.curr_steering, 
                "Velocity_Rq": self.curr_speed, 
                "Brake_Rq": self.curr_brake_rq, 
                "BrakeHoldReq": self.curr_brake_hold_rq
            }
            self.send_request()
            time.sleep(0.02)
            iter_count += 1
            decel_rate = min(-0.1, decel_rate + iter_count / 500)

    def update_command_manually(self, long_dir, steering, speed, brake, hold):
        """Replaces the ROS subscription callback so you can pass raw values directly"""
        self.data_dict = {
            "LongDir_Rq": long_dir, 
            "StrgWhlAng_Rq": steering, 
            "Velocity_Rq": speed, 
            "Brake_Rq": brake, 
            "BrakeHoldReq": hold
        }
        self.curr_steering = steering
        self.curr_speed = speed
        self.curr_long_dir_rq = long_dir
        self.curr_brake_rq = brake
        self.curr_brake_hold_rq = hold

    def send_request(self):
        if time.time() - self.start_time < 1:
            init_dict = {"LongDir_Rq": 0, "StrgWhlAng_Rq": 0.0, "Velocity_Rq": 0.0, "Brake_Rq": 0, "BrakeHoldReq": False}
            print("RUNNING INIT COMMAND")
            data = self.output_MSG.encode(init_dict)
        else:
            data = self.output_MSG.encode(self.data_dict)

        msg = can.Message(arbitration_id=self.output_MSG.frame_id, data=data, is_extended_id=False)
        try:
            self.can_Bus.send(msg)
            print(f"Message sent on {self.can_Bus.channel_info}")
        except can.CanError as e:
            print(f"Message NOT sent: {e}")


def main():
    commander = VehicleCommander()
    
    print("Starting pure Python CAN cyclic loop. Press Ctrl+C to stop.")
    
    # 3. Replace the 50 Hz ROS Timer with a standard Python while-loop
    try:
        while True:
            start_loop = time.time()
            
            # (Optional) Place logic here if you want to dynamically alter data_dict over time
            # Example: commander.update_command_manually(1, 15.5, 5.0, 0.0, False)
            
            commander.send_request()
            
            # Maintain 50Hz frequency (0.02 seconds) accurately by accounting for execution time
            elapsed = time.time() - start_loop
            sleep_time = max(0.001, 0.02 - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nStopping loop safely...")
    finally:
        # Trigger your slowdown cleanup before completely exiting
        commander.cleanup()
        # Explicitly shutdown socketcan interface
        commander.can_Bus.shutdown()

if __name__ == '__main__':
    main()

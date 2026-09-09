import can
import cantools
import sys

import cantools.database
import rclpy
from rclpy.node import Node
import rclpy
import rclpy.duration
import time

# import matplotlib.pyplot as plt

from fusion.msg import VehicleCommand

paths = [[]]
road_shape = []
road_starts = [] # Each road start is a rectangle and defined by 4 lines, (x1, x2, y1, y2)

class VehicleCommander(Node):

    def __init__(self, argv):
        super().__init__('VehicleCommander')
        
        can.rc['interface'] = 'socketcan'
        can.rc['channel'] = "can0"
        can.rc['bitrate'] = 500_000

        self.start_time = time.time()

        # self.temp_cmd_vals = [1, 90, 0, 0, 0]

        self.can_Bus = can.Bus()
        self.dbc = cantools.database.load_file('/home/connau/ConnAu/DBC-ARXML/Mapless_PoC 2.dbc')
        self.output_MSG = self.dbc.get_message_by_frame_id(225)
        self.data_dict = {"LongDir_Rq":1, "StrgWhlAng_Rq":0.0, "Velocity_Rq":0.0, "Brake_Rq":0.0, "BrakeHoldReq":False}
        self.timer = self.create_timer(0.02, self.send_request)
        self.command_sub = self.create_subscription(VehicleCommand, "vehicle_command", self.update_command, 10)  
        self.curr_steering = 0.0
        self.curr_speed = 0.0
        self.curr_long_dir_rq = 0.0
        self.curr_brake_rq = 0.0
        self.curr_brake_hold_rq = 0.0

    def cleanup(self):
        self.get_logger().info("*****************************************, on cleanup")
        decel_rate = -0.3
        iter = 0
        while self.curr_speed > 0.0:
            self.curr_speed = max(0.0, self.curr_speed + decel_rate*0.02)
            self.data_dict = {"LongDir_Rq":self.curr_long_dir_rq, "StrgWhlAng_Rq":self.curr_steering, "Velocity_Rq":self.curr_speed, "Brake_Rq":self.curr_brake_rq, "BrakeHoldReq":self.curr_brake_hold_rq}
            self.send_request()
            time.sleep(0.02)
            iter += 1
            decel_rate = min(-0.1, decel_rate+iter/500)

    def update_command(self, command):
        self.data_dict = {"LongDir_Rq":command.long_dir_rq, "StrgWhlAng_Rq":command.strgwhlang_rq, "Velocity_Rq":command.velocity_rq, "Brake_Rq":command.brake_rq, "BrakeHoldReq":command.brake_hold_rq}
        self.curr_steering = command.strgwhlang_rq
        self.curr_speed = command.velocity_rq
        self.curr_long_dir_rq = command.long_dir_rq
        self.curr_brake_rq = command.brake_rq
        self.curr_brake_hold_rq = command.brake_hold_rq

    def send_request(self):

        if time.time() - self.start_time < 1:
            data_dict = {"LongDir_Rq":0, "StrgWhlAng_Rq":0.0, "Velocity_Rq":0.0, "Brake_Rq":0, "BrakeHoldReq":False}
            self.get_logger().info("rUNNING INIT COMMAND")
            data = self.output_MSG.encode(data_dict)
        # data_dict = {"LongDir_Rq":command.long_dir_rq, "StrgWhlAng_Rq":command.strgwhlang_rq, "Velocity_Rq":command.velocity_rq, "Brake_Rq":command.brake_rq, "BrakeHoldReq":command.brake_hold_rq}
        else:
            data = self.output_MSG.encode(self.data_dict)
        # data = self.output_MSG.encode(data_dict)
        msg = can.Message(arbitration_id=self.output_MSG.frame_id, data=data, is_extended_id = False)
        try:
            self.can_Bus.send(msg)
            print(f"Message sent on {self.can_Bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")
            print(f"{can.CanError}")


def main(args=None):
    rclpy.init(args=args)
    
    publisher = VehicleCommander(sys.argv)

    try:
        rclpy.spin(publisher)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # publisher.cleanup()
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
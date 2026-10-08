"""
VehicleCommanderSRP_noros.py — pure-Python (no ROS) trailer reverse-parking
controller.

Same functionality as VehicleCommanderSRP.py but with all ROS dependencies
removed. The script can run as a standalone process on any machine with:
  - python-can + SocketCAN (or 'virtual' for development)
  - cantools (DBC parsing)
  - paho-mqtt (UWB data ingestion)
  - numpy + scipy (filtering)

Differences from the ROS version:
  - No rclpy; uses threading + time-based loops instead of ROS timers
  - No `fusion.msg` types (they weren't actually used in the ROS version either)
  - Logging via `logging` module instead of `rclpy` get_logger
  - Same CAN frame layout, same MQTT topics, same DBC

CAN bus layout (DBC: TrailerReverse_PoC.dbc):
  IN:
    195 TrailerReverseFeedback   (TrajectoryReceived, TrailerReverseStatus)
    209 VehControlSta           (Velocity_Stat, StrgWhlAng_Stat)
  OUT (1Hz at startup, then on update):
    228 TrailerConfig           (one-shot)
    229 TrailerAxleConfig       (one-shot)
    230 VehUWBSensConfig        (one-shot)
    231 TrailerUWBsensConfig    (one-shot)
  OUT (10Hz from UWB):
    226 ManeuverControl         (HitchAngle, UWB readings, status)
  OUT (10Hz from GPS):
    235 ROSAdtlInfo1            (uBlox lat/lon)
    237 VehGPSLatLon            (vehicle GPS lat/lon)
    238 VehGPSHdg               (vehicle heading)
  OUT (during trajectory upload):
    233 TrailerDestination      (last waypoint)
    234 InitialTrajectory       (per-waypoint)

MQTT topics (Pi gateway at 192.168.5.51:1883):
  dwm/node/0c39/uplink/data   (left trailer tag)
  dwm/node/879c/uplink/data   (right trailer tag)

Files read at runtime (Hummer paths; override via env vars):
  /home/connau/srp/birdview/hummer_path/pathx200_dan.txt
    -> planned trajectory (JSON list of [x, y, h])
  /home/connau/srp/birdview/hummer_path/execution_tag_trailer.txt
    -> control flag file: '0' = stop, '1' = start
  /home/connau/srp/birdview/hummer_path/gps.txt
    -> u-blox GPS as JSON [lat, lon] (degrees, decimal)
  /home/connau/srp/birdview/hummer_path/can_heading.txt
    -> vehicle GPS+heading as JSON [lat, lon, heading]

Run:
    python3.11 VehicleCommanderSRP_noros.py
"""

import base64
import collections
import json
import logging
import math
import os
import struct
import sys
import threading
import time
from pathlib import Path

import can
import cantools
import numpy as np
import paho.mqtt.client as mqtt
from scipy.signal import butter, lfilter

# =============================================================================
# Configuration
# =============================================================================

# CAN bus
CAN_INTERFACE = os.environ.get('CAN_INTERFACE', 'socketcan')
CAN_CHANNEL = os.environ.get('CAN_CHANNEL', 'can0')
CAN_BITRATE = int(os.environ.get('CAN_BITRATE', '500000'))
DBC_PATH = Path(os.environ.get(
    'DBC_PATH',
    '/home/connau/ConnAu/DBC-ARXML/TrailerReverse_PoC.dbc'))

# MQTT
MQTT_BROKER_IP = os.environ.get('MQTT_BROKER_IP', '192.168.5.51')
MQTT_BROKER_PORT = int(os.environ.get('MQTT_BROKER_PORT', '1883'))

# UWB tag IDs (Decawave DWM1001)
TAG1 = '0c39'   # left
TAG2 = '879c'   # right
ANC = '442a'
MQTT_TOPIC1 = f'dwm/node/{TAG1}/uplink/data'
MQTT_TOPIC2 = f'dwm/node/{TAG2}/uplink/data'

# Vehicle/trailer geometry
VEH_LEN = 5.0
TRAILER_LEN = 5.0
RV = 0.85            # half-width between the two UWB tags on trailer (m)
L_ARM = 1.20         # lever arm: anchor to hinge along vehicle (m)
L_HT = 0.35          # hinge to tag-line along vehicle (m)
YH = -0.35 - VEH_LEN / 2.0          # hinge y in vehicle frame (m)
LT = 1.2 + TRAILER_LEN / 2.0        # hinge to trailer center (m)
ELE_OFFSET = 0.0     # mounting height diff (anchor vs tag), 0 = coplanar

# File paths (override via env vars for portability)
TRAJ_FILE = Path(os.environ.get(
    'TRAJ_FILE',
    '/home/connau/srp/birdview/hummer_path/pathx200_dan.txt'))
CONTROL_FILE = Path(os.environ.get(
    'CONTROL_FILE',
    '/home/connau/srp/birdview/hummer_path/execution_tag_trailer.txt'))
GPS_UBLOX_FILE = Path(os.environ.get(
    'GPS_UBLOX_FILE',
    '/home/connau/srp/birdview/hummer_path/gps.txt'))
GPS_VEH_FILE = Path(os.environ.get(
    'GPS_VEH_FILE',
    '/home/connau/srp/birdview/hummer_path/can_heading.txt'))
LOG_D1_FILE = Path(os.environ.get(
    'LOG_D1_FILE',
    '/home/connau/srp/birdview/hummer_path/uwb_recent_d1.txt'))
LOG_D2_FILE = Path(os.environ.get(
    'LOG_D2_FILE',
    '/home/connau/srp/birdview/hummer_path/uwb_recent_d2.txt'))
LOG_THETA_FILE = Path(os.environ.get(
    'LOG_THETA_FILE',
    '/home/connau/srp/birdview/hummer_path/uwb_recent_theta.txt'))

# Loop periods (seconds)
TRAJ_UPLOAD_PERIOD = 1.0
UBLOX_SEND_PERIOD = 0.1     # 10 Hz
MQTT_HEARTBEAT_PERIOD = 60  # reconnect cadence (paho internal)

# Status codes for TrailerReverseRequestStatus
STATUS_INACTIVE = 0
STATUS_ACTIVE = 1
STATUS_PAUSE = 2
STATUS_SENDING = 3
STATUS_SENDING_TRAJ = 4

# Status codes written to CONTROL_FILE
CTRL_STOP = 0
CTRL_START = 1
CTRL_SENDING = 3

# Filter
LP_ALPHA = 0.1
HISTORY_LEN = 10
LPF_CUTOFF_HZ = 2.0
LPF_SAMPLE_HZ = 10.0
LPF_ORDER = 2


# =============================================================================
# Logging
# =============================================================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S')
log = logging.getLogger('srp')


# =============================================================================
# Filters
# =============================================================================
def is_outlier(val, window, threshold=1.0):
    """Std-dev outlier rejection on a small rolling window."""
    if len(window) < 5:
        return False
    arr = np.array(window)
    return abs(val - np.mean(arr)) > threshold * np.std(arr)


class LowPassFilter:
    """First-order IIR low-pass filter (EMA). Stateful."""

    def __init__(self, alpha=LP_ALPHA):
        self.alpha = alpha
        self.prev = None

    def __call__(self, x):
        if self.prev is None:
            self.prev = x
            return x
        y = self.alpha * x + (1.0 - self.alpha) * self.prev
        self.prev = y
        return y

    def reset(self):
        self.prev = None


class ButterLowPass:
    """Butterworth IIR filter for batch processing."""

    def __init__(self, cutoff=LPF_CUTOFF_HZ, fs=LPF_SAMPLE_HZ, order=LPF_ORDER):
        nyq = 0.5 * fs
        b, a = butter(order, cutoff / nyq, btype='low', analog=False)
        self.b, self.a = b, a

    def __call__(self, data):
        return lfilter(self.b, self.a, data)


# =============================================================================
# Geometry: trailer angle & position from 2 UWB ranges
# =============================================================================
def estimate_trailer_pos_dual_vehicle_sensors(d1, d2):
    """
    Recover the trailer's yaw angle theta (deg) and trailer center (x, y)
    in the vehicle frame, given the two UWB range measurements d1 (left
    tag), d2 (right tag).

    Geometry: 2 tags on the trailer separated by 2*Rv; anchor on the vehicle.
    d1, d2 are planar distances (after elevation correction).
    """
    denominator = 4 * RV * L_ARM
    sin_theta = (d2 ** 2 - d1 ** 2) / denominator
    sin_theta = np.clip(sin_theta, -1.0, 1.0)
    theta_rad = np.arcsin(sin_theta)

    tx = LT * np.sin(theta_rad)
    ty = YH - LT * np.cos(theta_rad)

    return -np.degrees(theta_rad), (tx, ty)


# =============================================================================
# GPS helpers
# =============================================================================
def cast_gps_int(val):
    """Decimal degrees -> milli-arcseconds for DBC encoding."""
    return int(val * 3_600_000)


def load_ublox():
    """Read u-blox GPS (lat, lon) from disk. Returns ([lat, lon], None)."""
    if not GPS_UBLOX_FILE.exists():
        return [0, 0], None
    try:
        with open(GPS_UBLOX_FILE, 'r') as f:
            line = f.readline()
        data = json.loads(line)
        return [cast_gps_int(data[0]), cast_gps_int(data[1])], data
    except (json.JSONDecodeError, IndexError, ValueError) as e:
        log.warning('ublox gps read failed: %s', e)
        return [0, 0], None


def load_veh_gps():
    """Read vehicle GPS+heading (lat, lon, heading) from disk."""
    if not GPS_VEH_FILE.exists():
        return [0, 0, 0]
    try:
        with open(GPS_VEH_FILE, 'r') as f:
            line = f.readline()
        data = json.loads(line)
        return [cast_gps_int(data[0]), cast_gps_int(data[1]), data[2]]
    except (json.JSONDecodeError, IndexError, ValueError) as e:
        log.warning('veh gps read failed: %s', e)
        return [0, 0, 0]


# =============================================================================
# Trajectory file
# =============================================================================
def read_nested_list_from_file_json(filename):
    with open(filename, 'r') as f:
        return json.loads(f.read())


# =============================================================================
# Main controller (no ROS)
# =============================================================================
class VehicleCommanderSRP:
    def __init__(self):
        # CAN bus setup
        can.rc['interface'] = CAN_INTERFACE
        can.rc['channel'] = CAN_CHANNEL
        can.rc['bitrate'] = CAN_BITRATE
        self.can_bus = can.Bus()

        # Load DBC
        log.info('loading DBC from %s', DBC_PATH)
        self.dbc = cantools.database.load_file(str(DBC_PATH))

        # Input (RX) frames
        self.input_msg_195 = self.dbc.get_message_by_frame_id(195)
        self.input_msg_209 = self.dbc.get_message_by_frame_id(209)

        # Output (TX) frames
        self.output_msg_226 = self.dbc.get_message_by_frame_id(226)
        self.output_msg_228 = self.dbc.get_message_by_frame_id(228)
        self.output_msg_229 = self.dbc.get_message_by_frame_id(229)
        self.output_msg_230 = self.dbc.get_message_by_frame_id(230)
        self.output_msg_231 = self.dbc.get_message_by_frame_id(231)
        self.output_msg_233 = self.dbc.get_message_by_frame_id(233)
        self.output_msg_234 = self.dbc.get_message_by_frame_id(234)
        self.output_msg_235 = self.dbc.get_message_by_frame_id(235)
        self.output_msg_237 = self.dbc.get_message_by_frame_id(237)
        self.output_msg_238 = self.dbc.get_message_by_frame_id(238)

        # Initial CAN data dicts (mutable state, mutated from MQTT thread)
        self.data_dict_226 = {
            'VehUWBSens2_reading': 0.0,
            'VehUWBSens1_reading': 0.0,
            'HitchInclination': 0.0,
            'HitchAngle': 0.0,
            'TrailerReverseRequestStatus': STATUS_INACTIVE,
        }
        self.data_dict_228 = {'VehRrAxl2HtchBall': 1.1, 'TrailerWidth': 2.54,
                              'TrailerWheelbase': 0.56, 'TrailerPresent': 1,
                              'TrailerLength': 5.5}
        self.data_dict_229 = {'TrailerHtch2Axle3': 1.0, 'TrailerHtch2Axle2': 1.0,
                              'TrailerHtch2Axle1': 1.0}
        self.data_dict_230 = {'VehUWBSens2_z': 1.0, 'VehUWBSens2_y': 2.54,
                              'VehUWBSens2_x': -2.56, 'VehUWBSens1_z': 1.1,
                              'VehUWBSens1_y': 0.8, 'VehUWBSens1_x': -1.6}
        self.data_dict_231 = {'TrailerUWBsens_z': 1.0, 'TrailerUWBsens_y': 2.54,
                              'TrailerUWBsens_x': -2.56}
        self.data_dict_233 = {'TrailerDestination_y': 28.0,
                              'TrailerDestination_x': 4.0,
                              'TrailerDestination_Heading': 10.0}
        self.data_dict_235 = {'uBloxGPS_inv': -1, 'uBlox_Latitude': 0,
                              'uBlox_Longitude': 0}
        self.data_dict_237 = {'VehGPS_inv': -1, 'vehGPS_Latitude': 0,
                              'vehGPS_Longitude': 0}
        self.data_dict_238 = {'vehGPS_Heading': 0.0}

        # Thread-safety lock for data_dict_* mutation across threads
        self._lock = threading.Lock()

        # Trajectory state
        self.points = None
        self.is_traj_track_ready = False

        # UWB shared state (mutated from MQTT thread)
        self.uwb_data = {TAG1: 0.0, TAG2: 0.0}

        # Filters
        self.lpf_theta = LowPassFilter(alpha=LP_ALPHA)
        self.history_theta = collections.deque([0.0] * HISTORY_LEN,
                                               maxlen=HISTORY_LEN)

        # MQTT client (created but not connected here; main() does that)
        self.mqtt_client = None

        # Lifecycle flags
        self._running = True

        # Send static CAN frames once at startup
        self._send_static_frames()

    # -------------------------------------------------------------------------
    # CAN sending helpers
    # -------------------------------------------------------------------------
    def _send_can(self, msg, label='CAN'):
        try:
            self.can_bus.send(msg)
            log.debug('%s sent on %s', label, self.can_bus.channel_info)
        except can.CanError as e:
            log.warning('%s send failed: %s', label, e)

    def _send_static_frames(self):
        """Frames 228/229/230/231: static vehicle/trailer config, sent once."""
        for frame_id, data in [
            (228, self.data_dict_228),
            (229, self.data_dict_229),
            (230, self.data_dict_230),
            (231, self.data_dict_231),
        ]:
            msg_obj = self.dbc.get_message_by_frame_id(frame_id)
            encoded = msg_obj.encode(data)
            self._send_can(
                can.Message(
                    arbitration_id=msg_obj.frame_id,
                    data=encoded,
                    is_extended_id=False,
                ),
                label=f'static-{frame_id}',
            )

    def _send_msg(self, msg_obj, data_dict, label='CAN'):
        """Generic encoder + sender."""
        encoded = msg_obj.encode(data_dict)
        self._send_can(
            can.Message(
                arbitration_id=msg_obj.frame_id,
                data=encoded,
                is_extended_id=False,
            ),
            label=label,
        )

    # -------------------------------------------------------------------------
    # Frame 226: live maneuver control (called from MQTT thread)
    # -------------------------------------------------------------------------
    def on_uwb_update(self, d1, d2, theta_est, control_status):
        """
        Called from MQTT callback thread when both tag distances are fresh.
        Encodes the latest d1, d2, theta into frame 226 and sends.

        Args:
            d1: left tag distance (m, planar)
            d2: right tag distance (m, planar)
            theta_est: filtered trailer angle (deg)
            control_status: int from CONTROL_FILE
        """
        with self._lock:
            self.data_dict_226['VehUWBSens2_reading'] = d1
            self.data_dict_226['VehUWBSens1_reading'] = d2
            self.data_dict_226['HitchAngle'] = theta_est
            self.data_dict_226['TrailerReverseRequestStatus'] = control_status
            snapshot = dict(self.data_dict_226)   # shallow copy for the encode

        self._send_msg(self.output_msg_226, snapshot, label='ManeuverControl-226')

    # -------------------------------------------------------------------------
    # Periodic u-blox + vehicle GPS send (10 Hz)
    # -------------------------------------------------------------------------
    def send_ublox(self):
        """Read GPS files, send frame 235 (u-blox) + 237 (vehicle) + 238 (heading)."""
        try:
            data_ublox, _ = load_ublox()
            data_veh_gps = load_veh_gps()

            with self._lock:
                self.data_dict_235['uBloxGPS_inv'] = 0   # 0 = valid
                self.data_dict_235['uBlox_Latitude'] = data_ublox[0]
                self.data_dict_235['uBlox_Longitude'] = data_ublox[1]
                self.data_dict_237['VehGPS_inv'] = -1
                self.data_dict_237['vehGPS_Latitude'] = data_veh_gps[0]
                self.data_dict_237['vehGPS_Longitude'] = data_veh_gps[1]
                self.data_dict_238['vehGPS_Heading'] = data_veh_gps[2]

                d235 = dict(self.data_dict_235)
                d237 = dict(self.data_dict_237)
                d238 = dict(self.data_dict_238)

            self._send_msg(self.output_msg_235, d235, label='uBlox-235')
            self._send_msg(self.output_msg_237, d237, label='VehGPS-237')
            self._send_msg(self.output_msg_238, d238, label='VehHdg-238')

            log.debug('ublox sent: lat=%d lon=%d', data_ublox[0], data_ublox[1])
        except Exception as e:
            log.error('send_ublox failed: %s', e)

    # -------------------------------------------------------------------------
    # Trajectory upload (1 Hz)
    # -------------------------------------------------------------------------
    def send_new_traj(self):
        if not TRAJ_FILE.exists():
            log.debug('traj file not found: %s', TRAJ_FILE)
            return
        try:
            new_points = read_nested_list_from_file_json(str(TRAJ_FILE))
        except json.JSONDecodeError:
            log.warning('traj file %s is empty or invalid JSON', TRAJ_FILE)
            return

        if self.points == new_points:
            return

        log.info('new trajectory with %d waypoints', len(new_points))
        self.points = new_points
        self.is_traj_track_ready = False

        traj_x, traj_y, traj_h = [], [], []
        for pt in new_points:
            traj_x.append(pt[0])
            traj_y.append(pt[1])
            traj_h.append(pt[2])

        # announce
        with self._lock:
            self.data_dict_226['TrailerReverseRequestStatus'] = STATUS_SENDING_TRAJ
        self._send_msg(self.output_msg_226,
                       dict(self.data_dict_226), label='Status=4')
        self._write_control(CTRL_SENDING)
        time.sleep(0.1)

        traj_max_id = min(len(traj_x) - 1, 255)
        msg_len = min(len(traj_x), 200)

        # send each waypoint (5x for first and last as a reliability padding)
        for i in range(msg_len):
            loop = 5 if (i == 0 or i == msg_len - 1) else 1
            for _ in range(loop):
                data_234 = {
                    'InitialTrajectory_y': traj_y[i],
                    'InitialTrajectory_x': traj_x[i],
                    'InitialTrajectory_MaxID': traj_max_id,
                    'InitialTrajectory_ID': i,
                    'InitialTrajectory_Heading': traj_h[i],
                }
                self._send_msg(self.output_msg_234, data_234,
                               label=f'Traj-{i}')
                time.sleep(0.02)

        time.sleep(0.1)

        # mark ready
        with self._lock:
            self.data_dict_226['TrailerReverseRequestStatus'] = STATUS_ACTIVE
        self._send_msg(self.output_msg_226, dict(self.data_dict_226),
                       label='Status=3')
        self.is_traj_track_ready = True

    def _write_control(self, value):
        try:
            with open(CONTROL_FILE, 'w') as f:
                f.write(str(value))
        except OSError as e:
            log.warning('control file write failed: %s', e)

    def _read_control(self):
        try:
            with open(CONTROL_FILE, 'r') as f:
                return int(f.readline().strip())
        except (OSError, ValueError):
            return CTRL_STOP

    # -------------------------------------------------------------------------
    # Loops
    # -------------------------------------------------------------------------
    def _traj_loop(self):
        while self._running:
            self.send_new_traj()
            time.sleep(TRAJ_UPLOAD_PERIOD)

    def _ublox_loop(self):
        while self._running:
            self.send_ublox()
            time.sleep(UBLOX_SEND_PERIOD)

    # -------------------------------------------------------------------------
    # MQTT
    # -------------------------------------------------------------------------
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(MQTT_TOPIC1)
            client.subscribe(MQTT_TOPIC2)
            log.info('MQTT connected, subscribed to %s + %s',
                     MQTT_TOPIC1, MQTT_TOPIC2)
        else:
            log.error('MQTT connect failed: rc=%s', rc)

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            raw_data = payload.get('data')
            if not raw_data:
                return

            sid = TAG1 if TAG1 in msg.topic else (
                TAG2 if TAG2 in msg.topic else None)
            if sid is None:
                return

            # 6-byte Decawave DWM1001 packet
            dec = base64.b64decode(raw_data)
            addr_l, addr_h, b1, b2, b3, b4 = struct.unpack('BBBBBB', dec)
            addr = f'{addr_h:02x}{addr_l:02x}'
            dis = (b1 + b2 * 256 + b3 * 256 ** 2 + b4 * 256 ** 3) / 1000.0
            if dis <= 0:
                return

            # store under lock; other thread may be reading concurrently
            with self._lock:
                self.uwb_data[sid] = dis
                d1 = self.uwb_data[TAG1]
                d2 = self.uwb_data[TAG2]

            # elevation correction (Pythagorean)
            d1 = math.sqrt(max(0.0, d1 ** 2 - ELE_OFFSET ** 2))
            d2 = math.sqrt(max(0.0, d2 ** 2 - ELE_OFFSET ** 2))

            # recover trailer angle
            theta_est, _ = estimate_trailer_pos_dual_vehicle_sensors(d1, d2)

            # low-pass filter (stateful)
            theta_lpf = self.lpf_theta(theta_est)
            self.history_theta.append(theta_lpf)
            theta_avg = sum(self.history_theta) / len(self.history_theta)

            # use the filtered value (FIX vs the ROS version which passed raw)
            control_status = self._read_control()

            # send to CAN frame 226
            self.on_uwb_update(d1, d2, theta_lpf, control_status)

            # write recent readings to disk (debug log)
            time_ms = int(time.time() * 1000)
            log_fn = LOG_D1_FILE if sid == TAG1 else LOG_D2_FILE
            try:
                with open(log_fn, 'w') as f:
                    f.write(f'[{time_ms},{dis:.3f}]')
                rad = math.radians(theta_lpf)
                tx = LT * math.sin(rad)
                ty = YH - LT * math.cos(rad)
                with open(LOG_THETA_FILE, 'w') as f:
                    f.write(f'[{time_ms},{theta_lpf:.3f},'
                            f'{tx:.3f},{ty:.3f}]')
            except OSError:
                pass

        except Exception as e:
            log.error('mqtt msg error: %s', e)

    def setup_mqtt(self):
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        self.mqtt_client.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT, 60)
        self.mqtt_client.loop_start()

    def stop(self):
        log.info('shutting down...')
        self._running = False
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

    # -------------------------------------------------------------------------
    # Main run
    # -------------------------------------------------------------------------
    def run(self):
        log.info('starting non-ROS VehicleCommanderSRP')

        # Start background loops
        traj_thread = threading.Thread(target=self._traj_loop,
                                      name='traj-loop', daemon=True)
        ublox_thread = threading.Thread(target=self._ublox_loop,
                                        name='ublox-loop', daemon=True)
        traj_thread.start()
        ublox_thread.start()

        # Start MQTT
        self.setup_mqtt()

        try:
            # Main thread just waits; MQTT/loops do the work
            while self._running:
                time.sleep(0.5)
        except KeyboardInterrupt:
            log.info('Ctrl-C received')
        finally:
            self.stop()


# =============================================================================
# Entry point
# =============================================================================
def main():
    cmd = VehicleCommanderSRP()
    cmd.run()


if __name__ == '__main__':
    main()

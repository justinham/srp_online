from fusion.msg import Track 
from fusion.msg import GPS
from fusion.msg import Acc
from fusion.msg import Vel

import numpy as np

import rclpy
import rclpy.duration
from visualization_msgs.msg import Marker



"""
General Notes:
    1.  Currently using the time at message creation for timestamp, which may result in some in-accuracy
        This may need to be changed for using Aquisition timestamp, but I have no idea as of now how it works as it is reported
        in ms, and ranges from 0-2047 and is only reported in a single frame per burst
    2. Sensor Ids: 0 - Infra, 1 - LRR, 2 - FCM, 3- SRRLF, 4 - SRRRF

"""
VISUALIZE = True
USE_CALIBRATED_VALUES = False

def visualize(node, msgs):
    for m in msgs:
        #if m.confidence < 2:
        #    continue
        if m.sensor_id > 2:
            continue
        marker = Marker()

        #Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/my_frame"
        marker.header.stamp = node.get_clock().now().to_msg()

        #Set the namespace and id for this marker.  This serves to create a unique ID
        #Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = str(m.sensor_id)
        marker.id = m.track_id

        #Set the marker type.
        

        #Set the marker action.  Options are ADD, DELETE, and new in rclcpp Indigo: 3 (DELETEALL)
        marker.action = 0

        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = -m.lat
        marker.pose.position.y = m.lon
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        if m.object_type:
            marker.type = 2
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
        else:
            marker.type = 1
            marker.scale.x = 2.0
            marker.scale.y = 3.0
            marker.scale.z = 1.0

        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.50

        match m.sensor_id:
            case 0:
                pass
            case 1:
                marker.color.r = 1.0
            case 2:
                marker.color.b = 1.0
            case 3:
                marker.color.g = 1.0
            case 4:
                marker.color.r = 1.0
                marker.color.b = 1.0
            case _:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0

        node.publisher_visual.publish(marker)


        marker.text = str(m.sensor_id) + "-" + str(m.track_id)
        marker.ns = "text"
        marker.id = int(int(m.sensor_id)*1000) + int(m.track_id)
        marker.type = 9 # Text
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 2.0
        node.publisher_visual.publish(marker)

def mesaurement_status_str_to_int(status):
    match status:
        case 'New Object':
            return 1
        case 'Latent Track Not Detected this Cycle':
            return 2
        case 'Measured this Cycle':
            return 3
        case _:
            print("Measurement Status does not make senese")
            return 0

def confidence_str_to_int(conf):
    match conf:
        case "Highly Speculative":
            return 0
        case "Speculative":
            return 1
        case "Confident":
            return 2
        case "Highly Confident":
            return 3
        case _:
            print("Confidence does not make sense")
            return 4

def brake_to_int(brk):
    match brk:
        case "UNKOWN":
            return 0
        case "OFF":
            return 1
        case "ON":
            return 2
        case _:
            print(brk)
            print("Brake Status Does not make sense")
            return 0

def turn_to_int(turn):
    match turn:
        case "UNKNOWN":
            return 0
        case "OFF":
            return 1
        case "LEFT":
            return 2
        case "RIGHT":
            return 3
        case "BOTH":
            return 4
        case _:
            print(turn)
            print("Turn Signal Does not make sense")
            return 0
            
def valid_to_bool(v):
    if "TRUE":
        return True
    else:
        return False

def can_to_message_24Lyriq_5(node, frame_id, data):
    pass

# ---------------------------------------------------------
# | NOTE: Inverted Lat as direction of +/- was incorrect compared to what the system expected
# ---------------------------------------------------
def can_to_message_24Lyriq_8(node, frame_id, data):
    # Use string representation here lower case
    msgs = []
    if frame_id in ['e9', 'de', 'ea', 'dd', 'eb', 'dc', 'ec', 'db']: #FCM Messages, there are more messages not translated, check DBC for more info
        for m in lyriq_24_fcm(node, frame_id, data):
            msgs.append(m)

    elif frame_id in ['d1', 'f7', 'd0', 'f8', 'cf', 'f9']: #SSRRF Messages
        for m in lyriq_24_srrrf(node, frame_id, data):
            msgs.append(m)

    elif frame_id in ['d4', 'f4', 'd3', 'f5', 'd2', 'f6']: # SSRLF Messages
        for m in lyriq_24_srrlf(node, frame_id, data):
            msgs.append(m)

    elif frame_id in ['d7', 'f1', 'd6', 'f2', 'd5', 'f3']: #LRR Messages
        for m in lyriq_24_lrr(node, frame_id, data):
            msgs.append(m)

    for m in msgs:
        if USE_CALIBRATED_VALUES and not m.object_type:
            match m.sensor_id:
                case 1: # LRR
    
                    #m.lat -= (-0.08703757198522322 * m.lat + 0.4975083707270722)
                    #m.covariance[0] = 1.8251249364952613 ** 2

                    m.lat -= -2.419053869245737
                    m.covariance[0] = 1.4507724901249546 ** 2

                    m.covariance[5] = 1.1397391860704769 ** 2
                    m.lon -= (0.012870600441629344*m.lat_vel -2.7469466928161044)

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    # Made this stat worse
                    #m.lat_vel -= 0.9417774655697115
                    m.covariance[10] = 1.0854550774863672 ** 2
                    # m.lat_vel -= -(1.0799586626974822*np.log(m.lat_vel+10) -2.3438068798389535)

                    m.covariance[15] = 0.79430056505056 ** 2
                    m.lon_vel -= (0.037659021346730714*m.lat_vel + 0.03565600889935616)
                    #m.covariance[15] = 1.184748728520708 ** 2
                    # m.lon_vel -= (0.2530452180355196 * m.lon_vel + 0.03411408759271623)

                case 2: # FCM
                    # No correlation for lat as such we will use the std of the error for the variance
                    # m.covariance[0] = 0.30008838371420826 ** 2
                    # m.lat -= (-0.0398772958817702 * m.lat - 1.1794496291252585)
                    m.lat -= -2.6151958699023945
                    m.covariance[0] = 0.7484454379075511 ** 2

                    m.lon -= (0.168112845695765 * m.lon -6.738610928812756)
                    m.covariance[5] =  3.937394235297366 ** 2

                    m.covariance[10] = 0.4308738226325788 ** 2
                    m.lat_vel -= -(0.03223546424250756*m.lat_vel -0.0010261241364479812)
                    #m.covariance[10] = 0.7102863492151918 ** 2
                    #m.lat_vel -= (-0.21224574323672737 * m.lat_vel + 0.32216676683938134)

                    m.covariance[15] = 1.1030768456015807 ** 2
                    m.lon_vel -= (0.09094914483207966*m.lat_vel + 0.24035786417876123)
                    #m.covariance[15] = 1.283579459334189 ** 2
                    # m.lon_vel -= (0.1504358365070556 * m.lon_vel - 1.4505595212837212)
                    
                case 3: # L-SRR

                    if abs(m.lat) < -63.5:
                        m.covariance[0] = 10.0
                    else:
                        #m.lat -= (-0.024900632558547273 *m.lat + 0.6936446611670223)
                        #m.covariance[0] = 1.993318703135391 ** 2
                        m.lat -= -2.3014358422303145
                        m.covariance[0] = 1.2578432504386736 ** 2

                    m.lon -= -2.0499637683019896
                    m.covariance[5] = 0.9659009891999046 ** 2

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.lat_vel -= -(0.03062625964147117)
                    m.covariance[10] = 0.8450576530193229 ** 2

                    # No benefit from function statistically
                    m.covariance[15] = 0.579667585031832 ** 2
                    m.lon_vel -= -0.07676799469277824

                case 4: # R-SRR
                    
                    # m.covariance[5] = 1.1248463656192702 ** 2
                    # if m.lon > 0:
                    #     m.lon -= (-2.2512233591184434 * np.log(m.lon) + 2.68004230709231)

                    m.lon -= -2.8816321769528144
                    m.covariance[5] = 1.2290563769856384 ** 2

                    if abs(m.lat) > 63.5:
                        m.covariance[0] = 10.0
                    else:
                        m.lat -= -(-0.003213205280205189*m.lat + 3.584112920232004)
                        m.covariance[0] = 0.9737339448650071 ** 2

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.lat_vel -= (-0.2714965952052286)
                    m.covariance[10] = 1.0676994554955777 ** 2

                    m.covariance[15] = 0.48150865881985333 ** 2
                    m.lon_vel -= (0.038288190138033196 * m.lon_vel + 0.08267236285070168)

        if USE_CALIBRATED_VALUES and m.object_type:
            match m.sensor_id:
                case 1: # LRR
                    pass

                case 2: # FCM
                    # No correlation for lat as such we will use the std of the error for the variance
                    # m.covariance[0] = 0.30008838371420826 ** 2
                    # m.lat -= (-0.0398772958817702 * m.lat - 1.1794496291252585)
                    if m.lat < 7.5:
                        m.lat -= -(-0.06322335908015714*m.lat - 2.7744502150974757)
                        m.covariance[0] = 0.3424351274043499 ** 2
                    else:
                        m.lat -= -(0.5684410479610241*m.lat - 7.217438758770081)
                        m.covariance[0] = 0.46083350572245413 ** 2

                    m.lon -= -5.049969366161228
                    m.covariance[5] = 0.49608680767215635 ** 2

                    m.covariance[10] = 0.22983512087705454 ** 2
                    m.lat_vel -= (-0.10043768142975318*m.lat_vel + 0.021023771237105526)
                    #m.covariance[10] = 0.7102863492151918 ** 2
                    #m.lat_vel -= (-0.21224574323672737 * m.lat_vel + 0.32216676683938134)

                    m.covariance[15] = 0.13489774457043874 ** 2
                    m.lon_vel -= (0.8740313651511209*m.lat_vel - 0.0058316530875448525)
                    #m.covariance[15] = 1.283579459334189 ** 2
                    # m.lon_vel -= (0.1504358365070556 * m.lon_vel - 1.4505595212837212)
                    
                case 3: # L-SRR
                    

                    m.lat -= 2.8665679316335835
                    m.covariance[0] = 0.37093576558550856 ** 2
                    
                    m.lon -= (0.03839420241979141*m.lon - 4.225827958170893)
                    m.covariance[5] = 0.3024094213947817 ** 2

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.lat_vel -= 0.0715261316022425
                    m.covariance[10] = 0.35518310627369415 ** 2

                    # No benefit from function statistically
                    m.covariance[15] = 0.3072655279141364 ** 2
                    m.lon_vel -= -0.007204749050194861

                case 4: # R-SRR
                    
                    # m.covariance[5] = 1.1248463656192702 ** 2
                    # if m.lon > 0:
                    #     m.lon -= (-2.2512233591184434 * np.log(m.lon) + 2.68004230709231)

                    m.lat -= -(-2.713488160815049)
                    m.covariance[0] = 0.2775766348184913 ** 2

                    m.lon -= (0.038442122960180125*m.lon - 4.205272298021499)
                    m.covariance[5] = 0.3274803477185302 ** 2

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.lat_vel -= 0.052271318007928814
                    m.covariance[10] = 0.29379141403331427 ** 2

                    m.covariance[15] = 0.27799312414147753 ** 2
                    m.lon_vel -= -0.010833188990531801

    if VISUALIZE:
        visualize(node, msgs)
        
    return msgs

def lyriq_24_fcm(node, frame_id, data):
    msgs = []
    if type(data) is not str:
        binary_data = data
    else:
        binary_data = bytes.fromhex(data)
    decoded = node.db_can8.decode_message(int(frame_id, 16), binary_data)
    
    match frame_id:
        case 'e9':
            # Pedestrian Track 1
            if decoded["FCODPB1P_FVPOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB1P_FVPOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB1P_FVPOTk1MStsAuth"])
                #print(decoded["FCODPB1P_FVPOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB1P_FVPOTk1CnfdStsAuth"])
                #print(decoded["FCODPB1P_FVPOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB1P_FVPOTk1PoILPsAuth"]
                new_m1.lon = decoded["FCODPB1P_FVPOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB1P_FVPOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB1P_FVPOTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_cov = 10.0 # Not provided by FC
                new_m1.theta_valid = False

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                
                

                msgs.append(new_m1)

            # Vehicle Track 1
            if decoded["FCODPB1P_FVVOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB1P_FVVOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB1P_FVVOTk1MStsAuth"])
                #print(decoded["FCODPB1P_FVVOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB1P_FVVOTk1CnfdStsAuth"])
                #print(decoded["FCODPB1P_FVVOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB1P_FVVOTk1PoILPsAuth"]
                new_m1.lon = decoded["FCODPB1P_FVVOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB1P_FVVOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB1P_FVVOTk1PoILgVAuth"]
                new_m1.theta = decoded["FCODPB1P_FVVOTk1HdgAngAuth"]
                 # Not provided by FC

                new_m1.theta_cov = decoded["FCODPB1P_FVVOTk1HdgAngCAuth"] #theta cov
                new_m1.theta_valid = True
                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB1P_FVVOTk1PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB1P_FVVOTk1PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB1P_FVVOTk1PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB1P_FVVOTk1PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB1P_FVVOTk1PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB1P_FVVOTk1BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB1P_FVVOTk1TSLgtStsAuth"])

                msgs.append(new_m1)

        case 'de':
            # Pedestrian Track 2
            if decoded["FCODPB2P_FVPOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB2P_FVPOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB2P_FVPOTk2MStsAuth"])
                #print(decoded["FCODPB2P_FVPOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB2P_FVPOTk2CnfdStsAuth"])
                #print(decoded["FCODPB2P_FVPOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB2P_FVPOTk2PoILPsAuth"]
                new_m1.lon = decoded["FCODPB2P_FVPOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB2P_FVPOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB2P_FVPOTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_cov = 10.0 # Not provided by FC
                new_m1.theta_valid = False

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 2
            if decoded["FCODPB2P_FVVOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB2P_FVVOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB2P_FVVOTk2MStsAuth"])
                #print(decoded["FCODPB2P_FVVOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB2P_FVVOTk2CnfdStsAuth"])
                #print(decoded["FCODPB2P_FVVOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB2P_FVVOTk2PoILPsAuth"]
                new_m1.lon = decoded["FCODPB2P_FVVOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB2P_FVVOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB2P_FVVOTk2PoILgVAuth"]
                new_m1.theta = decoded["FCODPB2P_FVVOTk2HdgAngAuth"]
                 
                new_m1.theta_cov = decoded["FCODPB2P_FVVOTk2HdgAngCAuth"] #theta cov
                new_m1.theta_valid = True

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB2P_FVVOTk2PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB2P_FVVOTk2PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB2P_FVVOTk2PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB2P_FVVOTk2PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB2P_FVVOTk2PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB2P_FVVOTk2BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB2P_FVVOTk2TSLgtStsAuth"])

                msgs.append(new_m1)

        case 'ea':
            # Pedestrian Track 3
            if decoded["FCODPB3P_FVPOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB3P_FVPOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB3P_FVPOTk3MStsAuth"])
                #print(decoded["FCODPB3P_FVPOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB3P_FVPOTk3CnfdStsAuth"])
                #print(decoded["FCODPB3P_FVPOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB3P_FVPOTk3PoILPsAuth"]
                new_m1.lon = decoded["FCODPB3P_FVPOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB3P_FVPOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB3P_FVPOTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_cov = 10.0
                new_m1.theta_valid = False

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 3
            if decoded["FCODPB3P_FVVOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB3P_FVVOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB3P_FVVOTk3MStsAuth"])
                #print(decoded["FCODPB3P_FVVOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB3P_FVVOTk3CnfdStsAuth"])
                #print(decoded["FCODPB3P_FVVOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB3P_FVVOTk3PoILPsAuth"]
                new_m1.lon = decoded["FCODPB3P_FVVOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB3P_FVVOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB3P_FVVOTk3PoILgVAuth"]
                new_m1.theta = decoded["FCODPB3P_FVVOTk3HdgAngAuth"]
                new_m1.theta_cov = decoded["FCODPB3P_FVVOTk3HdgAngCAuth"] #theta cov
                new_m1.theta_valid = True

                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB3P_FVVOTk3PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB3P_FVVOTk3PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB3P_FVVOTk3PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB3P_FVVOTk3PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB3P_FVVOTk3PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB3P_FVVOTk3BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB3P_FVVOTk3TSLgtStsAuth"])

                msgs.append(new_m1)
        
        case 'dd':
            # Pedestrian Track 4
            if decoded["FCODPB4P_FVPOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB4P_FVPOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB4P_FVPOTk4MStsAuth"])
                #print(decoded["FCODPB4P_FVPOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB4P_FVPOTk4CnfdStsAuth"])
                #print(decoded["FCODPB4P_FVPOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB4P_FVPOTk4PoILPsAuth"]
                new_m1.lon = decoded["FCODPB4P_FVPOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB4P_FVPOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB4P_FVPOTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_valid = False
                new_m1.theta_cov = 10.0 #theta 

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 4
            if decoded["FCODPB4P_FVVOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB4P_FVVOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB4P_FVVOTk4MStsAuth"])
                #print(decoded["FCODPB4P_FVVOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB4P_FVVOTk4CnfdStsAuth"])
                #print(decoded["FCODPB4P_FVVOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB4P_FVVOTk4PoILPsAuth"]
                new_m1.lon = decoded["FCODPB4P_FVVOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB4P_FVVOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB4P_FVVOTk4PoILgVAuth"]
                new_m1.theta = decoded["FCODPB4P_FVVOTk4HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB4P_FVVOTk4HdgAngCAuth"] #theta cov
                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["FCODPB4P_FVVOTk4PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB4P_FVVOTk4PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB4P_FVVOTk4PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB4P_FVVOTk4PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB4P_FVVOTk4PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB4P_FVVOTk4BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB4P_FVVOTk4TSLgtStsAuth"])

                msgs.append(new_m1)

        case 'eb':
            # Pedestrian Track 5
            if decoded["FCODPB5P_FVPOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB5P_FVPOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB5P_FVPOTk5MStsAuth"])
                #print(decoded["FCODPB5P_FVPOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB5P_FVPOTk5CnfdStsAuth"])
                #print(decoded["FCODPB5P_FVPOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB5P_FVPOTk5PoILPsAuth"]
                new_m1.lon = decoded["FCODPB5P_FVPOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB5P_FVPOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB5P_FVPOTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_valid = False # Not provided by FC
                new_m1.theta_cov = 10.0

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 5
            if decoded["FCODPB5P_FVVOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB5P_FVVOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB5P_FVVOTk5MStsAuth"])
                #print(decoded["FCODPB5P_FVVOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB5P_FVVOTk5CnfdStsAuth"])
                #print(decoded["FCODPB5P_FVVOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB5P_FVVOTk5PoILPsAuth"]
                new_m1.lon = decoded["FCODPB5P_FVVOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB5P_FVVOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB5P_FVVOTk5PoILgVAuth"]
                new_m1.theta = decoded["FCODPB5P_FVVOTk5HdgAngAuth"]
                new_m1.theta_valid = True # Not provided by FC
                new_m1.theta_cov = decoded["FCODPB5P_FVVOTk5HdgAngCAuth"] #theta cov
                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB5P_FVVOTk5PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB5P_FVVOTk5PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB5P_FVVOTk5PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB5P_FVVOTk5PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB5P_FVVOTk5PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB5P_FVVOTk5BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB5P_FVVOTk5TSLgtStsAuth"])

                msgs.append(new_m1)
        
        case 'dc':
            
            # Vehicle Track 6
            if decoded["FCODPB6P_FVVOTk6MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB6P_FVVOTk6IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB6P_FVVOTk6MStsAuth"])
                #print(decoded["FCODPB6P_FVVOTk6MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB6P_FVVOTk6CnfdStsAuth"])
                #print(decoded["FCODPB6P_FVVOTk6CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB6P_FVVOTk6PoILPsAuth"]
                new_m1.lon = decoded["FCODPB6P_FVVOTk6PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB6P_FVVOTk6PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB6P_FVVOTk6PoILgVAuth"]
                new_m1.theta = decoded["FCODPB6P_FVVOTk6HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB6P_FVVOTk6HdgAngCAuth"] #theta cov
                 # Not provided by FC

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB6P_FVVOTk6PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB6P_FVVOTk6PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB6P_FVVOTk6PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB6P_FVVOTk6PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB6P_FVVOTk6PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB6P_FVVOTk6BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB6P_FVVOTk6TSLgtStsAuth"])

                msgs.append(new_m1)

            # Vehicle Track 10
            if decoded["FCODPB6P_FVVOTk10MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB6P_FVVOTk10IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB6P_FVVOTk10MStsAuth"])
                #print(decoded["FCODPB6P_FVVOTk10MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB6P_FVVOTk10CnfdStsAuth"])
                #print(decoded["FCODPB6P_FVVOTk10CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB6P_FVVOTk10PoILPsAuth"]
                new_m1.lon = decoded["FCODPB6P_FVVOTk10PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB6P_FVVOTk10PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB6P_FVVOTk10PoILgVAuth"]
                new_m1.theta = decoded["FCODPB6P_FVVOTk10HdgAngAuth"]
                new_m1.theta_cov = decoded["FCODPB6P_FVVOTk10HdgAngCAuth"] #theta cov
                new_m1.theta_valid = True
                 # Not provided by FC

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["FCODPB6P_FVVOTk10PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB6P_FVVOTk10PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB6P_FVVOTk10PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB6P_FVVOTk10PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB6P_FVVOTk10PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB6P_FVVOTk10BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB6P_FVVOTk10TSLgtStsAuth"])

                msgs.append(new_m1)

        case 'ec':
            
            # Vehicle Track 7
            if decoded["FCODPB7P_FVVOTk7MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB7P_FVVOTk7IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB7P_FVVOTk7MStsAuth"])
                #print(decoded["FCODPB7P_FVVOTk7MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB7P_FVVOTk7CnfdStsAuth"])
                #print(decoded["FCODPB7P_FVVOTk7CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB7P_FVVOTk7PoILPsAuth"]
                new_m1.lon = decoded["FCODPB7P_FVVOTk7PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB7P_FVVOTk7PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB7P_FVVOTk7PoILgVAuth"]
                new_m1.theta = decoded["FCODPB7P_FVVOTk7HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB7P_FVVOTk7HdgAngCAuth"] #theta cov
                 # Not provided by FC

                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB7P_FVVOTk7PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB7P_FVVOTk7PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB7P_FVVOTk7PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB7P_FVVOTk7PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB7P_FVVOTk7PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB7P_FVVOTk7BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB7P_FVVOTk7TSLgtStsAuth"])

                msgs.append(new_m1)

        case 'db':
            
            # Vehicle Track 8
            if decoded["FCODPB8P_FVVOTk8MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB8P_FVVOTk8IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB8P_FVVOTk8MStsAuth"])
                #print(decoded["FCODPB8P_FVVOTk8MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB8P_FVVOTk8CnfdStsAuth"])
                #print(decoded["FCODPB8P_FVVOTk8CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB8P_FVVOTk8PoILPsAuth"]
                new_m1.lon = decoded["FCODPB8P_FVVOTk8PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB8P_FVVOTk8PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB8P_FVVOTk8PoILgVAuth"]
                new_m1.theta = decoded["FCODPB8P_FVVOTk8HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB8P_FVVOTk8HdgAngCAuth"] #theta cov
                 # Not provided by FC

                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB8P_FVVOTk8PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB8P_FVVOTk8PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["FCODPB8P_FVVOTk8PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB8P_FVVOTk8PoILgVCAuth"] #lon vel cov 
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB8P_FVVOTk8PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB8P_FVVOTk8BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB8P_FVVOTk8TSLgtStsAuth"])

                msgs.append(new_m1)

            # Vehicle Track 9
            if decoded["FCODPB8P_FVVOTk9MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB8P_FVVOTk9IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB8P_FVVOTk9MStsAuth"])
                #print(decoded["FCODPB8P_FVVOTk9MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB8P_FVVOTk9CnfdStsAuth"])
                #print(decoded["FCODPB8P_FVVOTk9CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB8P_FVVOTk9PoILPsAuth"]
                new_m1.lon = decoded["FCODPB8P_FVVOTk9PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB8P_FVVOTk9PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB8P_FVVOTk9PoILgVAuth"]
                new_m1.theta = decoded["FCODPB8P_FVVOTk9HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB8P_FVVOTk9HdgAngCAuth"] #theta cov
                 # Not provided by FC

                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["FCODPB8P_FVVOTk9PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB8P_FVVOTk9PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB8P_FVVOTk9PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB8P_FVVOTk9PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB8P_FVVOTk9PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB8P_FVVOTk9BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB8P_FVVOTk9TSLgtStsAuth"])

                msgs.append(new_m1)

        case _:
            node.get_logger().info('Unrecognized Frame ID in FCM conversion: %s' % frame_id)

    return msgs

def lyriq_24_srrlf(node, frame_id, data):
    msgs = []
    if type(data) is not str:
        binary_data = data
    else:
        binary_data = bytes.fromhex(data)
    decoded = node.db_can8.decode_message(int(frame_id, 16), binary_data)
    match frame_id:
        case 'd4': # First Frame
            # Pedestrian Track 1
            if decoded["SRRLFODPB1P_POTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB1P_POTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB1P_POTk1MStsAuth"])
                #print(decoded["SRRLFODPB1P_POTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB1P_POTk1CnfdStsAuth"])
                #print(decoded["SRRLFODPB1P_POTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB1P_POTk1PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB1P_POTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB1P_POTk1PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB1P_POTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Pedestrian Track 2
            if decoded["SRRLFODPB1P_POTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB1P_POTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB1P_POTk2MStsAuth"])
                #print(decoded["SRRLFODPB1P_POTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB1P_POTk2CnfdStsAuth"])
                #print(decoded["SRRLFODPB1P_POTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB1P_POTk2PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB1P_POTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB1P_POTk2PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB1P_POTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 1
            if decoded["SRRLFODPB1P_VOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB1P_VOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB1P_VOTk1MStsAuth"])
                #print(decoded["SRRLFODPB1P_VOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB1P_VOTk1CnfdStsAuth"])
                #print(decoded["SRRLFODPB1P_VOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB1P_VOTk1PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB1P_VOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB1P_VOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB1P_VOTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB1P_VOTk1PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB1P_VOTk1PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["SRRLFODPB1P_VOTk1PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB1P_VOTk1PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)

        case 'f4': # Consec Frame 1
            # Pedestrian Track 3
            if decoded["SRRLFODPB2P_POTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB2P_POTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB2P_POTk3MStsAuth"])
                #print(decoded["SRRLFODPB2P_POTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB2P_POTk3CnfdStsAuth"])
                #print(decoded["SRRLFODPB2P_POTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB2P_POTk3PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB2P_POTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB2P_POTk3PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB2P_POTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 2
            if decoded["SRRLFODPB2P_VOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB2P_VOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB2P_VOTk2MStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB2P_VOTk2CnfdStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB2P_VOTk2PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB2P_VOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB2P_VOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB2P_VOTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB2P_VOTk2PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB2P_VOTk2PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["SRRLFODPB2P_VOTk2PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB2P_VOTk2PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)
            
            # Vehicle Track 3
            if decoded["SRRLFODPB2P_VOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB2P_VOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB2P_VOTk3MStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB2P_VOTk3CnfdStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB2P_VOTk3PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB2P_VOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB2P_VOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB2P_VOTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB2P_VOTk3PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB2P_VOTk3PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB2P_VOTk3PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB2P_VOTk3PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 4
            if decoded["SRRLFODPB2P_VOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB2P_VOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB2P_VOTk4MStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB2P_VOTk4CnfdStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB2P_VOTk4PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB2P_VOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB2P_VOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB2P_VOTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB2P_VOTk4PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB2P_VOTk4PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB2P_VOTk4PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB2P_VOTk4PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'd3': # Consec Frame 2
            # Pedestrian Track 4
            if decoded["SRRLFODPB3P_POTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB3P_POTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB3P_POTk4MStsAuth"])
                #print(decoded["SRRLFODPB3P_POTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB3P_POTk4CnfdStsAuth"])
                #print(decoded["SRRLFODPB3P_POTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB3P_POTk4PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB3P_POTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB3P_POTk4PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB3P_POTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 5
            if decoded["SRRLFODPB3P_VOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB3P_VOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB3P_VOTk5MStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB3P_VOTk5CnfdStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB3P_VOTk5PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB3P_VOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB3P_VOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB3P_VOTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB3P_VOTk5PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB3P_VOTk5PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB3P_VOTk5PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB3P_VOTk5PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 6
            if decoded["SRRLFODPB3P_VOTk6MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB3P_VOTk6IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB3P_VOTk6MStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk6MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB3P_VOTk6CnfdStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk6CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB3P_VOTk6PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB3P_VOTk6PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB3P_VOTk6PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB3P_VOTk6PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB3P_VOTk6PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB3P_VOTk6PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB3P_VOTk6PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB3P_VOTk6PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 7
            if decoded["SRRLFODPB3P_VOTk7MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB3P_VOTk7IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB3P_VOTk7MStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk7MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB3P_VOTk7CnfdStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk7CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB3P_VOTk7PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB3P_VOTk7PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB3P_VOTk7PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB3P_VOTk7PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB3P_VOTk7PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB3P_VOTk7PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB3P_VOTk7PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB3P_VOTk7PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'f5': # Consec Frame 3
            # Pedestrian Track 5
            if decoded["SRRLFODPB4P_POTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB4P_POTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB4P_POTk5MStsAuth"])
                #print(decoded["SRRLFODPB4P_POTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB4P_POTk5CnfdStsAuth"])
                #print(decoded["SRRLFODPB4P_POTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB4P_POTk5PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB4P_POTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB4P_POTk5PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB4P_POTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 8
            if decoded["SRRLFODPB4P_VOTk8MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB4P_VOTk8IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB4P_VOTk8MStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk8MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB4P_VOTk8CnfdStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk8CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB4P_VOTk8PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB4P_VOTk8PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB4P_VOTk8PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB4P_VOTk8PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB4P_VOTk8PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB4P_VOTk8PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["SRRLFODPB4P_VOTk8PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB4P_VOTk8PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)
            
            # Vehicle Track 9
            if decoded["SRRLFODPB4P_VOTk9MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB4P_VOTk9IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB4P_VOTk9MStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk9MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB4P_VOTk9CnfdStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk9CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB4P_VOTk9PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB4P_VOTk9PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB4P_VOTk9PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB4P_VOTk9PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB4P_VOTk9PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB4P_VOTk9PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB4P_VOTk9PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB4P_VOTk9PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 10
            if decoded["SRRLFODPB4P_VOTk10MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB4P_VOTk10IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB4P_VOTk10MStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk10MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB4P_VOTk10CnfdStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk10CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB4P_VOTk10PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB4P_VOTk10PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB4P_VOTk10PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB4P_VOTk10PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB4P_VOTk10PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB4P_VOTk10PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB4P_VOTk10PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB4P_VOTk10PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'd2': # Consec Frame 4
            pass # Currently provides no data of interest
        case 'f6': 
            pass # Currently Provides no Data of interest
        case _: # Default case
            node.get_logger().info('Unrecognized Frame ID in SRRLF conversion: %s' % frame_id)

    return msgs

def lyriq_24_srrrf(node, frame_id, data):
    msgs = []
    if type(data) is not str:
        binary_data = data
    else:
        binary_data = bytes.fromhex(data)
    decoded = node.db_can8.decode_message(int(frame_id, 16), binary_data)
    match frame_id:
        case 'd1': # First Frame
            # Pedestrian Track 1
            if decoded["SRRRFODPB1P_POTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB1P_POTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB1P_POTk1MStsAuth"])
                #print(decoded["SRRRFODPB1P_POTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB1P_POTk1CnfdStsAuth"])
                #print(decoded["SRRRFODPB1P_POTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB1P_POTk1PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB1P_POTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB1P_POTk1PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB1P_POTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Pedestrian Track 2
            if decoded["SRRRFODPB1P_POTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB1P_POTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB1P_POTk2MStsAuth"])
                #print(decoded["SRRRFODPB1P_POTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB1P_POTk2CnfdStsAuth"])
                #print(decoded["SRRRFODPB1P_POTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB1P_POTk2PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB1P_POTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB1P_POTk2PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB1P_POTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 1
            if decoded["SRRRFODPB1P_VOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB1P_VOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB1P_VOTk1MStsAuth"])
                #print(decoded["SRRRFODPB1P_VOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB1P_VOTk1CnfdStsAuth"])
                #print(decoded["SRRRFODPB1P_VOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB1P_VOTk1PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB1P_VOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB1P_VOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB1P_VOTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB1P_VOTk1PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB1P_VOTk1PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB1P_VOTk1PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB1P_VOTk1PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'f7': # Consec Frame 1
            # Pedestrian Track 3
            if decoded["SRRRFODPB2P_POTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB2P_POTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB2P_POTk3MStsAuth"])
                #print(decoded["SRRRFODPB2P_POTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB2P_POTk3CnfdStsAuth"])
                #print(decoded["SRRRFODPB2P_POTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB2P_POTk3PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB2P_POTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB2P_POTk3PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB2P_POTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 2
            if decoded["SRRRFODPB2P_VOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB2P_VOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB2P_VOTk2MStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB2P_VOTk2CnfdStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB2P_VOTk2PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB2P_VOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB2P_VOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB2P_VOTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["SRRRFODPB2P_VOTk2PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB2P_VOTk2PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB2P_VOTk2PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB2P_VOTk2PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 3
            if decoded["SRRRFODPB2P_VOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB2P_VOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB2P_VOTk3MStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB2P_VOTk3CnfdStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB2P_VOTk3PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB2P_VOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB2P_VOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB2P_VOTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB2P_VOTk3PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB2P_VOTk3PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB2P_VOTk3PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB2P_VOTk3PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 4
            if decoded["SRRRFODPB2P_VOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB2P_VOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB2P_VOTk4MStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB2P_VOTk4CnfdStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB2P_VOTk4PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB2P_VOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB2P_VOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB2P_VOTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB2P_VOTk4PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB2P_VOTk4PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB2P_VOTk4PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB2P_VOTk4PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'd0': # Consec Frame 2
            # Pedestrian Track 4
            if decoded["SRRRFODPB3P_POTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB3P_POTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB3P_POTk4MStsAuth"])
                #print(decoded["SRRRFODPB3P_POTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB3P_POTk4CnfdStsAuth"])
                #print(decoded["SRRRFODPB3P_POTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB3P_POTk4PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB3P_POTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB3P_POTk4PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB3P_POTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 5
            if decoded["SRRRFODPB3P_VOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB3P_VOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB3P_VOTk5MStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB3P_VOTk5CnfdStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB3P_VOTk5PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB3P_VOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB3P_VOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB3P_VOTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB3P_VOTk5PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB3P_VOTk5PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB3P_VOTk5PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB3P_VOTk5PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 6
            if decoded["SRRRFODPB3P_VOTk6MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB3P_VOTk6IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB3P_VOTk6MStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk6MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB3P_VOTk6CnfdStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk6CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB3P_VOTk6PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB3P_VOTk6PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB3P_VOTk6PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB3P_VOTk6PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB3P_VOTk6PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB3P_VOTk6PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB3P_VOTk6PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB3P_VOTk6PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 7
            if decoded["SRRRFODPB3P_VOTk7MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB3P_VOTk7IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB3P_VOTk7MStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk7MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB3P_VOTk7CnfdStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk7CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB3P_VOTk7PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB3P_VOTk7PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB3P_VOTk7PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB3P_VOTk7PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["SRRRFODPB3P_VOTk7PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB3P_VOTk7PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB3P_VOTk7PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB3P_VOTk7PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'f8': # Consec Frame 3
            # Pedestrian Track 5
            if decoded["SRRRFODPB4P_POTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB4P_POTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB4P_POTk5MStsAuth"])
                #print(decoded["SRRRFODPB4P_POTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB4P_POTk5CnfdStsAuth"])
                #print(decoded["SRRRFODPB4P_POTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB4P_POTk5PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB4P_POTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB4P_POTk5PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB4P_POTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 8
            if decoded["SRRRFODPB4P_VOTk8MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB4P_VOTk8IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB4P_VOTk8MStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk8MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB4P_VOTk8CnfdStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk8CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB4P_VOTk8PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB4P_VOTk8PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB4P_VOTk8PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB4P_VOTk8PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB4P_VOTk8PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB4P_VOTk8PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB4P_VOTk8PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB4P_VOTk8PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 9
            if decoded["SRRRFODPB4P_VOTk9MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB4P_VOTk9IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB4P_VOTk9MStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk9MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB4P_VOTk9CnfdStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk9CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB4P_VOTk9PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB4P_VOTk9PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB4P_VOTk9PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB4P_VOTk9PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB4P_VOTk9PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB4P_VOTk9PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["SRRRFODPB4P_VOTk9PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB4P_VOTk9PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 10
            if decoded["SRRRFODPB4P_VOTk10MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB4P_VOTk10IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB4P_VOTk10MStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk10MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB4P_VOTk10CnfdStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk10CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB4P_VOTk10PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB4P_VOTk10PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB4P_VOTk10PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB4P_VOTk10PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB4P_VOTk10PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB4P_VOTk10PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB4P_VOTk10PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB4P_VOTk10PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)

        case 'cf': # Consec Frame 4
            pass # Currently provides no data of interest

        case 'f9': 
            pass # Currently Provides no Data of interest

        case _: # Default case
            node.get_logger().info('Unrecognized Frame ID in SRRRF conversion: %s' % frame_id)

    return msgs

def lyriq_24_lrr(node, frame_id, data):
    msgs = []
    if type(data) is not str:
        binary_data = data
    else:
        binary_data = bytes.fromhex(data)
    decoded = node.db_can8.decode_message(int(frame_id, 16), binary_data)
    match frame_id:
        case 'd7': # First Frame
            # Pedestrian Track 1
            if decoded["LRRODPB1P_POTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB1P_POTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB1P_POTk1MStsAuth"])
                #print(decoded["LRRODPB1P_POTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB1P_POTk1CnfdStsAuth"])
                #print(decoded["LRRODPB1P_POTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB1P_POTk1PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB1P_POTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB1P_POTk1PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB1P_POTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 1
            if decoded["LRRODPB1P_VOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB1P_VOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB1P_VOTk1MStsAuth"])
                #print(decoded["LRRODPB1P_VOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB1P_VOTk1CnfdStsAuth"])
                #print(decoded["LRRODPB1P_VOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB1P_VOTk1PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB1P_VOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB1P_VOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB1P_VOTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB1P_VOTk1PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB1P_VOTk1PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB1P_VOTk1PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB1P_VOTk1PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 2
            if decoded["LRRODPB1P_VOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB1P_VOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB1P_VOTk2MStsAuth"])
                #print(decoded["LRRODPB1P_VOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB1P_VOTk2CnfdStsAuth"])
                #print(decoded["LRRODPB1P_VOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB1P_VOTk2PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB1P_VOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB1P_VOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB1P_VOTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB1P_VOTk2PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB1P_VOTk2PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB1P_VOTk2PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB1P_VOTk2PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'f1': # Consec Frame 1
            # Pedestrian Track 2
            if decoded["LRRODPB2P_POTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB2P_POTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB2P_POTk2MStsAuth"])
                #print(decoded["LRRODPB2P_POTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB2P_POTk2CnfdStsAuth"])
                #print(decoded["LRRODPB2P_POTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB2P_POTk2PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB2P_POTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB2P_POTk2PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB2P_POTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 3
            if decoded["LRRODPB2P_VOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB2P_VOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB2P_VOTk3MStsAuth"])
                #print(decoded["LRRODPB2P_VOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB2P_VOTk3CnfdStsAuth"])
                #print(decoded["LRRODPB2P_VOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB2P_VOTk3PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB2P_VOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB2P_VOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB2P_VOTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["LRRODPB2P_VOTk3PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB2P_VOTk3PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB2P_VOTk3PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB2P_VOTk3PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 4
            if decoded["LRRODPB2P_VOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB2P_VOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB2P_VOTk4MStsAuth"])
                #print(decoded["LRRODPB2P_VOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB2P_VOTk4CnfdStsAuth"])
                #print(decoded["LRRODPB2P_VOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB2P_VOTk4PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB2P_VOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB2P_VOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB2P_VOTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["LRRODPB2P_VOTk4PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB2P_VOTk4PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB2P_VOTk4PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB2P_VOTk4PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 5
            if decoded["LRRODPB2P_VOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB2P_VOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB2P_VOTk5MStsAuth"])
                #print(decoded["LRRODPB2P_VOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB2P_VOTk5CnfdStsAuth"])
                #print(decoded["LRRODPB2P_VOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB2P_VOTk5PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB2P_VOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB2P_VOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB2P_VOTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB2P_VOTk5PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB2P_VOTk5PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB2P_VOTk5PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB2P_VOTk5PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'd6': # Consec Frame 2
            # Pedestrian Track 3
            if decoded["LRRODPB3P_POTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB3P_POTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB3P_POTk3MStsAuth"])
                #print(decoded["LRRODPB3P_POTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB3P_POTk3CnfdStsAuth"])
                #print(decoded["LRRODPB3P_POTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB3P_POTk3PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB3P_POTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB3P_POTk3PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB3P_POTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 6
            if decoded["LRRODPB3P_VOTk6MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB3P_VOTk6IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB3P_VOTk6MStsAuth"])
                #print(decoded["LRRODPB3P_VOTk6MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB3P_VOTk6CnfdStsAuth"])
                #print(decoded["LRRODPB3P_VOTk6CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB3P_VOTk6PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB3P_VOTk6PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB3P_VOTk6PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB3P_VOTk6PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB3P_VOTk6PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB3P_VOTk6PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB3P_VOTk6PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB3P_VOTk6PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 7
            if decoded["LRRODPB3P_VOTk7MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB3P_VOTk7IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB3P_VOTk7MStsAuth"])
                #print(decoded["LRRODPB3P_VOTk7MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB3P_VOTk7CnfdStsAuth"])
                #print(decoded["LRRODPB3P_VOTk7CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB3P_VOTk7PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB3P_VOTk7PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB3P_VOTk7PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB3P_VOTk7PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB3P_VOTk7PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB3P_VOTk7PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB3P_VOTk7PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB3P_VOTk7PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 8
            if decoded["LRRODPB3P_VOTk8MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB3P_VOTk8IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB3P_VOTk8MStsAuth"])
                #print(decoded["LRRODPB3P_VOTk8MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB3P_VOTk8CnfdStsAuth"])
                #print(decoded["LRRODPB3P_VOTk8CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB3P_VOTk8PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB3P_VOTk8PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB3P_VOTk8PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB3P_VOTk8PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB3P_VOTk8PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB3P_VOTk8PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB3P_VOTk8PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB3P_VOTk8PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
        case 'f2': # Consec Frame 3
            # Pedestrian Track 4
            if decoded["LRRODPB4P_POTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB4P_POTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB4P_POTk4MStsAuth"])
                #print(decoded["LRRODPB4P_POTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB4P_POTk4CnfdStsAuth"])
                #print(decoded["LRRODPB4P_POTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB4P_POTk4PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB4P_POTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB4P_POTk4PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB4P_POTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Pedestrian Track 5
            if decoded["LRRODPB4P_POTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB4P_POTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB4P_POTk5MStsAuth"])
                #print(decoded["LRRODPB4P_POTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB4P_POTk5CnfdStsAuth"])
                #print(decoded["LRRODPB4P_POTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB4P_POTk5PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB4P_POTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB4P_POTk5PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB4P_POTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            # Vehicle Track 9
            if decoded["LRRODPB4P_VOTk9MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB4P_VOTk9IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB4P_VOTk9MStsAuth"])
                #print(decoded["LRRODPB4P_VOTk9MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB4P_VOTk9CnfdStsAuth"])
                #print(decoded["LRRODPB4P_VOTk9CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB4P_VOTk9PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB4P_VOTk9PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB4P_VOTk9PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB4P_VOTk9PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB4P_VOTk9PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB4P_VOTk9PoILgPsCAuth"] #lon cov
                new_m1.covariance[10] = decoded["LRRODPB4P_VOTk9PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB4P_VOTk9PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 10
            if decoded["LRRODPB4P_VOTk10MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB4P_VOTk10IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB4P_VOTk10MStsAuth"])
                #print(decoded["LRRODPB4P_VOTk10MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB4P_VOTk10CnfdStsAuth"])
                #print(decoded["LRRODPB4P_VOTk10CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB4P_VOTk10PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB4P_VOTk10PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB4P_VOTk10PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB4P_VOTk10PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB4P_VOTk10PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB4P_VOTk10PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB4P_VOTk10PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB4P_VOTk10PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
        case 'd5': # Consec Frame 4
            pass # Currently provides no data of interest
        case 'f3': 
            pass # Currently Provides no Data of interest
        case _: # Default case
            node.get_logger().info('Unrecognized Frame ID in LRR conversion: %s' % frame_id)

    return msgs



def can_to_message_22Escalade_8(node, frame_id, data):
    # Use string representation here lower case
    msgs = []
    if frame_id in ['8e','8f','90','91','92','93','94','95','e1','e2','e3','ec','ed','ee','ef']: #FCM Messages
        for m in my_22_fcm(node, frame_id, data):
            msgs.append(m)

    elif frame_id in ['aa','ab','ac','ad','ae','af']: #SSRRF Messages
        for m in my_22_srrf(node, frame_id, data):

            msgs.append(m)

    elif frame_id in ['9e','9f','a0','a1','a2','a3']: # SSRLF Messages
        for m in my_22_srrlf(node, frame_id, data):

            msgs.append(m)

    elif frame_id in ['98','99','9a','9b','9c','9d']: #LRR Messages
        for m in my_22_lrr(node, frame_id, data):

            msgs.append(m)

    
    if VISUALIZE:
        visualize(node, msgs)

    
    
    for m in msgs:
        if USE_CALIBRATED_VALUES and not m.object_type:
            match m.sensor_id:
                case 1: # LRR
    
                    #m.lat -= (-0.08703757198522322 * m.lat + 0.4975083707270722)
                    #m.covariance[0] = 1.8251249364952613 ** 2

                    m.lat -= -0.32363373398958334
                    m.covariance[0] = 1.9087416501404038 ** 2

                    m.covariance[5] = 1.6729411928822384 ** 2
                    m.lon -= -3.775156893462339

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    # Made this stat worse
                    #m.lat_vel -= 0.9417774655697115
                    m.covariance[10] = 2.034917289810119 ** 2
                    m.lat_vel -= 0.9417774655697115

                    m.covariance[15] = 1.3588741711631098 ** 2
                    m.lon_vel -= 0.005144692112980788
                    #m.covariance[15] = 1.184748728520708 ** 2
                    # m.lon_vel -= (0.2530452180355196 * m.lon_vel + 0.03411408759271623)

                case 2: # FCM
                    # No correlation for lat as such we will use the std of the error for the variance
                    # m.covariance[0] = 0.30008838371420826 ** 2
                    # m.lat -= (-0.0398772958817702 * m.lat - 1.1794496291252585)
                    m.lat -= 1.3290568493873873
                    m.covariance[0] = 0.3163984057053055 ** 2

                    m.lon -= (-0.06466239597748658 * m.lon - 6.284011764072128)
                    m.covariance[5] =  1.177921184582073 ** 2

                    m.covariance[10] = 0.7309356799685403 ** 2
                    m.lat_vel -= 0.30794534437837845
                    #m.covariance[10] = 0.7102863492151918 ** 2
                    #m.lat_vel -= (-0.21224574323672737 * m.lat_vel + 0.32216676683938134)

                    m.covariance[15] = 1.3505901864595427
                    m.lon_vel -= -1.042705606418919
                    #m.covariance[15] = 1.283579459334189 ** 2
                    # m.lon_vel -= (0.1504358365070556 * m.lon_vel - 1.4505595212837212)
                    
                case 3: # L-SRR

                    if abs(m.lat) < -63.5:
                        m.covariance[0] = 10.0
                    else:
                        #m.lat -= (-0.024900632558547273 *m.lat + 0.6936446611670223)
                        #m.covariance[0] = 1.993318703135391 ** 2
                        m.lat -= -0.8299548149327112
                        m.covariance[0] = 1.953521232599644

                    if m.lon > 0:
                        m.lon -= (-1.8530872574246962 * np.log(m.lon) + 1.1145362826806227)
                    m.covariance[5] = 1.5498820773292588 ** 2

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.lat_vel -= -0.11048249013163067
                    m.covariance[10] = 0.7755070400470802 ** 2

                    # No benefit from function statistically
                    m.covariance[15] = 0.5812925798843384 ** 2
                    m.lon_vel -= 0.04905138971365421

                case 4: # R-SRR
                    
                    m.covariance[5] = 1.1248463656192702 ** 2
                    if m.lon > 0:
                        m.lon -= (-2.2512233591184434 * np.log(m.lon) + 2.68004230709231)

                    if abs(m.lat) > 63.5:
                        m.covariance[0] = 10.0
                    else:
                        m.lat -= -0.7695654564600879
                        m.covariance[0] = 2.33447700265333 ** 2

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.lat_vel -= -0.05762124828535505
                    m.covariance[10] = 1.070712387115616 ** 2

                    m.covariance[15] = 0.6851340293004434 ** 2
                    m.lon_vel -= (0.13902626632661755 * m.lon_vel - 0.11158128896313226)
    
    
    for m in msgs:
        if USE_CALIBRATED_VALUES and m.object_type:
            match m.sensor_id:
                case 1: # LRR
    
                    # No Corelation for lat as such we will use the std of the error for the variance
                    m.covariance[0] = 0.741373398823215 ** 2

                    m.covariance[5] = 0.5155798245603188 ** 2

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.covariance[10] = 0.6156007895391087 ** 2

                    m.covariance[15] = 0.3998980969840088 ** 2
                    m.lon_vel -= 0.5331162896980735 * m.lon_vel + 0.0016021153662147958

                case 2: # FCM

                    # No correlation for lat as such we will use the std of the error for the variance
                    m.covariance[0] = 0.3360611561602104 ** 2
                    m.lat -= -0.05511392405998433 * m.lat + 0.158242581509988

                    m.lon -= -0.037126224018847 * m.lon - 0.6487406308105296
                    m.covariance[5] =  0.5761297198557471 ** 2

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.covariance[10] = 0.29635838754286353** 2
                    m.lat_vel -= -0.15144613505386095 * m.lat_vel - 0.005916756964720937

                    m.covariance[15] = 0.26083037099040784 ** 2
                    m.lon_vel -= 0.74156530010494 * m.lon_vel + 0.029707974563131078

                    
                case 3: # L-SRR

                    m.lat -= 0.032453609487344326 * m.lat + 0.1813722942774989
                    m.covariance[0] =  0.24665782198873903 ** 2
                    
                    m.covariance[5] = 0.16657156083818409 ** 2
                    m.lon -= 0.01198146174114297 * m.lon + 0.38279159965839055

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.covariance[10] = 0.34485247761995763 ** 2
                    m.lat_vel -= 0.09395554000389372 * m.lat_vel + 0.011905739067240018

                    # No benefit from function statistically
                    m.covariance[15] = 0.3575099672493341 ** 2

                case 4: # R-SRR

                    m.lat -= 0.016991930539205925 * m.lat + 0.10062339431619796
                    m.covariance[0] = 0.529853441480699 ** 2

                    m.covariance[5] = 0.20755511960030246 **2
                    m.lon -= 0.014883478277086248 * m.lon + 0.3126154624655155

                    # No correlation for lat_vel as such we will use the std of the error for the variance
                    m.covariance[10] = 0.35316016460306093 ** 2
                    m.lat_vel -= 0.06663743261649147 * m.lat_vel - 0.021409390371740278

                    m.covariance[15] = 0.3286495839533845 ** 2
    
    
    return msgs

def my_22_lrr(node, frame_id, data):
    msgs = []
    if type(data) is not str:
        binary_data = data
    else:
        binary_data = bytes.fromhex(data)
    decoded = node.db_can8.decode_message(int(frame_id, 16), binary_data)
    match frame_id:
        case '98': # First Frame
            # Pedestrian Track 1
            if decoded["LRRODPB1P_POTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB1P_POTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB1P_POTk1MStsAuth"])
                #print(decoded["LRRODPB1P_POTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB1P_POTk1CnfdStsAuth"])
                #print(decoded["LRRODPB1P_POTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB1P_POTk1PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB1P_POTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB1P_POTk1PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB1P_POTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 1
            if decoded["LRRODPB1P_VOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB1P_VOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB1P_VOTk1MStsAuth"])
                #print(decoded["LRRODPB1P_VOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB1P_VOTk1CnfdStsAuth"])
                #print(decoded["LRRODPB1P_VOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB1P_VOTk1PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB1P_VOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB1P_VOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB1P_VOTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB1P_VOTk1PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB1P_VOTk1PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB1P_VOTk1PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB1P_VOTk1PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 2
            if decoded["LRRODPB1P_VOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB1P_VOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB1P_VOTk2MStsAuth"])
                #print(decoded["LRRODPB1P_VOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB1P_VOTk2CnfdStsAuth"])
                #print(decoded["LRRODPB1P_VOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB1P_VOTk2PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB1P_VOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB1P_VOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB1P_VOTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB1P_VOTk2PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB1P_VOTk2PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB1P_VOTk2PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB1P_VOTk2PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case '99': # Consec Frame 1
            # Pedestrian Track 2
            if decoded["LRRODPB2P_POTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB2P_POTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB2P_POTk2MStsAuth"])
                #print(decoded["LRRODPB2P_POTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB2P_POTk2CnfdStsAuth"])
                #print(decoded["LRRODPB2P_POTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB2P_POTk2PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB2P_POTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB2P_POTk2PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB2P_POTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 3
            if decoded["LRRODPB2P_VOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB2P_VOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB2P_VOTk3MStsAuth"])
                #print(decoded["LRRODPB2P_VOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB2P_VOTk3CnfdStsAuth"])
                #print(decoded["LRRODPB2P_VOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB2P_VOTk3PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB2P_VOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB2P_VOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB2P_VOTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["LRRODPB2P_VOTk3PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB2P_VOTk3PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB2P_VOTk3PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB2P_VOTk3PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 4
            if decoded["LRRODPB2P_VOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB2P_VOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB2P_VOTk4MStsAuth"])
                #print(decoded["LRRODPB2P_VOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB2P_VOTk4CnfdStsAuth"])
                #print(decoded["LRRODPB2P_VOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB2P_VOTk4PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB2P_VOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB2P_VOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB2P_VOTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["LRRODPB2P_VOTk4PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB2P_VOTk4PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB2P_VOTk4PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB2P_VOTk4PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 5
            if decoded["LRRODPB2P_VOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB2P_VOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB2P_VOTk5MStsAuth"])
                #print(decoded["LRRODPB2P_VOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB2P_VOTk5CnfdStsAuth"])
                #print(decoded["LRRODPB2P_VOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB2P_VOTk5PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB2P_VOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB2P_VOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB2P_VOTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB2P_VOTk5PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB2P_VOTk5PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB2P_VOTk5PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB2P_VOTk5PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case '9a': # Consec Frame 2
            # Pedestrian Track 3
            if decoded["LRRODPB3P_POTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB3P_POTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB3P_POTk3MStsAuth"])
                #print(decoded["LRRODPB3P_POTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB3P_POTk3CnfdStsAuth"])
                #print(decoded["LRRODPB3P_POTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB3P_POTk3PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB3P_POTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB3P_POTk3PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB3P_POTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 6
            if decoded["LRRODPB3P_VOTk6MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB3P_VOTk6IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB3P_VOTk6MStsAuth"])
                #print(decoded["LRRODPB3P_VOTk6MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB3P_VOTk6CnfdStsAuth"])
                #print(decoded["LRRODPB3P_VOTk6CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB3P_VOTk6PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB3P_VOTk6PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB3P_VOTk6PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB3P_VOTk6PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB3P_VOTk6PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB3P_VOTk6PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB3P_VOTk6PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB3P_VOTk6PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 7
            if decoded["LRRODPB3P_VOTk7MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB3P_VOTk7IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB3P_VOTk7MStsAuth"])
                #print(decoded["LRRODPB3P_VOTk7MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB3P_VOTk7CnfdStsAuth"])
                #print(decoded["LRRODPB3P_VOTk7CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB3P_VOTk7PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB3P_VOTk7PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB3P_VOTk7PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB3P_VOTk7PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB3P_VOTk7PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB3P_VOTk7PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB3P_VOTk7PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB3P_VOTk7PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 8
            if decoded["LRRODPB3P_VOTk8MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB3P_VOTk8IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB3P_VOTk8MStsAuth"])
                #print(decoded["LRRODPB3P_VOTk8MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB3P_VOTk8CnfdStsAuth"])
                #print(decoded["LRRODPB3P_VOTk8CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB3P_VOTk8PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB3P_VOTk8PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB3P_VOTk8PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB3P_VOTk8PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB3P_VOTk8PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB3P_VOTk8PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB3P_VOTk8PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB3P_VOTk8PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
        case '9b': # Consec Frame 3
            # Pedestrian Track 4
            if decoded["LRRODPB4P_POTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB4P_POTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB4P_POTk4MStsAuth"])
                #print(decoded["LRRODPB4P_POTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB4P_POTk4CnfdStsAuth"])
                #print(decoded["LRRODPB4P_POTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB4P_POTk4PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB4P_POTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB4P_POTk4PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB4P_POTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Pedestrian Track 5
            if decoded["LRRODPB4P_POTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB4P_POTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB4P_POTk5MStsAuth"])
                #print(decoded["LRRODPB4P_POTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB4P_POTk5CnfdStsAuth"])
                #print(decoded["LRRODPB4P_POTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["LRRODPB4P_POTk5PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB4P_POTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB4P_POTk5PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB4P_POTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            # Vehicle Track 9
            if decoded["LRRODPB4P_VOTk9MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB4P_VOTk9IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB4P_VOTk9MStsAuth"])
                #print(decoded["LRRODPB4P_VOTk9MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB4P_VOTk9CnfdStsAuth"])
                #print(decoded["LRRODPB4P_VOTk9CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB4P_VOTk9PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB4P_VOTk9PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB4P_VOTk9PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB4P_VOTk9PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB4P_VOTk9PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB4P_VOTk9PoILgPsCAuth"] #lon cov
                new_m1.covariance[10] = decoded["LRRODPB4P_VOTk9PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB4P_VOTk9PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 10
            if decoded["LRRODPB4P_VOTk10MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 1
                new_m1.track_id = decoded['LRRODPB4P_VOTk10IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["LRRODPB4P_VOTk10MStsAuth"])
                #print(decoded["LRRODPB4P_VOTk10MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["LRRODPB4P_VOTk10CnfdStsAuth"])
                #print(decoded["LRRODPB4P_VOTk10CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["LRRODPB4P_VOTk10PoILPsAuth"]
                new_m1.lon = decoded["LRRODPB4P_VOTk10PoILgPsAuth"]
                new_m1.lat_vel = decoded["LRRODPB4P_VOTk10PoILVAuth"]
                new_m1.lon_vel = decoded["LRRODPB4P_VOTk10PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by LRR
                 # Not provided by LRR

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["LRRODPB4P_VOTk10PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["LRRODPB4P_VOTk10PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["LRRODPB4P_VOTk10PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["LRRODPB4P_VOTk10PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
        case '9c': # Consec Frame 4
            pass # Currently provides no data of interest
        case '9d': 
            pass # Currently Provides no Data of interest
        case _: # Default case
            node.get_logger().info('Unrecognized Frame ID in LRR conversion: %s' % frame_id)

    return msgs

def my_22_srrlf(node, frame_id, data):
    msgs = []
    if type(data) is not str:
        binary_data = data
    else:
        binary_data = bytes.fromhex(data)
    decoded = node.db_can8.decode_message(int(frame_id, 16), binary_data)
    match frame_id:
        case '9e': # First Frame
            # Pedestrian Track 1
            if decoded["SRRLFODPB1P_POTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB1P_POTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB1P_POTk1MStsAuth"])
                #print(decoded["SRRLFODPB1P_POTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB1P_POTk1CnfdStsAuth"])
                #print(decoded["SRRLFODPB1P_POTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB1P_POTk1PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB1P_POTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB1P_POTk1PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB1P_POTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Pedestrian Track 2
            if decoded["SRRLFODPB1P_POTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB1P_POTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB1P_POTk2MStsAuth"])
                #print(decoded["SRRLFODPB1P_POTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB1P_POTk2CnfdStsAuth"])
                #print(decoded["SRRLFODPB1P_POTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB1P_POTk2PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB1P_POTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB1P_POTk2PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB1P_POTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 1
            if decoded["SRRLFODPB1P_VOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB1P_VOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB1P_VOTk1MStsAuth"])
                #print(decoded["SRRLFODPB1P_VOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB1P_VOTk1CnfdStsAuth"])
                #print(decoded["SRRLFODPB1P_VOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB1P_VOTk1PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB1P_VOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB1P_VOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB1P_VOTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB1P_VOTk1PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB1P_VOTk1PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["SRRLFODPB1P_VOTk1PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB1P_VOTk1PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)

        case '9f': # Consec Frame 1
            # Pedestrian Track 3
            if decoded["SRRLFODPB2P_POTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB2P_POTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB2P_POTk3MStsAuth"])
                #print(decoded["SRRLFODPB2P_POTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB2P_POTk3CnfdStsAuth"])
                #print(decoded["SRRLFODPB2P_POTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB2P_POTk3PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB2P_POTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB2P_POTk3PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB2P_POTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 2
            if decoded["SRRLFODPB2P_VOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB2P_VOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB2P_VOTk2MStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB2P_VOTk2CnfdStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB2P_VOTk2PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB2P_VOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB2P_VOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB2P_VOTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB2P_VOTk2PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB2P_VOTk2PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["SRRLFODPB2P_VOTk2PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB2P_VOTk2PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)
            
            # Vehicle Track 3
            if decoded["SRRLFODPB2P_VOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB2P_VOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB2P_VOTk3MStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB2P_VOTk3CnfdStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB2P_VOTk3PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB2P_VOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB2P_VOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB2P_VOTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB2P_VOTk3PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB2P_VOTk3PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB2P_VOTk3PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB2P_VOTk3PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 4
            if decoded["SRRLFODPB2P_VOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB2P_VOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB2P_VOTk4MStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB2P_VOTk4CnfdStsAuth"])
                #print(decoded["SRRLFODPB2P_VOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB2P_VOTk4PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB2P_VOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB2P_VOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB2P_VOTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB2P_VOTk4PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB2P_VOTk4PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB2P_VOTk4PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB2P_VOTk4PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'a0': # Consec Frame 2
            # Pedestrian Track 4
            if decoded["SRRLFODPB3P_POTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB3P_POTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB3P_POTk4MStsAuth"])
                #print(decoded["SRRLFODPB3P_POTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB3P_POTk4CnfdStsAuth"])
                #print(decoded["SRRLFODPB3P_POTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB3P_POTk4PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB3P_POTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB3P_POTk4PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB3P_POTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 5
            if decoded["SRRLFODPB3P_VOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB3P_VOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB3P_VOTk5MStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB3P_VOTk5CnfdStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB3P_VOTk5PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB3P_VOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB3P_VOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB3P_VOTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB3P_VOTk5PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB3P_VOTk5PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB3P_VOTk5PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB3P_VOTk5PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 6
            if decoded["SRRLFODPB3P_VOTk6MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB3P_VOTk6IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB3P_VOTk6MStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk6MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB3P_VOTk6CnfdStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk6CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB3P_VOTk6PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB3P_VOTk6PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB3P_VOTk6PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB3P_VOTk6PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB3P_VOTk6PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB3P_VOTk6PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB3P_VOTk6PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB3P_VOTk6PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 7
            if decoded["SRRLFODPB3P_VOTk7MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB3P_VOTk7IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB3P_VOTk7MStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk7MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB3P_VOTk7CnfdStsAuth"])
                #print(decoded["SRRLFODPB3P_VOTk7CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB3P_VOTk7PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB3P_VOTk7PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB3P_VOTk7PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB3P_VOTk7PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB3P_VOTk7PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB3P_VOTk7PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB3P_VOTk7PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB3P_VOTk7PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'a1': # Consec Frame 3
            # Pedestrian Track 5
            if decoded["SRRLFODPB4P_POTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB4P_POTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB4P_POTk5MStsAuth"])
                #print(decoded["SRRLFODPB4P_POTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB4P_POTk5CnfdStsAuth"])
                #print(decoded["SRRLFODPB4P_POTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRLFODPB4P_POTk5PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB4P_POTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB4P_POTk5PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB4P_POTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 8
            if decoded["SRRLFODPB4P_VOTk8MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB4P_VOTk8IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB4P_VOTk8MStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk8MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB4P_VOTk8CnfdStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk8CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB4P_VOTk8PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB4P_VOTk8PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB4P_VOTk8PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB4P_VOTk8PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB4P_VOTk8PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB4P_VOTk8PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["SRRLFODPB4P_VOTk8PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB4P_VOTk8PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)
            
            # Vehicle Track 9
            if decoded["SRRLFODPB4P_VOTk9MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB4P_VOTk9IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB4P_VOTk9MStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk9MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB4P_VOTk9CnfdStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk9CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB4P_VOTk9PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB4P_VOTk9PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB4P_VOTk9PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB4P_VOTk9PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB4P_VOTk9PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB4P_VOTk9PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB4P_VOTk9PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB4P_VOTk9PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 10
            if decoded["SRRLFODPB4P_VOTk10MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 3
                new_m1.track_id = decoded['SRRLFODPB4P_VOTk10IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRLFODPB4P_VOTk10MStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk10MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRLFODPB4P_VOTk10CnfdStsAuth"])
                #print(decoded["SRRLFODPB4P_VOTk10CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRLFODPB4P_VOTk10PoILPsAuth"]
                new_m1.lon = decoded["SRRLFODPB4P_VOTk10PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRLFODPB4P_VOTk10PoILVAuth"]
                new_m1.lon_vel = decoded["SRRLFODPB4P_VOTk10PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRLF
                 # Not provided by SRRLF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRLFODPB4P_VOTk10PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRLFODPB4P_VOTk10PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRLFODPB4P_VOTk10PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRLFODPB4P_VOTk10PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'a2': # Consec Frame 4
            pass # Currently provides no data of interest
        case 'a3': 
            pass # Currently Provides no Data of interest
        case _: # Default case
            node.get_logger().info('Unrecognized Frame ID in SRRLF conversion: %s' % frame_id)


    return msgs

def my_22_srrf(node, frame_id, data):
    msgs = []
    if type(data) is not str:
        binary_data = data
    else:
        binary_data = bytes.fromhex(data)
    decoded = node.db_can8.decode_message(int(frame_id, 16), binary_data)
    match frame_id:
        case 'aa': # First Frame
            # Pedestrian Track 1
            if decoded["SRRRFODPB1P_POTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB1P_POTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB1P_POTk1MStsAuth"])
                #print(decoded["SRRRFODPB1P_POTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB1P_POTk1CnfdStsAuth"])
                #print(decoded["SRRRFODPB1P_POTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB1P_POTk1PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB1P_POTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB1P_POTk1PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB1P_POTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Pedestrian Track 2
            if decoded["SRRRFODPB1P_POTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB1P_POTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB1P_POTk2MStsAuth"])
                #print(decoded["SRRRFODPB1P_POTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB1P_POTk2CnfdStsAuth"])
                #print(decoded["SRRRFODPB1P_POTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB1P_POTk2PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB1P_POTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB1P_POTk2PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB1P_POTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 1
            if decoded["SRRRFODPB1P_VOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB1P_VOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB1P_VOTk1MStsAuth"])
                #print(decoded["SRRRFODPB1P_VOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB1P_VOTk1CnfdStsAuth"])
                #print(decoded["SRRRFODPB1P_VOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB1P_VOTk1PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB1P_VOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB1P_VOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB1P_VOTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB1P_VOTk1PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB1P_VOTk1PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB1P_VOTk1PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB1P_VOTk1PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'ab': # Consec Frame 1
            # Pedestrian Track 3
            if decoded["SRRRFODPB2P_POTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB2P_POTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB2P_POTk3MStsAuth"])
                #print(decoded["SRRRFODPB2P_POTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB2P_POTk3CnfdStsAuth"])
                #print(decoded["SRRRFODPB2P_POTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB2P_POTk3PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB2P_POTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB2P_POTk3PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB2P_POTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 2
            if decoded["SRRRFODPB2P_VOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB2P_VOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB2P_VOTk2MStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB2P_VOTk2CnfdStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB2P_VOTk2PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB2P_VOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB2P_VOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB2P_VOTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["SRRRFODPB2P_VOTk2PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB2P_VOTk2PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB2P_VOTk2PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB2P_VOTk2PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 3
            if decoded["SRRRFODPB2P_VOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB2P_VOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB2P_VOTk3MStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB2P_VOTk3CnfdStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB2P_VOTk3PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB2P_VOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB2P_VOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB2P_VOTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB2P_VOTk3PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB2P_VOTk3PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB2P_VOTk3PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB2P_VOTk3PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 4
            if decoded["SRRRFODPB2P_VOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB2P_VOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB2P_VOTk4MStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB2P_VOTk4CnfdStsAuth"])
                #print(decoded["SRRRFODPB2P_VOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB2P_VOTk4PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB2P_VOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB2P_VOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB2P_VOTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB2P_VOTk4PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB2P_VOTk4PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB2P_VOTk4PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB2P_VOTk4PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'ac': # Consec Frame 2
            # Pedestrian Track 4
            if decoded["SRRRFODPB3P_POTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB3P_POTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB3P_POTk4MStsAuth"])
                #print(decoded["SRRRFODPB3P_POTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB3P_POTk4CnfdStsAuth"])
                #print(decoded["SRRRFODPB3P_POTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB3P_POTk4PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB3P_POTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB3P_POTk4PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB3P_POTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 5
            if decoded["SRRRFODPB3P_VOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB3P_VOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB3P_VOTk5MStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB3P_VOTk5CnfdStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB3P_VOTk5PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB3P_VOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB3P_VOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB3P_VOTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB3P_VOTk5PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB3P_VOTk5PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB3P_VOTk5PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB3P_VOTk5PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 6
            if decoded["SRRRFODPB3P_VOTk6MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB3P_VOTk6IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB3P_VOTk6MStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk6MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB3P_VOTk6CnfdStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk6CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB3P_VOTk6PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB3P_VOTk6PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB3P_VOTk6PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB3P_VOTk6PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB3P_VOTk6PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB3P_VOTk6PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB3P_VOTk6PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB3P_VOTk6PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

            # Vehicle Track 7
            if decoded["SRRRFODPB3P_VOTk7MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB3P_VOTk7IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB3P_VOTk7MStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk7MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB3P_VOTk7CnfdStsAuth"])
                #print(decoded["SRRRFODPB3P_VOTk7CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB3P_VOTk7PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB3P_VOTk7PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB3P_VOTk7PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB3P_VOTk7PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["SRRRFODPB3P_VOTk7PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB3P_VOTk7PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB3P_VOTk7PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB3P_VOTk7PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)

        case 'ad': # Consec Frame 3
            # Pedestrian Track 5
            if decoded["SRRRFODPB4P_POTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB4P_POTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB4P_POTk5MStsAuth"])
                #print(decoded["SRRRFODPB4P_POTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB4P_POTk5CnfdStsAuth"])
                #print(decoded["SRRRFODPB4P_POTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["SRRRFODPB4P_POTk5PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB4P_POTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB4P_POTk5PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB4P_POTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)
            
            
            # Vehicle Track 8
            if decoded["SRRRFODPB4P_VOTk8MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB4P_VOTk8IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB4P_VOTk8MStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk8MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB4P_VOTk8CnfdStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk8CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB4P_VOTk8PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB4P_VOTk8PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB4P_VOTk8PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB4P_VOTk8PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB4P_VOTk8PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB4P_VOTk8PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB4P_VOTk8PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB4P_VOTk8PoILgVCAuth"] #lon vel cov 
                

                msgs.append(new_m1)
            
            # Vehicle Track 9
            if decoded["SRRRFODPB4P_VOTk9MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB4P_VOTk9IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB4P_VOTk9MStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk9MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB4P_VOTk9CnfdStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk9CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB4P_VOTk9PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB4P_VOTk9PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB4P_VOTk9PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB4P_VOTk9PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB4P_VOTk9PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB4P_VOTk9PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["SRRRFODPB4P_VOTk9PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB4P_VOTk9PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 10
            if decoded["SRRRFODPB4P_VOTk10MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 4
                new_m1.track_id = decoded['SRRRFODPB4P_VOTk10IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["SRRRFODPB4P_VOTk10MStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk10MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["SRRRFODPB4P_VOTk10CnfdStsAuth"])
                #print(decoded["SRRRFODPB4P_VOTk10CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["SRRRFODPB4P_VOTk10PoILPsAuth"]
                new_m1.lon = decoded["SRRRFODPB4P_VOTk10PoILgPsAuth"]
                new_m1.lat_vel = decoded["SRRRFODPB4P_VOTk10PoILVAuth"]
                new_m1.lon_vel = decoded["SRRRFODPB4P_VOTk10PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by SRRRF
                 # Not provided by SRRRF

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["SRRRFODPB4P_VOTk10PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["SRRRFODPB4P_VOTk10PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["SRRRFODPB4P_VOTk10PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["SRRRFODPB4P_VOTk10PoILgVCAuth"] #lon vel cov
                

                msgs.append(new_m1)

        case 'ae': # Consec Frame 4
            pass # Currently provides no data of interest

        case 'af': 
            pass # Currently Provides no Data of interest

        case _: # Default case
            node.get_logger().info('Unrecognized Frame ID in SRRRF conversion: %s' % frame_id)
    return msgs

def my_22_fcm(node, frame_id, data):
    msgs = []
    if type(data) is not str:
        binary_data = data
    else:
        binary_data = bytes.fromhex(data)
    decoded = node.db_can8.decode_message(int(frame_id, 16), binary_data)
    
    match frame_id:
        case '8e':
            # Pedestrian Track 1
            if decoded["FCODPB1P_FVPOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB1P_FVPOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB1P_FVPOTk1MStsAuth"])
                #print(decoded["FCODPB1P_FVPOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB1P_FVPOTk1CnfdStsAuth"])
                #print(decoded["FCODPB1P_FVPOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB1P_FVPOTk1PoILPsAuth"]
                new_m1.lon = decoded["FCODPB1P_FVPOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB1P_FVPOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB1P_FVPOTk1PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_cov = 10.0 # Not provided by FC
                new_m1.theta_valid = False

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                
                

                msgs.append(new_m1)

            # Vehicle Track 1
            if decoded["FCODPB1P_FVVOTk1MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB1P_FVVOTk1IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB1P_FVVOTk1MStsAuth"])
                #print(decoded["FCODPB1P_FVVOTk1MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB1P_FVVOTk1CnfdStsAuth"])
                #print(decoded["FCODPB1P_FVVOTk1CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB1P_FVVOTk1PoILPsAuth"]
                new_m1.lon = decoded["FCODPB1P_FVVOTk1PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB1P_FVVOTk1PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB1P_FVVOTk1PoILgVAuth"]
                new_m1.theta = decoded["FCODPB1P_FVVOTk1HdgAngAuth"]
                 # Not provided by FC

                new_m1.theta_cov = decoded["FCODPB1P_FVVOTk1HdgAngCAuth"] #theta cov
                new_m1.theta_valid = True
                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB1P_FVVOTk1PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB1P_FVVOTk1PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB1P_FVVOTk1PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB1P_FVVOTk1PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB1P_FVVOTk1PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB1P_FVVOTk1BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB1P_FVVOTk1TSLgtStsAuth"])

                msgs.append(new_m1)

        case '8f':
            # Pedestrian Track 2
            if decoded["FCODPB2P_FVPOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB2P_FVPOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB2P_FVPOTk2MStsAuth"])
                #print(decoded["FCODPB2P_FVPOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB2P_FVPOTk2CnfdStsAuth"])
                #print(decoded["FCODPB2P_FVPOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB2P_FVPOTk2PoILPsAuth"]
                new_m1.lon = decoded["FCODPB2P_FVPOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB2P_FVPOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB2P_FVPOTk2PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_cov = 10.0 # Not provided by FC
                new_m1.theta_valid = False

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 2
            if decoded["FCODPB2P_FVVOTk2MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB2P_FVVOTk2IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB2P_FVVOTk2MStsAuth"])
                #print(decoded["FCODPB2P_FVVOTk2MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB2P_FVVOTk2CnfdStsAuth"])
                #print(decoded["FCODPB2P_FVVOTk2CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB2P_FVVOTk2PoILPsAuth"]
                new_m1.lon = decoded["FCODPB2P_FVVOTk2PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB2P_FVVOTk2PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB2P_FVVOTk2PoILgVAuth"]
                new_m1.theta = decoded["FCODPB2P_FVVOTk2HdgAngAuth"]
                 
                new_m1.theta_cov = decoded["FCODPB2P_FVVOTk2HdgAngCAuth"] #theta cov
                new_m1.theta_valid = True

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB2P_FVVOTk2PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB2P_FVVOTk2PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB2P_FVVOTk2PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB2P_FVVOTk2PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB2P_FVVOTk2PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB2P_FVVOTk2BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB2P_FVVOTk2TSLgtStsAuth"])

                msgs.append(new_m1)

        case '90':
            # Pedestrian Track 3
            if decoded["FCODPB3P_FVPOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB3P_FVPOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB3P_FVPOTk3MStsAuth"])
                #print(decoded["FCODPB3P_FVPOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB3P_FVPOTk3CnfdStsAuth"])
                #print(decoded["FCODPB3P_FVPOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB3P_FVPOTk3PoILPsAuth"]
                new_m1.lon = decoded["FCODPB3P_FVPOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB3P_FVPOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB3P_FVPOTk3PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_cov = 10.0
                new_m1.theta_valid = False

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 3
            if decoded["FCODPB3P_FVVOTk3MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB3P_FVVOTk3IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB3P_FVVOTk3MStsAuth"])
                #print(decoded["FCODPB3P_FVVOTk3MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB3P_FVVOTk3CnfdStsAuth"])
                #print(decoded["FCODPB3P_FVVOTk3CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB3P_FVVOTk3PoILPsAuth"]
                new_m1.lon = decoded["FCODPB3P_FVVOTk3PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB3P_FVVOTk3PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB3P_FVVOTk3PoILgVAuth"]
                new_m1.theta = decoded["FCODPB3P_FVVOTk3HdgAngAuth"]
                new_m1.theta_cov = decoded["FCODPB3P_FVVOTk3HdgAngCAuth"] #theta cov
                new_m1.theta_valid = True

                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB3P_FVVOTk3PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB3P_FVVOTk3PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB3P_FVVOTk3PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB3P_FVVOTk3PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB3P_FVVOTk3PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB3P_FVVOTk3BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB3P_FVVOTk3TSLgtStsAuth"])

                msgs.append(new_m1)
        
        case '91':
            # Pedestrian Track 4
            if decoded["FCODPB4P_FVPOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB4P_FVPOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB4P_FVPOTk4MStsAuth"])
                #print(decoded["FCODPB4P_FVPOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB4P_FVPOTk4CnfdStsAuth"])
                #print(decoded["FCODPB4P_FVPOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB4P_FVPOTk4PoILPsAuth"]
                new_m1.lon = decoded["FCODPB4P_FVPOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB4P_FVPOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB4P_FVPOTk4PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_valid = False
                new_m1.theta_cov = 10.0 #theta 

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov 
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 4
            if decoded["FCODPB4P_FVVOTk4MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB4P_FVVOTk4IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB4P_FVVOTk4MStsAuth"])
                #print(decoded["FCODPB4P_FVVOTk4MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB4P_FVVOTk4CnfdStsAuth"])
                #print(decoded["FCODPB4P_FVVOTk4CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB4P_FVVOTk4PoILPsAuth"]
                new_m1.lon = decoded["FCODPB4P_FVVOTk4PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB4P_FVVOTk4PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB4P_FVVOTk4PoILgVAuth"]
                new_m1.theta = decoded["FCODPB4P_FVVOTk4HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB4P_FVVOTk4HdgAngCAuth"] #theta cov
                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["FCODPB4P_FVVOTk4PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB4P_FVVOTk4PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB4P_FVVOTk4PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB4P_FVVOTk4PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB4P_FVVOTk4PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB4P_FVVOTk4BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB4P_FVVOTk4TSLgtStsAuth"])

                msgs.append(new_m1)

        case '92':
            # Pedestrian Track 5
            if decoded["FCODPB5P_FVPOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB5P_FVPOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB5P_FVPOTk5MStsAuth"])
                #print(decoded["FCODPB5P_FVPOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB5P_FVPOTk5CnfdStsAuth"])
                #print(decoded["FCODPB5P_FVPOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                
                new_m1.object_type = True

                new_m1.lat = decoded["FCODPB5P_FVPOTk5PoILPsAuth"]
                new_m1.lon = decoded["FCODPB5P_FVPOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB5P_FVPOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB5P_FVPOTk5PoILgVAuth"]
                new_m1.theta = 0.0 # Not provided by FC
                new_m1.theta_valid = False # Not provided by FC
                new_m1.theta_cov = 10.0

                """
                No Covariance provided for Pedestrians
                Use Covariance of 1 for Lat/Lon and Vel
                Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                """
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = 1.0 #lat cov
                new_m1.covariance[5] = 1.0 #lon cov  
                new_m1.covariance[10] = 1.0 #lat vel cov 
                new_m1.covariance[15] = 1.0 #lon vel cov
                

                msgs.append(new_m1)

            # Vehicle Track 5
            if decoded["FCODPB5P_FVVOTk5MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB5P_FVVOTk5IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB5P_FVVOTk5MStsAuth"])
                #print(decoded["FCODPB5P_FVVOTk5MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB5P_FVVOTk5CnfdStsAuth"])
                #print(decoded["FCODPB5P_FVVOTk5CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB5P_FVVOTk5PoILPsAuth"]
                new_m1.lon = decoded["FCODPB5P_FVVOTk5PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB5P_FVVOTk5PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB5P_FVVOTk5PoILgVAuth"]
                new_m1.theta = decoded["FCODPB5P_FVVOTk5HdgAngAuth"]
                new_m1.theta_valid = True # Not provided by FC
                new_m1.theta_cov = decoded["FCODPB5P_FVVOTk5HdgAngCAuth"] #theta cov
                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB5P_FVVOTk5PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB5P_FVVOTk5PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB5P_FVVOTk5PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB5P_FVVOTk5PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB5P_FVVOTk5PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB5P_FVVOTk5BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB5P_FVVOTk5TSLgtStsAuth"])

                msgs.append(new_m1)
        
        case '93':
            
            # Vehicle Track 6
            if decoded["FCODPB6P_FVVOTk6MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB6P_FVVOTk6IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB6P_FVVOTk6MStsAuth"])
                #print(decoded["FCODPB6P_FVVOTk6MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB6P_FVVOTk6CnfdStsAuth"])
                #print(decoded["FCODPB6P_FVVOTk6CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB6P_FVVOTk6PoILPsAuth"]
                new_m1.lon = decoded["FCODPB6P_FVVOTk6PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB6P_FVVOTk6PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB6P_FVVOTk6PoILgVAuth"]
                new_m1.theta = decoded["FCODPB6P_FVVOTk6HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB6P_FVVOTk6HdgAngCAuth"] #theta cov
                 # Not provided by FC

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB6P_FVVOTk6PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB6P_FVVOTk6PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB6P_FVVOTk6PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB6P_FVVOTk6PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB6P_FVVOTk6PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB6P_FVVOTk6BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB6P_FVVOTk6TSLgtStsAuth"])

                msgs.append(new_m1)

            # Vehicle Track 10
            if decoded["FCODPB6P_FVVOTk10MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB6P_FVVOTk10IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB6P_FVVOTk10MStsAuth"])
                #print(decoded["FCODPB6P_FVVOTk10MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB6P_FVVOTk10CnfdStsAuth"])
                #print(decoded["FCODPB6P_FVVOTk10CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB6P_FVVOTk10PoILPsAuth"]
                new_m1.lon = decoded["FCODPB6P_FVVOTk10PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB6P_FVVOTk10PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB6P_FVVOTk10PoILgVAuth"]
                new_m1.theta = decoded["FCODPB6P_FVVOTk10HdgAngAuth"]
                new_m1.theta_cov = decoded["FCODPB6P_FVVOTk10HdgAngCAuth"] #theta cov
                new_m1.theta_valid = True
                 # Not provided by FC

                
                # Use High covariance for Theta/Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["FCODPB6P_FVVOTk10PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB6P_FVVOTk10PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB6P_FVVOTk10PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB6P_FVVOTk10PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB6P_FVVOTk10PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB6P_FVVOTk10BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB6P_FVVOTk10TSLgtStsAuth"])

                msgs.append(new_m1)

        case '94':
            
            # Vehicle Track 7
            if decoded["FCODPB7P_FVVOTk7MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB7P_FVVOTk7IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB7P_FVVOTk7MStsAuth"])
                #print(decoded["FCODPB7P_FVVOTk7MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB7P_FVVOTk7CnfdStsAuth"])
                #print(decoded["FCODPB7P_FVVOTk7CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB7P_FVVOTk7PoILPsAuth"]
                new_m1.lon = decoded["FCODPB7P_FVVOTk7PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB7P_FVVOTk7PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB7P_FVVOTk7PoILgVAuth"]
                new_m1.theta = decoded["FCODPB7P_FVVOTk7HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB7P_FVVOTk7HdgAngCAuth"] #theta cov
                 # Not provided by FC

                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB7P_FVVOTk7PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB7P_FVVOTk7PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB7P_FVVOTk7PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB7P_FVVOTk7PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB7P_FVVOTk7PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB7P_FVVOTk7BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB7P_FVVOTk7TSLgtStsAuth"])

                msgs.append(new_m1)

        case '95':
            
            # Vehicle Track 8
            if decoded["FCODPB8P_FVVOTk8MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB8P_FVVOTk8IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB8P_FVVOTk8MStsAuth"])
                #print(decoded["FCODPB8P_FVVOTk8MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB8P_FVVOTk8CnfdStsAuth"])
                #print(decoded["FCODPB8P_FVVOTk8CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB8P_FVVOTk8PoILPsAuth"]
                new_m1.lon = decoded["FCODPB8P_FVVOTk8PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB8P_FVVOTk8PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB8P_FVVOTk8PoILgVAuth"]
                new_m1.theta = decoded["FCODPB8P_FVVOTk8HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB8P_FVVOTk8HdgAngCAuth"] #theta cov
                 # Not provided by FC

                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16 
                new_m1.covariance[0] = decoded["FCODPB8P_FVVOTk8PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB8P_FVVOTk8PoILgPsCAuth"] #lon cov 
                new_m1.covariance[10] = decoded["FCODPB8P_FVVOTk8PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB8P_FVVOTk8PoILgVCAuth"] #lon vel cov 
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB8P_FVVOTk8PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB8P_FVVOTk8BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB8P_FVVOTk8TSLgtStsAuth"])

                msgs.append(new_m1)

            # Vehicle Track 9
            if decoded["FCODPB8P_FVVOTk9MStsAuth"] != "No Object":
                new_m1 = Track()
                new_m1.timestamp = node.get_clock().now().to_msg()
                new_m1.timestamp = new_m1.timestamp
                new_m1.sensor_id = 2
                new_m1.track_id = decoded['FCODPB8P_FVVOTk9IDAuth']
                new_m1.u_id = node.i
                node.i += 1


                new_m1.track_status = mesaurement_status_str_to_int(decoded["FCODPB8P_FVVOTk9MStsAuth"])
                #print(decoded["FCODPB8P_FVVOTk9MStsAuth"])
                #print(new_m1.track_status)
                new_m1.confidence = confidence_str_to_int(decoded["FCODPB8P_FVVOTk9CnfdStsAuth"])
                #print(decoded["FCODPB8P_FVVOTk9CnfdStsAuth"])
                #print(new_m1.confidence)
                new_m1.object_type = False

                new_m1.lat = decoded["FCODPB8P_FVVOTk9PoILPsAuth"]
                new_m1.lon = decoded["FCODPB8P_FVVOTk9PoILgPsAuth"]
                new_m1.lat_vel = decoded["FCODPB8P_FVVOTk9PoILVAuth"]
                new_m1.lon_vel = decoded["FCODPB8P_FVVOTk9PoILgVAuth"]
                new_m1.theta = decoded["FCODPB8P_FVVOTk9HdgAngAuth"]
                new_m1.theta_valid = True
                new_m1.theta_cov = decoded["FCODPB8P_FVVOTk9HdgAngCAuth"] #theta cov
                 # Not provided by FC

                
                # Use High covariance for Omega as those values are invalid and thus should not be trusted
                new_m1.covariance = [0.0] * 16
                new_m1.covariance[0] = decoded["FCODPB8P_FVVOTk9PoILPsCAuth"]#lat cov
                new_m1.covariance[5] = decoded["FCODPB8P_FVVOTk9PoILgPsCAuth"] #lon cov  
                new_m1.covariance[10] = decoded["FCODPB8P_FVVOTk9PoILVCAuth"] #lat vel cov 
                new_m1.covariance[15] = decoded["FCODPB8P_FVVOTk9PoILgVCAuth"] #lon vel cov
                

                #FCM Vehicle Specific
                new_m1.track_width = decoded["FCODPB8P_FVVOTk9PoIWAuth"]
                new_m1.brake_light_status = brake_to_int(decoded["FCODPB8P_FVVOTk9BrkLgtStsAuth"])
                new_m1.turn_signal_status = turn_to_int(decoded["FCODPB8P_FVVOTk9TSLgtStsAuth"])

                msgs.append(new_m1)

        case 'e1':
            pass # Not Needed at this time

        case 'e2':
            pass # Not Needed at this time

        case 'e3':
            pass # Not Needed at this time

        case 'ed':
            pass # Not Needed at this time

        case 'ec':
            pass # Not Needed at this time

        case 'ee':
            pass # Not Needed at this time

        case 'ef':
            pass # Not Needed at this time
        case _:
            node.get_logger().info('Unrecognized Frame ID in FCM conversion: %s' % frame_id)

    return msgs
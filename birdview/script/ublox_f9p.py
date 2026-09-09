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



def gps2file2(lat,lon):
    path = "/home/connau/srp/birdview/hummer_path/gpslog_f9p.txt"
    # path = "a.txt"
    try:
        with open(path, "a") as gps_log:
            gps_log.write("[%f,%f]\n"%(lat, lon))
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
                    gps2file2(lat, lon)
                    
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

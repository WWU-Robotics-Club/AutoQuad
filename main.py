import argparse
import configparser
import cv2
import time
import sys
from pancv import PanCV
try:
    from pivideostream import PiVideoStream
except:
    usePiCamera = False
else:
    usePiCamera = True
import dronekit

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--config", help="Defaults to rasberry pi zero config. Options: windows",
    default="DEFAULT")
args = ap.parse_args()

# config file
config = configparser.ConfigParser()
config.read('config.ini')
configSection = args.config
framerate = config.getint(configSection,'framerate')
display = config.getboolean(configSection, 'display')
takeoffAlt = config.getint(configSection,'takeoffAlt')
targetCoords = dronekit.LocationGlobalRelative(config.getfloat(configSection, 'targetLat'),
    config.getfloat(configSection, 'targetLon'), takeoffAlt)
startChNum = config.get(configSection,'startChNum')
useSITL = config.getboolean(configSection, 'sitl')
DKconnect = config.getboolean(configSection, 'connect')
# set up image recognition
panCv = PanCV()
# HSV 8 bit: 0-180, 0-255, 0-255
panCv.colorLower1 = (0, 120, 50) 
panCv.colorUpper1 = (25, 255, 255)
panCv.colorLower2 = (160, 120, 50) 
panCv.colorUpper2 = (180, 255, 255)
panCv.targetRadius = config.getfloat(configSection, 'targetRadius')
panCv.cameraFOV = config.getfloat(configSection, 'cameraFoV')
panCv.horizontalRes = config.getint(configSection,'horizontalRes')
panCv.verticalRes = config.getint(configSection,'verticalRes')
panCv.resetHistorySize(config.getint(configSection,'historySize'))

# setup video input
# if a video path was not supplied, grab the reference to the webcam or pi camera
if config.get(configSection,'video') == '':
    useVideoFile = False
    if usePiCamera:
        camera = PiVideoStream(resolution=resolution, framerate=framerate)
        camera.start()
        print("Using pi camera")
    else:
        camera = cv2.VideoCapture(0)
        print("Using webcam")
else: # otherwise, grab a reference to the video file
    useVideoFile = True
    camera = cv2.VideoCapture(config.get(configSection,'video'))
    print("Using video file")

# set up dronekit
# if it should try connecting
if DKconnect:
    # if using simulator
    if useSITL:
        import dronekit_sitl
        # set up simulator
        sitl = dronekit_sitl.start_default()
        connectionString = sitl.connection_string()
        print("SITL running on %s" % connectionString)
    else:
        connectionString = config.get(configSection,'address')
    try: # connect to drone
        vehicle = dronekit.connect(connectionString, wait_ready=True, baud=57600)
    # Bad TCP connection
    except socket.error:
        print 'Dronekit exception: No server exists!'
    # Bad TTY connection
    except exceptions.OSError as e:
        print 'Dronekit exception: No serial exists!'
    except :
        print 'Dronekit exception while connecting.'
    # set up updates for attitude
    #def attitudeCallback(self, attrName, value):
    #    print(vehicle.attitude)
    #vehicle.add_attribute_listener('attitude', attitudeCallback)
        
    # print vehicle info
    print("Vehicle status:")
    print(" GPS: %s" % vehicle.gps_0)
    print(" Battery: %s" % vehicle.battery)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Mode: %s" % vehicle.mode.name)

    # set up vehicle
    if not useSITL and (vehicle.parameters['PLND_ENABLED'] != 1 or vehicle.parameters['PLND_TYPE'] != 1):
        vehicle.parameters['PLND_ENABLED'] == 1
        vehicle.parameters['PLND_TYPE'] == 1
        vehicle.commands.upload()
        print("Enabled precision landing. Please reboot.")
        time.sleep(2)
        sys.exit()
    print("Waiting for vehicle armable")
    while not vehicle.is_armable:
        time.sleep(1)
    print("Vehicle is armable.")
    
    # fly vehicle
    while vehicle.channels[startChNum] < 1700 and not useSITL:
        print("Waiting for start signal. Current level: "
            + str(vehicle.channels[config.get(configSection,'startChNum')]))
        time.sleep(1.5)
    print("Received start signal")
    print("Changing mode to guided")
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    print("Arming")
    vehicle.armed = True
    vehicle.commands.upload()
    while not vehicle.armed:
        time.sleep(1)
    print("Armed")
    print("Taking off")
    vehicle.simple_takeoff(takeoffAlt)
    print("Waiting to reach takeoff altitude")
    while vehicle.location.global_relative_frame.alt < takeoffAlt*0.95:
        time.sleep(1)
    print("Reached takeoff altitude")
    print("Using gps to go to target")
    localNED = vehicle.location.local_frame
    targetNorth = localNED.north + config.getfloat(configSection, 'targetNorth')
    targetEast = localNED.east + config.getfloat(configSection, 'targetEast')
    targetDown = localNED.down

# main loop
while not DKconnect or vehicle.channels[startChNum] > 1700 or useSITL:
    if usePiCamera and not useVideoFile:
        # if a new frame has been read from camera
        if camera.framesElapsed > 0: 
            frame = camera.read()
        # else wait for frame maybe?
    else:
        # grab the current frame
        grabbed, frame = camera.read()
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if useVideoFile and not grabbed:
            break
        
    panCv.detect(frame)
    # todo, call gotoLocalNED or sendLandMsg as needed
    if display:
        panCv.display()
        
    key = cv2.waitKey(1) & 0xFF 
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
    if DKconnect and not vehicle.armed:
        print("Vehicle disarmed. Exiting.")
        break

# cleanup
camera.release()
cv2.destroyAllWindows() # close any open windows
# Close vehicle object
if config.getboolean(configSection, 'connect'):
    vehicle.close()
    if config.getboolean(configSection, 'sitl'):
        # Shut down simulator
        sitl.stop()

# http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
def gotoLocalNED(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.commands.upload()

# see http://mavlink.org/messages/common#LANDING_TARGET
# https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml#L4078
def sendLandMsg(point):
    msg = vehicle.message_factory.landing_target_encode(
        0, # time_boot_ms (not used)
        0, # target num
        0, # frame. I don't think this matters for angle_x and y
        point.angleX, # target radians from center of image
        point.angleY, # target radians from center of image
        point.distance, # altitude.  Not supported?
        point.radians, # size of target in radians
        point.radians) # size of target in radians
    vehicle.send_mavlink(msg)
    vehicle.commands.upload()
def printMavlink(msg):
    # https://github.com/dronekit/dronekit-python/issues/270
    # Send arbitrary data over the Pixhawk's 3DR telemetry radio from DroneKit (in 251 byte chunks)
    # set target_system and target_component to your GCS's IDs or to 0 for broadcast
    # GCS ids (maybe) see https://github.com/mavlink/qgroundcontrol/issues/2907
    systemId = 255 # 0
    compId = 190 # 0
    msg = dronekitobj.vehicle.message_factory.file_transfer_protocol_encode(255,190,0,data_to_send)
    msg.pack(dronekitobj.vehicle.message_factory)

    # Can't use DroneKit's send_mavlink() since it alters the message's "target_system" field  
    # Instead, access the name-mangled superprivate class variable and write the msg to the autopilot
    dronekitobj.vehicle._MPVehicle__module.master.mav.send(msg)

import argparse
import configparser
import cv2
import time
import sys
import math
import socket
import exceptions
from pancv import PanCV
try:
    from pivideostream import PiVideoStream
except:
    USE_PI_CAMERA = False
else:
    USE_PI_CAMERA = True
import dronekit
from pymavlink import mavutil

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--config", help="Defaults to rasberry pi zero config. See config file for options",
    default="DEFAULT")
args = ap.parse_args()

# config file
config = configparser.ConfigParser()
config.read('config.ini')
configSection = args.config
FRAMERATE = config.getint(configSection,'framerate')
DISPLAY = config.getboolean(configSection, 'display')
TAKEOFF_ALT = config.getint(configSection,'takeoffAlt')
targetCoords = dronekit.LocationGlobalRelative(config.getfloat(configSection, 'targetLat'),
    config.getfloat(configSection, 'targetLon'), TAKEOFF_ALT)
START_CH_NUM = config.get(configSection,'startChNum')
USE_SITL = config.getboolean(configSection, 'sitl')
DK_CONNECT = config.getboolean(configSection, 'connect')
PRINT_TO_MAVLINK = config.getboolean(configSection, 'printToMavlink')
MAX_FLIGHT_TIME = config.getfloat(configSection, 'maxFlightTime')
MAX_MODE_TIME = config.getfloat(configSection, 'maxModeTime')
GEOFENCE_RAD = config.getfloat(configSection, 'geofenceRadius')
USE_VIDEO_FILE = False if config.get(configSection,'video') == '' else True
# Main loop modes
TAKEOFF = 1 # take off
GPS_LOCAL = 2 # fly to target using position relative to start
LAND_GUIDED = 3 # use precision landing
LAND_NORM = 4 # normal landing
GPS_GLOBAL = 5 # fly to target using global coordinates
ABORT = 6 # 
MANUAL = 7 # don't try to control quadcopter
mode = MANUAL
running = True

# other globals
homeLocal = None # dronekit.LocationLocal of start location. Should be close to (0,0,0)
panCv = PanCV()
sitl = None # reference to simulator object
vehicle = None # dronekit.Vehicle object
camera = None # camera object
targetLocal = dronekit.LocationLocal(
    config.getfloat(configSection, 'targetNorth'),
    config.getfloat(configSection, 'targetEast'),
    config.getfloat(configSection, 'targetDown'))
flightStartTime = None
modeStartTime = None
lastMsgTime = 0.0 # for keeping track of how often messages are sent
landGuidedAttempts = 0

# set up image recognition
def initCV(): 
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
    panCv.scaleRes(1)

# setup video input
def initCamera():
    global camera
    if USE_VIDEO_FILE:
        #grab a reference to the video file
        camera = cv2.VideoCapture(config.get(configSection,'video'))
        print("Using video file")
    # if a video path was not supplied, grab the reference to the webcam or pi camera
    else:
        if USE_PI_CAMERA:
            camera = PiVideoStream(resolution=(config.getint(configSection,'horizontalRes'),
                config.getint(configSection,'verticalRes')), framerate=FRAMERATE)
            camera.start()
            print("Using pi camera")
        else:
            camera = cv2.VideoCapture(0)
            printb("Using webcam")

# set up dronekit
def initDronekit():
    global sitl
    global mode
    global vehicle
    # if using simulator
    if USE_SITL:
        import dronekit_sitl
        # set up simulator
        sitl = dronekit_sitl.start_default()
        connectionString = sitl.connection_string()
        print("SITL running on %s" % connectionString)
    else:
        connectionString = config.get(configSection,'address')
    try: # connect to drone
        vehicle = dronekit.connect(connectionString, wait_ready=True, baud=config.getfloat(configSection,'baud'))
    # Bad TCP connection
    except socket.error:
        print 'Dronekit exception: No server exists!'
        return False
    # Bad TTY connection
    except exceptions.OSError as e:
        print 'Dronekit exception: No serial exists!'
        return False
    except dronekit.APIException:
        print 'Dronekit api exception: timeout'
        return False
    except :
        print 'Dronekit exception while connecting.'
        return False
    # set up updates for attitude
    def mode_callback(self, attrName, attrValue):
        global mode
        modeName = attrValue.name
        printb("Vehicle Mode changed to " + modeName)
        printb(attrValue)
        if modeName == "LAND" and mode != LAND_NORM:
            mode = LAND_NORM
        elif modeName != "GUIDED" and modeName != "LAND":
            mode = MANUAL
            printb("Mode switched away from guided. Ceasing control.")
    #Add observer callback for attribute mode
    vehicle.add_attribute_listener('mode', mode_callback)

    # set up vehicle
    if not USE_SITL and (vehicle.parameters['PLND_ENABLED'] != 1 or vehicle.parameters['PLND_TYPE'] != 1):
        vehicle.parameters['PLND_ENABLED'] == 1
        vehicle.parameters['PLND_TYPE'] == 1
        vehicle.commands.upload()
        printb("Enabled precision landing. Please reboot.")
        time.sleep(2)
        return False
    # print vehicle info
    print("Vehicle status:")
    print(" GPS: %s" % vehicle.gps_0)
    print(" Battery: %s" % vehicle.battery)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Mode: %s" % vehicle.mode.name)
    return True

def takeoff():
    global mode
    global homeLocal
    global modeStartTime
    global flightStartTime
    printb("Waiting for vehicle armable.")
    while not vehicle.is_armable:
        time.sleep(1)
    printb("Vehicle is armable.")
    
    # fly vehicle
    printb("Waiting for start signal. Current level: ")
    while vehicle.channels[START_CH_NUM] < 1700 and not USE_SITL:
        sys.stdout.write(str(vehicle.channels[START_CH_NUM]) + '\r')
        sys.stdout.flush()
        time.sleep(1)
    printb('\r')
    printb("Received start signal")
    printb("Waiting for gps fix. Current fix type, satelites: ")
    while vehicle.gps_0.fix_type != 3:
        sys.stdout.write(str(vehicle.gps_0.fix_type) + ","
            + str(vehicle.gps_0.satellites_visible) + '\r')
        sys.stdout.flush()
        time.sleep(1)
    printb('\r')
    printb("Changing mode to guided")
    vehicle.mode = dronekit.VehicleMode("GUIDED")
    printb("Arming")
    vehicle.armed = True
    vehicle.commands.upload()
    while not vehicle.armed:
        time.sleep(1)
    printb("Armed. Setting home location")
    homeLocal = vehicle.location.local_frame
    printb("Taking off")
    vehicle.simple_takeoff(TAKEOFF_ALT)
    flightStartTime = time.time()
    printb("Waiting to reach takeoff altitude")
    mode = TAKEOFF
    modeStartTime = time.time()

def cleanup():
    if not USE_VIDEO_FILE:
        camera.stop()
    else:
        camera.release()
    cv2.destroyAllWindows() # close any open windows
    # Close vehicle object
    if DK_CONNECT:
        vehicle.close()
        if USE_SITL:
            # Shut down simulator
            sitl.stop()

# http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
# NED relative to homeLocal
def gotoLocal(locationLocal):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        homeLocal.north + locationLocal.north,
        homeLocal.east + locationLocal.east,
        homeLocal.down + locationLocal.down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.commands.upload()

# see http://mavlink.org/messages/common#LANDING_TARGET
# https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml#L4078
def sendLandMsg(point):
    #print("landMsg" + str(point.angleX) + " " + str(point.angleY)
    #+ " " + str(point.distance) + " " + str(point.radians))
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
def printb(msg):
    print(msg)
    if PRINT_TO_MAVLINK:
        printMavlink(msg)
    
def printMavlink(msg): # this doesn't work
    # https://github.com/dronekit/dronekit-python/issues/270
    # Send arbitrary data over the Pixhawk's 3DR telemetry radio from DroneKit (in 251 byte chunks)
    # set target_system and target_component to your GCS's IDs or to 0 for broadcast
    # GCS ids (maybe) see https://github.com/mavlink/qgroundcontrol/issues/2907
    
    # trim bytes
    #msg = msg.encode('utf-8')[:250]
    #msg = msg.decode('utf-8', 'ignore')
    systemId = 255 # 0
    compId = 190 # 0
    msg = vehicle.message_factory.file_transfer_protocol_encode(0,255,190,msg)
    msg.pack(vehicle.message_factory)

    # Can't use DroneKit's send_mavlink() since it alters the message's "target_system" field  
    # Instead, access the name-mangled superprivate class variable and write the msg to the autopilot
    vehicle._MPVehicle__module.master.mav.send(msg)
def getDistance(aLocation1, aLocation2):
    # Returns the ground distance in metres between two LocationGlobal objects.
    # https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# get and process image. Returns true if new image was processed,
# False is no new image was available
def processImage():
    if USE_PI_CAMERA and not USE_VIDEO_FILE:
        # if a new frame has been read from camera
        if camera.framesElapsed > 0: 
            frame = camera.read()
        else:
            return False
    else:
        # grab the current frame
        grabbed, frame = camera.read()
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if USE_VIDEO_FILE and not grabbed:
            return False
    panCv.detect(frame)
    if DISPLAY:
        panCv.display()
    return True
def abort():
    global mode
    global modeStartTime
    vehicle.simple_goto(targetCoords)
    mode = ABORT
    modeStartTime = time.time()
def land():
    global mode
    global modeStartTime
    mode = LAND_NORM
    modeStartTime = time.time()
    vehicle.mode = dronekit.VehicleMode("LAND")
    vehicle.commands.upload() 

initCamera()
initCV()
if DK_CONNECT:
    if initDronekit():
        takeoff()
    else:
        running = False
# main loop. Run if dronekit isn't being used (images only),
# or if dronekit is being used check if switch on radio is enabled, unless running from simulator.
while running and (not DK_CONNECT or (DK_CONNECT and (vehicle.channels[START_CH_NUM] > 1700 or USE_SITL))):
    key = cv2.waitKey(1) & 0xFF 
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
    if DK_CONNECT:
        # safety and exit condition checks
        if not vehicle.armed:
            printb("Vehicle disarmed. Exiting.")
            break
        if (getDistance(vehicle.location.global_frame, targetCoords) > GEOFENCE_RAD
            and mode != GPS_GLOBAL and not USE_SITL):
            # maybe use GPS GLOBAL instead
            printb("Outside geofence. Switching to GPS_GLOBAL")
            mode = GPS_GLOBAL
            vehicle.simple_goto(targetCoords)
            modeStartTime = time.time()
            panCv.scaleRes(1)
            
        # modes that don't use camera
        if mode == ABORT:
            # just fly up to safe altitude. Handled by abort()
            time.sleep(0.2)
            continue
        if ((time.time() - modeStartTime > MAX_MODE_TIME
            or time.time() - flightStartTime > MAX_FLIGHT_TIME) and mode != ABORT):
            abort()
            printb("Max mode or flight time exceeded. Aborting.")
            continue
        if mode == TAKEOFF:
            if vehicle.location.global_relative_frame.alt < TAKEOFF_ALT*0.9:
                time.sleep(0.2)
            else:
                printb("Reached takeoff altitude. Switching to GPS_LOCAL")
                mode = GPS_LOCAL
                modeStartTime = time.time()
            continue
        if mode == LAND_NORM:
            if vehicle.mode.name != "LAND":
                vehicle.mode = dronekit.VehicleMode("LAND")
                vehicle.commands.upload()
            time.sleep(0.5)
            continue
        
        # computer vision
        # if no new frame is available, continue
        if not processImage():
            # returns false if no new frame is available. When reading from file
            # this only happens when last frame is reached.
            if USE_VIDEO_FILE and DK_CONNECT and USE_SITL:
                print("Last video frame reached. Landing")
                land()
            continue

        # modes that use the camera
        if mode == GPS_LOCAL or mode == GPS_GLOBAL:
            if panCv.points[0].confidence > 0.6:
                printb("Spotted target. Switching to LAND_GUIDED")
                mode = LAND_GUIDED
                modeStartTime = time.time()
            elif mode == GPS_LOCAL:
                # send command every second
                if time.time() - lastMsgTime > 1:
                    gotoLocal(targetLocal)
            elif mode == GPS_GLOBAL:
                pass
        if mode == LAND_GUIDED:
            if panCv.points[0].confidence > 0.6:
                sendLandMsg(panCv.points[0])
            # else if target wasn't found
            else:
                # count number of bad points
                lastAcceptableP = panCv.lastGoodPoint(0.6)
                lastAccAge = panCv.points[lastAcceptableP].age()
                lastGoodPoint = panCv.lastGoodPoint(1)
                # if it just lost it and was close, just land
                if (lastAcceptableP != -1 and lastAccAge < 0.5
                    and panCv.points[lastAcceptableP].distance < 0.9):
                    printb("Close enough to target. Landing.")
                    land()
                # if it hasn't had a fix for a while and wasn't close, switch back to GPS
                elif lastAcceptableP == -1 or lastAccAge > 1.5:
                    # retry up to 3 times
                    if landGuidedAttempts < 3:
                        printb("Lost target. Switching to GPS_LOCAL. Bad frames: " + str(lastAcceptableP) + ", last good frame age: " + str(lastAccAge))
                        mode = GPS_LOCAL
                        modeStartTime = time.time()
                        landGuidedAttempts += 1
                        panCv.scaleRes(1)
                    # if already retried twice just land
                    else:
                        printb("Lost target 3rd time. Landing.")
                        land()

    else: # only process images, presumably from file
        # if end of file reached
        if not processImage():
            printb("Last video frame reached. Exiting")
            break

cleanup()

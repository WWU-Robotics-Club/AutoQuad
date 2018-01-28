import argparse
import cv2
from pancv import PanCV
try:
    from pivideostream import PiVideoStream
except:
    usePiCamera = False
else:
    usePiCamera = True
import dronekit_sitl
from dronekit import connect, VehicleMode

panCv = PanCV()
panCv.colorLower = (90, 150, 80) # HSV for 8 bit: 0-180, 0-255, 0-255
panCv.colorUpper = (150, 255, 255)
# panCv.targetRadius = 
# panCv.cameraFOV =
resolution = (640, 480)
framerate = 30

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to (optional) video file to use in place of camera feed")
ap.add_argument("-s", "--sitl", help="Use the SITL (simulated) vehicle instead of physical drone",
    action="store_true")
ap.add_argument("-d", "--display", help="Draw each frame on the display",
    action="store_true")
ap.add_argument("-a", "--address", help="Connection string aka target address of vehicle to connect to. Default /dev/ttyAMA0",
    default="/dev/ttyAMA0")
ap.add_argument("-i", "--imgonly", help="Only process images, don't try to connect to quadcopter.",
    action="store_true")
args = ap.parse_args()

# setup video input
# if a video path was not supplied, grab the reference to the webcam or pi camera
if args.video == None:
    if usePiCamera:
        camera = PiVideoStream(resolution=resolution, framerate=framerate)
        camera.start()
        print("Using pi camera")
    else:
        camera = cv2.VideoCapture(0)
        print("Using webcam")
else: # otherwise, grab a reference to the video file
    camera = cv2.VideoCapture(args.video)
    print("Using video file")

# set up dronekit
if args.imgonly == False:
    if args.sitl == True:
        sitl = dronekit_sitl.start_default()
        connectionString = sitl.connection_string()
        print("SITL running on %s" % connectionString)
        vehicle = connect(connectionString)
        vehicle.wait_ready(True, timeout=30) # wait until available vehicle parameters have been collected
    else: # connect to drone
        vehicle = connect(args.address, wait_ready=True, baud=57600)
    # set up updates for attitude
    def attitudeCallback(self, attrName, value):
        print(vehicle.attitude)
    vehicle.add_attribute_listener('attitude', attitudeCallback)
        
    # print vehicle info
    print("Some vehicle attribute values:")
    print(" GPS: %s" % vehicle.gps_0)
    print(" Battery: %s" % vehicle.battery)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Mode: %s" % vehicle.mode.name) # settable

# main loop
while True:
    if usePiCamera:
        # if a new frame has been read from camera
        if camera.framesElapsed > 0: 
            frame = camera.read()
        # else wait for frame maybe?
    else:
        # grab the current frame
        grabbed, frame = camera.read()
        # if we are viewing a video and we did not grab a frame,
        # then we have reached the end of the video
        if args.video and not grabbed:
            break
        
    panCv.detect(frame)
    if args.display:
        panCv.display()
        
    key = cv2.waitKey(1) & 0xFF 
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# cleanup
camera.release()
cv2.destroyAllWindows() # close any open windows
# Close vehicle object
vehicle.close()
# Shut down simulator
if args.sitl:
    sitl.stop()

# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
import argparse
import cv2
from pancv import PanCV

panCv = PanCV()
panCv.colorLower = (29, 86, 6)
panCv.colorUpper = (64, 255, 255)
# panCv.targetRadius = 
# panCv.cameraFOV =

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
args = vars(ap.parse_args())
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    camera = cv2.VideoCapture(0)
# otherwise, grab a reference to the video file
else:
    camera = cv2.VideoCapture(args["video"])

while True:
    # grab the current frame
    grabbed, frame = camera.read()
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if args.get("video") and not grabbed:
        break
    panCv.detect(frame)

    panCv.display()
    key = cv2.waitKey(1) & 0xFF
 
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

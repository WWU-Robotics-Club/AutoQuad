from collections import deque
import numpy as np
import imutils
import cv2

class Point:
    imgPoint = None
    imgCentroid = None
    radius = None
    #time = None
    #x = None
    #y = None
    #z = None
    #confidence = None
class PanCV:
    camera = None

    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    colorLower = None
    colorUpper = None
    targetRadius = None # For calculating distance
    frameWidth = 600 # width in pixels to resize frame to
    cameraFOV = None # Degrees? Radians?
    historySize = 50
    points = deque(maxlen=historySize)
    lastFrame = None
    lastMask = None
    def resetHistorySize(self, size):
        self.historySize = size
        points = deque(maxlen=historySize)
    def detect(self, frame):
        currentPoint = Point()
     
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=self.frameWidth)
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, self.colorLower, self.colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        self.lastFrame = frame
        self.lastMask = mask
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
     
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            currentPoint.imgPoint = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            currentPoint.imgCentroid = (int(x),int(y))
            currentPoint.radius = radius

            #setCoords(currentPoint) # uncomment when setCoords is implemented
            # update the points queue
            self.points.appendleft(currentPoint)
        else:
            return False
        
        return True
    def setCoords(self, point): # calculate relative coordinates given a "point"
        # See Point class. Given self.cameraFOV, self.frameWidth,
        # x pixels (point.imgPoint[0]), y pixels (point.imgPoint[1]) and radius (point.radius),
        # populate point's x, y, and z with distance from target
        return
    def display(self):
        # loop over the set of tracked points
        for i in range(1, len(self.points)):
            # if either of the tracked points are None, ignore
            # them
            if self.points[i - 1] is None or self.points[i] is None:
                continue
     
            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(self.historySize / float(i + 1)) * 2.5)
            cv2.line(self.lastFrame, self.points[i - 1].imgPoint, self.points[i].imgPoint, (0, 0, 255), thickness)
        # only draw circle if the radius meets a minimum size
        if len(self.points) > 0 and self.points[0].radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(self.lastFrame, self.points[0].imgPoint, int(self.points[0].radius),
                (0, 255, 255), 2)
            cv2.circle(self.lastFrame, self.points[0].imgCentroid, 5, (0, 0, 255), -1)
        # show the frame to our screen
        cv2.imshow("Frame", self.lastFrame)

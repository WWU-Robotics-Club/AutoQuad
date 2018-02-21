# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
from collections import deque
import numpy as np
import imutils
import cv2
import math
import time

class Point:
    imgPoint = None # (x,y) position of moment of detected shape
    imgCentroid = None # (x,y) position of min enclosing circle
    radius = None # radius of target in meters
    radians = None # width of target in radians
    distance = None # triangulated distance from camera
    children = None # number of contours found within outer contour
    time = None # time point was processed
    angleX = None # radians from center of image
    angleY = None
    confidence = None # 0 to 1
    def age(self):
        return time.time() - self.time
class PanCV:
    camera = None

    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    colorLower = None
    colorUpper = None
    targetRadius = None # meters. For calculating distance
    horizontalRes = None # width in pixels to resize frame to
    verticalRes = None # height of frame after resizing
    cameraFOV = None # radians
    historySize = None
    points = deque(maxlen=historySize)
    lastFrame = None
    lastMask = None
    idealRadius = 50 # used when scaling
    # private
    _hResScaled = None
    _vResScaled = None
    _resScalar = None # scale resolution. Use lower when closer to get higher framerate
    def scaleRes(self, scale, add = False):
        if add:
            scale = scale + self._resScalar
        if scale > 1:
            scale = 1
        elif scale < 0.1:
            scale = 0.1
        self._resScalar = scale
        self._hResScaled = int(self.horizontalRes*scale)
        self._vResScaled = int(self.verticalRes*scale)
    # returns index of last 1 confidence point. -1 if none have 1 confidence
    def lastGoodPoint(self, threshold):
        i = 0
        lq = len(self.points)
        while i < lq and self.points[i].confidence < threshold:
            i += 1
        if self.points[i].confidence == 1:
            return i
        else:
            return -1
    def resetHistorySize(self, size):
        self.historySize = size
        self.points = deque(maxlen = size)
    def detect(self, frame):
        currentPoint = Point()
        currentPoint.time = time.time()
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=self._hResScaled)
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # construct a mask for the color, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask1 = cv2.inRange(hsv, self.colorLower1, self.colorUpper1)
        mask2 = cv2.inRange(hsv, self.colorLower2, self.colorUpper2)
        mask = mask1 + mask2
        
        #mask = cv2.erode(mask, None, iterations=1)
        #mask = cv2.dilate(mask, None, iterations=1)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        (_, contours, hierarchy) = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame, contours, -1, (255, 0, 0), 3)
        self.checkContours(contours, hierarchy, currentPoint)
        if currentPoint.confidence > 0:
            self.setCoords(currentPoint)
            cv2.putText(frame, "Dist: " + str(currentPoint.distance)[:5], (5,15), 1, 1.0, (0,200,200), 1)
            cv2.putText(frame, "Children: " + str(currentPoint.children)[:5], (5,30), 1, 1.0, (0,200,200), 1)
            cv2.putText(frame, "Confidence: " + str(currentPoint.confidence)[:5], (5,45), 1, 1.0, (0,200,200), 1)
            cv2.putText(frame, "Pix radius: " + str(currentPoint.radius)[:5], (5,60), 1, 1.0, (0,200,200), 1)

        # change scale
        if currentPoint.confidence == 1:
            self.scaleRes(self._resScalar * self.idealRadius / currentPoint.radius)
        else: # increase scale slightly to hopefully get better result
            self.scaleRes(0.02, True)
        # update the points queue
        self.points.appendleft(currentPoint)
        
        self.lastFrame = frame
        self.lastMask = mask
        return
    def checkContours(self, contours, hierarchy, point):
        # hierarchy is array of arrays containing indexes of contours: [[Next, Previous, First_Child, Parent]]
        # all within an array. So hierarchy[0][0] corresponds to contours[0], h[0][1] to c[1], and so on
        # loop through outer contours
        largest = None
        largestArea = 0
        numChildren = None
        # Search for largest contour
        for i in range(len(hierarchy[0])):
            if hierarchy[0][i][3] == -1: # Parent is -1 when is the contour is the outermost
                area = cv2.contourArea(contours[i])
                if area > largestArea:
                    largestArea = area
                    largest = i
                    # count number of children. Aka number of rings
                    numChildren = self.countChildren(hierarchy, i)
                        
        if largest != None: # if contour was found
            ((x, y), radius) = cv2.minEnclosingCircle(contours[largest])
            moments = cv2.moments(contours[largest])
            if moments["m00"] != 0:
                point.imgPoint = (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"]))
                point.imgCentroid = (int(x),int(y))
                point.radius = radius
                # 1 when 3 children. 0.5 when no children
                point.confidence = 1 - 0.5*(1 - numChildren/3)
            else:
                point.confidence = 0
            point.children = numChildren
        else:
            point.confidence = 0

        
    # returns longest chain of descendants of contour hierarchy
    # https://docs.opencv.org/trunk/d9/d8b/tutorial_py_contours_hierarchy.html
    def countChildren(self, hierarchy, index):
        siblingChildren = 0
        children = 0
        # add sibling and siblings children/siblings
        if hierarchy[0][index][0] != -1:
            siblingChildren = self.countChildren(hierarchy, hierarchy[0][index][0])
        # add child and child's children/siblings
        if hierarchy[0][index][2] != -1:
            children += 1
            children += self.countChildren(hierarchy, hierarchy[0][index][2])
        return children if children > siblingChildren else siblingChildren
    def setCoords(self, point): # calculate relative coordinates given a "point"
        # See Point class. Given self.cameraFOV, self.frameWidth,
        # x pixels (point.imgPoint[0]), y pixels (point.imgPoint[1]) and radius (point.radius),
        # populate point's x, y, and z with distance from target
        radPerPix = np.deg2rad(self.cameraFOV) / self._hResScaled
        halfTargetAngle = point.radius * radPerPix
        point.radians = 2 * halfTargetAngle
        point.distance = self.targetRadius / math.tan(halfTargetAngle)
        point.angleX = (point.imgCentroid[0] - self._hResScaled/2) * radPerPix
        point.angleY = (point.imgCentroid[1] - self._vResScaled/2) * radPerPix
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
            cv2.line(self.lastFrame, self.points[i - 1].imgPoint, self.points[i].imgPoint,
                (0, 0, 255), thickness)
        # only draw circle if the radius meets a minimum size
        if len(self.points) > 0 and self.points[0].confidence > 0 and self.points[0].radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(self.lastFrame, self.points[0].imgPoint, int(self.points[0].radius),
                (0, 255, 255), 2)
            cv2.circle(self.lastFrame, self.points[0].imgCentroid, 5, (0, 0, 255), -1)
        # show the frame to our screen
        cv2.imshow("Frame", self.lastFrame)
        cv2.imshow("Mask", self.lastMask)

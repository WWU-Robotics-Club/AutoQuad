# https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2
import time
 
class PiVideoStream:
    def __init__(self, resolution=(320, 240), framerate=32, threaded=True):
        # initialize the camera and stream
        self.camera = PiCamera()
        self._thread = None
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.threaded = threaded
        if threaded:
            self.stream = self.camera.capture_continuous(self.rawCapture,
                format="bgr", use_video_port=True)
            # number of frames read from camera since last read() call
            self.framesElapsed = 0
        else:
            # stays 1 if not threaded. Meaningless but allows the same logic to be used as for threaded
            self.framesElapsed = 1
            self.stream = None
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False
        #lastFrameTime = time.perf_counter()
        
    def start(self):
        # start the thread to read frames from the video stream
        if self.threaded:
            self._thread = Thread(target=self.update, args=())
            self._thread.start()
        return self
 
    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)
            self.framesElapsed += 1
            
            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return
    def read(self):
        if self.threaded:
            self.framesElapsed = 0
            # return the frame most recently read
            return self.frame
        else:
            self.camera.capture(self.rawCapture, format="bgr")
            self.frame = self.rawCapture.array
            self.rawCapture.truncate(0)
            return self.frame
 
    def stop(self):
        # stop the camera thread and wait for it to finish executing
        self.stopped = True
        if self.threaded:
            self._thread.join()
        else:
            self.rawCapture.close()
            self.camera.close()
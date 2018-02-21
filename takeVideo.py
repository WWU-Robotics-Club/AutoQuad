import picamera
import os

duration = 30
preview = True

with picamera.PiCamera() as camera:
    camera.resolution = (1280,720)
    if preview:
        camera.start_preview()
    camera.start_recording('video1.h264')
    camera.wait_recording(duration)
    camera.stop_recording()
    if preview:
        camera.stop_preview()
# convert to mp4
os.system("MP4Box -add video1.h264 video2.mp4")
os.system("rm video1.h264")

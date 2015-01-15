import tcam
import time
import picamera
import sys

cam = picamera.PiCamera()
cam.resolution = (1920, 1080)

tc = tcam.TempCam("/dev/ttyACM0")
tcv = tcam.Video(tc, camera=cam)

tcv.display(limit=0.5, width=80)

print("Beginning capture")
b = tcv.capture(int(sys.argv[2]), sys.argv[1], hcap=True, video=True)

#tcv.capture_to_movie(b, 'cap1')

tc.close()
tcv.close()
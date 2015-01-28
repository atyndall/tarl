from thinglib import *
import time
import picamera
import sys

pcam = picamera.PiCamera()
pcam.resolution = (1280, 720)

tc = cam.Manager("/dev/ttyACM0")
tcv = cam.Visualizer(tc, camera=pcam)

#tcv.display(limit=0.5, width=80)

print("Beginning capture")
b = tcv.capture(int(sys.argv[2]), sys.argv[1], hcap=True, video=True)

#tcv.capture_to_movie(b, 'cap1')

tc.close()
tcv.close()

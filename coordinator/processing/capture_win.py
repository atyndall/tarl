from thinglib import *
import time
import sys

tc = cam.Manager("COM4")
tcv = cam.Visualizer(tc)


print("Beginning capture")
b = tcv.capture(5, 'test1', hcap=True, video=True)

tcv.playback('test1')

#tcv.capture_to_movie(b, 'cap1')

tc.close()
tcv.close()
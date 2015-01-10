import tcam
import time

tc = tcam.TempCam("COM4")
tcv = tcam.Video(tc)

print("Beginning capture")
b = tcv.capture(10, 'test')

tcv.playback('test')
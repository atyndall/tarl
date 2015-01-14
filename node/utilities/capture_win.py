import tcam
import time
import sys

tc = tcam.TempCam("COM4")
tcv = tcam.Video(tc)


print("Beginning capture")
b = tcv.capture(5, 'test1', hcap=True, video=True)

tcv.playback('test1')

#tcv.capture_to_movie(b, 'cap1')

tc.close()
tcv.close()
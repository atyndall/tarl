import tcam
import time

tc = tcam.TempCam("COM4")

tc.display(block=True, tmin=15, tmax=40)

#print("Beginning capture")
#b = tc.capture(30, 'test', True)

#print(b)

#try:
#  time.sleep(35)
#except KeyboardInterrupt:
#  tc.close()

tc.close() 
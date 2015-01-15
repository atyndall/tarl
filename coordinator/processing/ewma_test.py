import tcam
import features
import time
from multiprocessing import Process, freeze_support

if __name__ == '__main__':
  freeze_support()

  tc = tcam.TempCam("COM4")

  q = tc.subscribe()

  ew = features.Features(q) 

  time.sleep(5)
  print("MOTION ENABLED")
  ew.motion = True
  time.sleep(9999)
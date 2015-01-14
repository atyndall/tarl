import tcam
import ewma
import time
from multiprocessing import Process, freeze_support

if __name__ == '__main__':
  freeze_support()

  tc = tcam.TempCam("COM4")

  q = tc.subscribe()

  ew = ewma.EWMA(q) 

  time.sleep(10)
  print("MOTION ENABLED")
  ew.motion = True
  time.sleep(9999)
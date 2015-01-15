from thinglib import *
import time
from multiprocessing import Process, freeze_support

if __name__ == '__main__':
  freeze_support()

  tc = cam.Manager("COM4")

  q = tc.subscribe()

  ew = features.Features(q) 

  time.sleep(5)
  print("MOTION ENABLED")
  ew.motion = True
  time.sleep(9999)
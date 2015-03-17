import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from thinglib import *

import time
import picamera
from multiprocessing import Process, freeze_support

if __name__ == '__main__':
  freeze_support()
  pcam = picamera.PiCamera()

  tc = cam.OnDemandManager("/dev/ttyACM0")
  tcv = cam.Visualizer(tc, camera=pcam)

  #tcv.display(limit=0.5, width=80)

  print("Beginning capture")
  b = tcv.capture_synced(int(sys.argv[2]), sys.argv[1], hz=2)

  #tcv.capture_to_movie(b, 'cap1')

  tc.close()
  tcv.close()

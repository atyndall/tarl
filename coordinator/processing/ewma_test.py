from thinglib import *
import time
from multiprocessing import Process, freeze_support

if __name__ == '__main__':
  freeze_support()

  tc = cam.Manager("/dev/tty.usbmodem411")
  tcv = cam.Visualizer(tc)
  tcv.display(width=80)

  q = tc.subscribe()

  print("Running at ", tc.irhz)

  ew = features.Features(q, tc.irhz) 

  time.sleep(9999)
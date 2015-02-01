from thinglib import *
from multiprocessing import Process, freeze_support
import sys

if __name__ == '__main__':
  freeze_support()
  tc = cam.Manager(sys.argv[1])
  tcv = cam.Visualizer(tc)
  tcv.display(block=True, tmin=15, tmax=50, width=80)
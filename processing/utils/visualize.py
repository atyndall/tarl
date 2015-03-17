import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from thinglib import *

from multiprocessing import Process, freeze_support

if __name__ == '__main__':
  freeze_support()
  tc = cam.Manager(sys.argv[1])
  tcv = cam.Visualizer(tc)
  tcv.display(block=True, tmin=15, tmax=50, width=80)
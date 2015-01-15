from thinglib import *
from multiprocessing import Process, freeze_support

if __name__ == '__main__':
  freeze_support()
  tc = cam.Manager("COM4")
  tcv = cam.Visualizer(tc)
  tcv.display(block=True, tmin=15, tmax=35)
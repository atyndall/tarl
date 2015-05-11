import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from tarl import *

import time
from multiprocessing import Process, freeze_support, Queue
import csv

if __name__ == '__main__':
  freeze_support()
  tcv = cam.Visualizer(None)
  caps = tcv.file_to_capture(sys.argv[1]+'/output')[1]

  temps = [[] for x in range(16)]

  with open('graph.csv', 'wb') as csvfile:
    writer = csv.writer(csvfile)

    for seq, cap in enumerate(caps):
      for i in range(16):
        temps[i].append(cap['ir'][0][i])

    for temp in temps:
      writer.writerow(temp)


  tcv.close()

  time.sleep(1)
  sys.exit()
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from thinglib import *

import time
from multiprocessing import Process, freeze_support, Queue
import csv

header = """@RELATION persondata

@ATTRIBUTE npeople  NUMERIC
@ATTRIBUTE numactive   NUMERIC
@ATTRIBUTE numconnected  NUMERIC
@ATTRIBUTE sizeconnected   NUMERIC

@DATA"""

if __name__ == '__main__':
  freeze_support()
  tcv = cam.Visualizer(None)

  with open('weka.csv', 'wb') as csvfile:
    csvfile.write(header)

    dirs = sys.argv[1].split(',')

    for dname in dirs:
      print("Processing {}...".format(dname))
      q = Queue()
      feat = features.Features(q, 1, display=False)
      caps = tcv.file_to_capture(dname+'/output')[1]

      with open(dname+'/truth', 'r') as truthfile:
        writer = csv.writer(csvfile)
        truths = truthfile.readlines()

        for seq, (cap, truth) in enumerate(zip(caps, truths)):
          cap['movement'] = False
          if seq > 20:
            cap['movement'] = True

          print('test1')
          q.put(cap)
          print('test2')

          feats = feat.get_features()

          row = [int(truth.strip())] + list(feats)
          row = ['?' if x is None else x for x in row]

          writer.writerow(row)

      q.close()
      feat.close()

  tcv.close()

  time.sleep(1)
  sys.exit()
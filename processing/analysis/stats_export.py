import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from tarl import *

import time
import fractions
import subprocess
import numpy as np
import csv

tcv = cam.Visualizer()

name = sys.argv[1]

hz, frames = tcv.file_to_capture(name)

tcv.close()

coll    = [ [ [] for _ in range(16) ] for _ in range (4)]
avg     = [ [None] * 16 for _ in range (4)]
stddev  = [ [None] * 16 for _ in range (4)]

for frame in frames:
  for i, row in enumerate(frame['ir']):
    for j, v in enumerate(row):
      coll[i][j].append(v)

for i, row in enumerate(coll):
  for j, vals in enumerate(row):
    avg[i][j] = np.mean(vals)
    stddev[i][j] = np.std(vals)

cname = '{}_stats.csv'.format(name)
with open(cname, 'wb') as csvfile:
  cwriter = csv.writer(csvfile)

  cwriter.writerow(['Hz', hz])

  cwriter.writerow(['Avg'])

  for row in avg:
    cwriter.writerow(row)

  cwriter.writerow(['StdDev'])

  for row in stddev:
    cwriter.writerow(row)

  cwriter.writerow(['Both'])

  for i in range(4):
    cwriter.writerow(['{} {}'.format(round(avg[i][j],2), round(stddev[i][j],2)) for j in range(16)])

print('Stats written to "{}"'.format(cname))
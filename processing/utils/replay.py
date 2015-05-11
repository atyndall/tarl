import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from tarl import *

import time
from multiprocessing import Process, freeze_support

if __name__ == '__main__':
  freeze_support()
  tc = cam.ManagerPlaybackEmulator()
  tcv = cam.Visualizer(tc)

  cap = tcv.file_to_capture(sys.argv[1])
  tc.set_playback_data(cap)

  feat = features.Features(tc.subscribe(), int(tc.irhz), motion_window=2)

  tcv.display()
  tc.start()

  time.sleep(9999)

  tc.close()
  tcv.close()
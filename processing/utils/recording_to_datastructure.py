import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from thinglib import *

from multiprocessing import Process, freeze_support

def get_packet():
  line = f.readline().decode("ascii", "ignore").strip()
  msg = []

  # Capture a whole packet
  while not line.startswith("START"):
    line = f.readline().decode("ascii", "ignore").strip()

  while not line.startswith("STOP"): 
    msg.append(line)
    line = f.readline().decode("ascii", "ignore").strip()

  msg.append(line)

  return msg

if __name__ == '__main__':
  freeze_support()

  tcv = cam.BaseManager('', init=False)

  temps = []

  with open(sys.argv[1], 'r') as f:
    while True:
      packet = tcv._decode_packet(get_packet(), splitchar=None)

      temps.append(packet['ir'][2][7])

      if f.tell() == os.fstat(f.fileno()).st_size:
        break

  for t in temps:
    print('{0:.2f} '.format(t))
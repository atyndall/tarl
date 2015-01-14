import threading
import display_helpers
import time
import math

def tuple_to_list(l):
  new = []

  for r in l:
    new.append(list(r))

  return new

class EWMA:
  _q = None
  _thread = None

  _img = None
  _means = None
  _stds = None

  weight = None

  motion = False

  def __init__(self, q, weight=0.1):
    self._q = q
    self.weight = weight

    self._thread = threading.Thread(group=None, target=self._monitor_thread)
    self._thread.daemon = True

    self._thread.start()


  def _monitor_thread(self):
    bdisp, _ = display_helpers.create_pixel_display(caption="Background")
    ddisp, _ = display_helpers.create_pixel_display(caption="Deviation")

    n = 1
    while True:
      frame = self._q.get()

      if not self.motion:
        if n == 1:
          self._img = tuple_to_list(frame['ir'])
          self._means = tuple_to_list(frame['ir'])
          self._stds = [[0 for x in range(16)] for x in range(4)]
        else:
          for i in range(4):
            for j in range(16):
              prev = self._img[i][j]
              cur = frame['ir'][i][j]

              cur_mean = self._means[i][j]
              cur_std = self._stds[i][j]

              self._img[i][j]   = self.weight * cur + (1 - self.weight) * prev
              self._means[i][j] = cur_mean + (cur - cur_mean) / n
              self._stds[i][j]  = cur_std + (cur - cur_mean) * (cur - self._means[i][j])

        bdisp.put({'ir': self._img})

      if n >= 2:
        std = {'ir': tuple_to_list(frame['ir'])}

        for i in range(4):
          for j in range(16):
            sigma = math.sqrt(self._stds[i][j] / (n-1))

            if (frame['ir'][i][j] - self._img[i][j]) < (3 * sigma):
              std['ir'][i][j] = 0

        ddisp.put(std) 

      if not self.motion:
        n += 1

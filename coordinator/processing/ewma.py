from __future__ import division
import threading
import display_helpers
import time
import math
import copy

def tuple_to_list(l):
  new = []

  for r in l:
    new.append(list(r))

  return new

def min_temps(l, n):
  flat = []
  for i, r in enumerate(l):
    for j, v in enumerate(r):
      flat.append(((i,j), v))
  flat.sort(key=lambda x: x[1])

  ret = [x[0] for x in flat]
  return ret[:n]


def init_arr(val=None):
  return [[val for x in range(16)] for x in range(4)]

class EWMA:
  _q = None
  _thread = None

  _background = None
  _means = None
  _stds = None
  _stds_post = None
  _active = None

  _lock = None

  motion_weight = None
  nomotion_weight = None

  motion = False

  display = None

  def __init__(self, q, motion_weight=0.1, nomotion_weight=0.01, display=True):
    self._q = q
    self.motion_weight = motion_weight
    self.nomotion_weight = nomotion_weight
    self.display = display

    self._active = []

    self._thread = threading.Thread(group=None, target=self._monitor_thread)
    self._thread.daemon = True

    self._lock = threading.Lock()

    self._thread.start()

  def get_background(self):
    self._lock.acquire()
    background = copy.deepcopy(self._background)
    self._lock.release()
    return background

  def get_means(self):
    self._lock.acquire()
    means = copy.deepcopy(self._means)
    self._lock.release()
    return means

  def get_stds(self):
    self._lock.acquire()
    stds = copy.deepcopy(self._stds_post)
    self._lock.release()
    return stds

  def get_active(self):
    self._lock.acquire()
    active = copy.deepcopy(self._active)
    self._lock.release()
    return active

  def _monitor_thread(self):
    bdisp = None
    ddisp = None

    n = 1
    while True:
      if self.display and bdisp is None:
        bdisp, _ = display_helpers.create_pixel_display(caption="Background")
        ddisp, _ = display_helpers.create_pixel_display(caption="Deviation")

      frame = self._q.get()['ir']

      self._lock.acquire()

      self._active = []

      if n == 1:
        self._background = tuple_to_list(frame)
        self._means = tuple_to_list(frame)
        self._stds = init_arr(0)
        self._stds_post = init_arr()
      else:
        weight = self.nomotion_weight
        use_frame = frame

        # Not currently working
        #if self.motion:
        #  indeces = min_temps(frame, 5)
        #  scalepx = []
        #
        #  for i, j in indeces:
        #    scalepx.append(self._background[i][j] / frame[i][j])
        #
        #  scale = sum(scalepx) / len(scalepx)
        #  scaled_bg = [[x * scale for x in r] for r in frame]
        #
        #  weight = self.motion_weight
        #  use_frame = scaled_bg
          
        for i in range(4):
          for j in range(16):
            prev = self._background[i][j]
            cur = use_frame[i][j]

            cur_mean = self._means[i][j]
            cur_std = self._stds[i][j]

            if not self.motion: # TODO: temp fix
              self._background[i][j]   = weight * cur + (1 - weight) * prev

              # maybe exclude these from motion calculations?
              # n doesn't change when in motion, so it'll cause all sort of corrupted results, as they use n?
              self._means[i][j] = cur_mean + (cur - cur_mean) / n
              self._stds[i][j]  = cur_std + (cur - cur_mean) * (cur - self._means[i][j])
              self._stds_post[i][j] = math.sqrt(self._stds[i][j] / (n-1))

            if (cur - self._background[i][j]) > (3 * self._stds_post[i][j]):
              self._active.append((i,j))

      self._lock.release()
      
      if self.display:
        active = self.get_active()
        bdisp.put({'ir': self._background})

        if n >= 2:
          std = {'ir': init_arr(0)}

          for i, j in active:
            std['ir'][i][j] = frame[i][j]

          ddisp.put(std) 

      if not self.motion:
        n += 1

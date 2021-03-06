from __future__ import division
from __future__ import print_function

import threading
import pxdisplay
import time
import math
import copy
import networkx as nx
import itertools
import collections
import matplotlib.pyplot as plt

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

class Features(object):
  _q = None
  _thread = None

  _background = None
  _means = None
  _stds = None
  _stds_post = None
  _active = None

  _num_active = None
  _connected_graph = None
  _num_connected = None
  _size_connected = None

  _lock = None

  _rows = None
  _columns = None

  motion_weight = None
  nomotion_weight = None

  motion_window = None

  hz = None

  display = None

  _exit = False

  draw = False
  freeze = False

  def __init__(self, q, hz, motion_window=10, motion_weight=0.1, nomotion_weight=0.01, display=True, rows=4, columns=16):
    self._q = q
    self.hz = hz
    self.motion_weight = motion_weight
    self.nomotion_weight = nomotion_weight
    self.display = display
    self.motion_window = motion_window

    self.motion_override = None

    self._active = []

    self._rows = rows
    self._columns = columns

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

  def get_features(self):
    self._lock.acquire()
    num_active = self._num_active
    num_connected = self._num_connected
    size_connected = self._size_connected
    self._lock.release()
    return (num_active, num_connected, size_connected)

  def close(self):
    self._exit = True

    if self._thread is not None:
      while self._thread.is_alive(): # Wait for thread to terminate
        pass

  def __del__(self):
    self.close()

  def _monitor_thread(self):
    bdisp = None
    ddisp = None

    freq = self.hz * self.motion_window
    mwin = collections.deque([False] * freq)

    n = 1
    while True:
      fdata = None

      if self._exit:
        return

      try:
        fdata = self._q.get(True, 0.3)
      except:
        continue

      if self.display and bdisp is None:
        ndisp, _ = pxdisplay.create(caption="Normal", width=80)
        bdisp, _ = pxdisplay.create(caption="Average", width=80)
        sdisp, _ = pxdisplay.create(caption="Std Dev", width=80, tmin=0, tmax=0.5)
        ddisp, _ = pxdisplay.create(caption="Deviation", width=80)

      frame = fdata['ir']

      mwin.popleft()
      mwin.append(fdata['movement'])
      if self.motion_override is not None:
        motion = self.motion_override
      else:
        motion = any(mwin)

      self._lock.acquire()

      self._active = []

      g = nx.Graph()

      if n == 1:
        self._background = tuple_to_list(frame)
        self._means = tuple_to_list(frame)
        self._stds = init_arr(0)
        self._stds_post = init_arr()
      else:
        weight = self.nomotion_weight
        use_frame = frame

        # Not currently working
        #if motion:
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
          
        for i in range(self._rows):
          for j in range(self._columns):
            prev = self._background[i][j]
            cur = use_frame[i][j]

            cur_mean = self._means[i][j]
            cur_std = self._stds[i][j]

            if not motion: # TODO: temp fix
              self._background[i][j]   = weight * cur + (1 - weight) * prev

              # maybe exclude these from motion calculations?
              # n doesn't change when in motion, so it'll cause all sort of corrupted results, as they use n?
              self._means[i][j] = cur_mean + (cur - cur_mean) / n
              self._stds[i][j]  = cur_std + (cur - cur_mean) * (cur - self._means[i][j])
              self._stds_post[i][j] = math.sqrt(self._stds[i][j] / (n-1))

            if (cur - self._background[i][j]) > (3 * self._stds_post[i][j]):
              self._active.append((i,j))

              g.add_node((i,j))

              x = [(-1, -1), (-1, 0), (-1, 1), (0, -1)] # Nodes that have already been computed as active
              for ix, jx in x:
                if (i+ix, j+jx) in self._active:
                  g.add_edge((i,j), (i+ix,j+jx))

      active = self._active

      self._num_active = len(self._active)

      components = list(nx.connected_components(g))

      self._connected_graph = g
      self._num_connected = nx.number_connected_components(g)
      self._size_connected = max(len(component) for component in components) if len(components) > 0 else None

      self._lock.release()
    
      active_pix = None

      if self.display:
        ndisp.put({'ir': frame})
        bdisp.put({'ir': self._background})
        sdisp.put({'ir': self._stds_post})

        #print(self._stds_post)

        if n >= 2:
          std = {'ir': init_arr(0)}

          for i, j in active:
            std['ir'][i][j] = frame[i][j]

          ddisp.put(std)
          active_pix = std

      while self.freeze:
        time.sleep(0.5)
        pass

      if self.draw:
        #nx.draw(g)
        #plt.show()

        print('########################')

        print('Edgelist')

        for l in nx.generate_edgelist(g, data=False):
          print(l)

        print('Frame')
        print(frame)

        print('Average')
        print(self._background)

        print('Stddev')
        print(self._stds_post)

        print('Deviation')
        print(active_pix)

        while True:
          pass

      if not motion:
        n += 1

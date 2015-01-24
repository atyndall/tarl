from __future__ import division
from __future__ import print_function

import serial
import copy
import Queue as queue
import time
from collections import deque
import threading
import pygame
import colorsys
import datetime
from PIL import Image
import subprocess
import tempfile
import os
import os.path
import fractions
import pxdisplay
import multiprocessing

class Manager(object):
  tty = None
  baud = None

  driver = None
  build = None
  irhz = None

  _serial_thread = None
  _serial_stop = False
  _serial_obj = None
  _serial_ready = False

  _decode_thread = None

  _read_decode_queue = None

  _temps = None

  _queues = []

  def __init__(self, tty, baud=115200):
    self.tty = tty
    self.baud = baud

    self._serial_obj = serial.Serial(port=self.tty, baudrate=self.baud, rtscts=True, dsrdtr=True)

    self._serial_thread = threading.Thread(group=None, target=self._read_thread_run)
    self._serial_thread.daemon = True

    self._decode_thread = threading.Thread(group=None, target=self._decode_thread_run)
    self._decode_thread.daemon = True

    self._serial_obj.write('r') # Reset the sensor
    self._serial_obj.flush()    

    self._read_decode_queue = queue.Queue()

    self._decode_thread.start()
    self._serial_thread.start()

    while not self._serial_ready: # Wait until we've populated data before continuing
      pass

  def __del__(self):
    self.close() 

  def _decode_packet(self, packet):
    decoded_packet = {}
    ir = []

    for line in packet:
      parted = line.partition(" ")
      cmd = parted[0]
      val = parted[2]

      try:
        if cmd == "START":
          decoded_packet['start_millis'] = long(val)
        elif cmd == "STOP":
          decoded_packet['stop_millis'] = long(val)
        elif cmd == "MOVEMENT":
          if val == "0":
            decoded_packet['movement'] = False
          elif val == "1":
            decoded_packet['movement'] = True
        else:
          ir.append(tuple(float(x) for x in line.split("\t")))
      except ValueError:
        print(packet)
        print("WARNING: Could not decode corrupted packet") 
        return {}

    decoded_packet['ir'] = tuple(ir)

    return decoded_packet

  def _decode_info(self, packet):
    decoded_packet = {}
    ir = []

    for line in packet:
      parted = line.partition(" ")
      cmd = parted[0]
      val = parted[2]

      if cmd == "INFO":
        pass
      elif cmd == "DRIVER":
        decoded_packet['driver'] = val
      elif cmd == "BUILD":
        decoded_packet['build'] = val
      elif cmd == "IRHZ":
        decoded_packet['irhz'] = int(val) if int(val) != 0 else 0.5

    return decoded_packet

  def close(self):
    self._serial_stop = True

    if self._serial_thread is not None:
      while self._serial_thread.is_alive(): # Wait for thread to terminate
        pass

  def get_temps(self):
    if self._temps is None:
      return False
    else:
      return copy.deepcopy(self._temps)

  def subscribe(self):
    q = queue.Queue()
    self._queues.append(q)
    return q

  def subscribe_multiprocess(self):
    q = multiprocessing.Queue()
    self._queues.append(q)
    return q

  def subscribe_lifo(self):
    q = queue.LifoQueue()
    self._queues.append(q)
    return q

  def _update_info(self):
    ser = self._serial_obj

    ser.write('i')
    ser.flush()
    imsg = []

    line = ser.readline().decode("ascii", "ignore").strip()

    # Capture a whole packet
    while not line == "INFO START":
      line = ser.readline().decode("ascii", "ignore").strip()

    while not line == "INFO STOP":
      imsg.append(line)
      line = ser.readline().decode("ascii", "ignore").strip()

    imsg.append(line)

    packet = self._decode_info(imsg)

    self.driver = packet['driver']
    self.build = packet['build']
    self.irhz = packet['irhz']

  def _read_thread_run(self):
    ser = self._serial_obj
    q = self._read_decode_queue
    self._update_info()

    while True:
      line = ser.readline().decode("ascii", "ignore").strip()
      msg = []

      # Capture a whole packet
      while not line.startswith("START"):
        line = ser.readline().decode("ascii", "ignore").strip()

      while not line.startswith("STOP"): 
        msg.append(line)
        line = ser.readline().decode("ascii", "ignore").strip()

      msg.append(line)

      q.put(msg)
      self._serial_ready = True

      if self._serial_stop:
        ser.close()
        return

  def _decode_thread_run(self):
    dq = self._read_decode_queue
    while True:
      msg = dq.get(block=True)

      dpct = self._decode_packet(msg)

      if 'ir' in dpct:
        self._temps = dpct

        for q in self._queues:
          q.put(self.get_temps())

      if self._serial_stop:
        return

class Visualizer(object):
  _display_thread = None
  _display_stop = False
  _tmin = None
  _tmax = None
  _limit = None
  _dwidth = None

  _tcam = None
  _ffmpeg_loc = None

  _camera = None

  def __init__(self, tcam=None, camera=None, ffmpeg_loc="ffmpeg"):
    self._tcam = tcam
    self._ffmpeg_loc = ffmpeg_loc
    self._camera = camera

  def display(self, block=False, limit=0, width=100, tmin=15, tmax=45):
    q = self._tcam.subscribe_multiprocess()
    _, proc = pxdisplay.create(q, limit=limit, width=width, tmin=tmin, tmax=tmax)

    if block:
      proc.join()

  def playback(self, filen, tmin=15, tmax=45):
    hz, playdata = self.file_to_capture(filen)

    print(hz)

    q, thread = pxdisplay.create(
      limit=hz,
      tmin=tmin, 
      tmax=tmax, 
      caption="Playing back '{}'".format(filen)
    )

    start = datetime.datetime.now()
    offset = playdata[0]['start_millis']

    for n, frame in enumerate(playdata):
      frame['text'] = 'T+%.3f' % ((frame['start_millis'] - offset)/ 1000.0)
      q.put(frame)

  def display_close(self):
    if self._display_thread is None:
      return

    self._display_stop = True
    self._display_thread = None

  def close(self):
    self.display_close()

  def capture_to_file(self, capture, hz, filen):
    with open(filen + '_thermal.hcap', 'w') as f:
      f.write(str(hz) + "\n")

      for frame in capture:
        t = frame['start_millis']
        arr = frame['ir']
        f.write(str(t) + "\n")
        for l in arr:
          f.write('\t'.join([str(x) for x in l]) + "\n")
        f.write("\n")

  def capture_to_img_sequence(self, capture, directory, tmin=15, tmax=45):
    hz, frames = capture

    for i, frame in enumerate(frames):
      rgb_seq = []
      for row in frame['ir']:
        for px in row:
          rgb_seq.append( self._temp_to_rgb(px, tmin, tmax) )

      im = Image.new("RGB", (16, 4))
      im.putdata(rgb_seq)
      im.save(os.path.join(directory, '{:04d}.png'.format(i)))

  def capture_to_movie(self, capture, filename, width=1920, height=480, tmin=15, tmax=45):
    hz, frames = capture
    tdir = tempfile.mkdtemp()

    self.capture_to_img_sequence(capture, tdir, tmin=tmin, tmax=tmax)

    args = [self._ffmpeg_loc, 
      "-y", 
      "-r", str(fractions.Fraction(hz)),
      "-i", os.path.join(tdir, "%04d.png"),
      "-s", "{}x{}".format(width, height),
      "-sws_flags", "neighbor",
      "-sws_dither", "none",
      filename + '_thermal.avi'
      ]

    subprocess.call(args)

  def file_to_capture(self, filen):
    capture = []
    hz = None
    with open(filen + '_thermal.hcap', 'r') as f:
      frame = {'ir':[]}

      for i, line in enumerate(f):
        if i == 0:
          hz = float(line)
          continue

        j = (i-1) % 6
        if j == 0:
          frame['start_millis'] = int(line)
        elif 0 < j < 5:
          frame['ir'].append(tuple([float(x) for x in line.split("\t")]))
        elif j == 5:
          capture.append(frame)
          frame = {'ir':[]}

    return (hz, capture)

  def capture(self, seconds, name=None, hcap=False, video=False):
    buff = []
    q = self._tcam.subscribe()
    hz = self._tcam.irhz
    tdir = tempfile.mkdtemp()

    camera = None
    visfile = name + '_visual.h264' #os.path.join(tdir, name + '_visual.h264')

    if video and self._camera is not None:
      self._camera.resolution = (1920, 1080)
      self._camera.framerate = hz
      self._camera.start_recording(visfile)

    start = time.time()
    elapsed = 0

    while elapsed <= seconds:
      elapsed = time.time() - start
      buff.append( q.get() )

    if video and self._camera is not None:
      self._camera.stop_recording()

      #args = [self._ffmpeg_loc, 
      #  "-y", 
      #  "-r", str(fractions.Fraction(hz)),
      #  "-i", visfile,
      #  "-vcodec", "copy",
      #  name + '_visual.mp4'
      #  ]

      #subprocess.call(args)

      #os.remove(visfile)


    if hcap:
      self.capture_to_file(buff, hz, name)

    return (hz, buff)
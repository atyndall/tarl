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
from PIL import Image, ImageDraw, ImageFont
import subprocess
import tempfile
import os
import os.path
import fractions
import pxdisplay
import multiprocessing
import numpy as np
import io



class BaseManager(object):
  driver = None
  build = None
  irhz = None

  tty = None
  baud = None

  hflip = True
  vflip = True

  _temps = None
  _serial_obj = None
  _queues = []

  def __init__(self, tty, hz=8, baud=115200, init=True):
    self.tty = tty
    self.baud = baud
    self.irhz = hz

    if init:
      self._serial_obj = serial.Serial(port=self.tty, baudrate=self.baud, rtscts=True, dsrdtr=True)

  def __del__(self):
    self.close()

  def _reset_and_conf(self, timers=True):
    self._serial_obj.write('r\n') # Reset the sensor
    self._serial_obj.flush()

    time.sleep(2)

    if timers:
      self._serial_obj.write('t\n') # Turn on timers
    else:
      self._serial_obj.write('o\n') # Turn on timers

    self._serial_obj.flush() 

  def _decode_packet(self, packet, splitchar="\t"):
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
          ir.append(tuple(float(x) for x in line.split(splitchar)))
      except ValueError:
        print(packet)
        print("WARNING: Could not decode corrupted packet") 
        return {}

    if self.hflip:
      ir = map(tuple, np.fliplr(ir))

    if self.vflip:
      ir = map(tuple, np.flipud(ir))

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

    if packet['irhz'] != self.irhz:
      ser.write('f{}'.format(self.irhz))
      self._update_info()

  def _wait_read_packet(self):
    ser = self._serial_obj
    line = ser.readline().decode("ascii", "ignore").strip()
    msg = []

    # Capture a whole packet
    while not line.startswith("START"):
      line = ser.readline().decode("ascii", "ignore").strip()

    while not line.startswith("STOP"): 
      msg.append(line)
      line = ser.readline().decode("ascii", "ignore").strip()

    msg.append(line)

    return msg

  def close(self):
    return

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



class Manager(BaseManager):
  _serial_thread = None
  _serial_stop = False
  _serial_ready = False

  _decode_thread = None

  _read_decode_queue = None

  def __init__(self, tty, hz=8, baud=115200):
    super(self.__class__, self).__init__(tty, hz, baud)

    self._serial_thread = threading.Thread(group=None, target=self._read_thread_run)
    self._serial_thread.daemon = True

    self._decode_thread = threading.Thread(group=None, target=self._decode_thread_run)
    self._decode_thread.daemon = True

    self._reset_and_conf(timers=True)

    self._read_decode_queue = queue.Queue()

    self._decode_thread.start()
    self._serial_thread.start()

    while not self._serial_ready: # Wait until we've populated data before continuing
      pass

  def close(self):
    self._serial_stop = True

    if self._serial_thread is not None:
      while self._serial_thread.is_alive(): # Wait for thread to terminate
        pass

  def _read_thread_run(self):
    ser = self._serial_obj
    q = self._read_decode_queue
    self._update_info()

    while True:
      msg = self._wait_read_packet()

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


class OnDemandManager(BaseManager):
  def __init__(self, tty, hz=8, baud=115200):
    super(self.__class__, self).__init__(tty, hz, baud)

    self._reset_and_conf(timers=False)

    self._update_info()

  def close(self):
    self._serial_obj.close()

  def capture(self):
    self._serial_obj.write('p') # Capture frame manually
    self._serial_obj.flush()

    msg = self._wait_read_packet()  
    dpct = self._decode_packet(msg)

    if 'ir' in dpct:
      self._temps = dpct

      for q in self._queues:
        q.put(self.get_temps())

    return dpct



class ManagerPlaybackEmulator(BaseManager):
  _playback_data = None

  _pb_thread = None
  _pb_stop = False
  _pb_len = 0

  _i = 0

  def __init__(self, playback_data=None):
    if playback_data is not None:
      self.irhz, self._playback_data = playback_data
      self._pb_len = len(self._playback_data)

    self.driver = "Playback"
    self.build = "1"

  def set_playback_data(self, playback_data):
    self.stop()
    self.irhz, self._playback_data = playback_data
    self._pb_len = len(self._playback_data)

  def close(self):
    return

  def start(self):
    if self._pb_thread is None:
      self._pb_stop = False
      self._pb_thread = threading.Thread(group=None, target=self._pb_thread_run)
      self._pb_thread.daemon = True
      self._pb_thread.start()

  def pause(self):
    self._pb_stop = True

    while self._pb_thread is not None and self._pb_thread.is_alive():
      pass

    self._pb_thread = None

  def stop(self):
    self._pb_stop = True

    while self._pb_thread is not None and self._pb_thread.is_alive():
      pass

    self._pb_thread = None
    self._i = 0

  def get_temps(self):
    return self._playback_data[self._i]

  def _pb_thread_run(self):
    while True:
      if self._pb_stop:
        return

      for q in self._queues:
        q.put(self._playback_data[self._i])

      time.sleep(1.0/float(self.irhz))

      self._i += 1

      if self._i >= self._pb_len:
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
        motion = frame['movement']
        arr = frame['ir']
        f.write(str(t) + "\n")
        f.write(str(motion) + "\n")
        for l in arr:
          f.write('\t'.join([str(x) for x in l]) + "\n")
        f.write("\n")

  def capture_to_img_sequence(self, capture, directory, tmin=15, tmax=45, text=True):
    hz, frames = capture
    pxwidth = 120
    print(directory)

    for i, frame in enumerate(frames):
      im = Image.new("RGB", (1920, 480))
      draw = ImageDraw.Draw(im)
      font = ImageFont.truetype("arial.ttf", 35)

      for k, row in enumerate(frame['ir']):
        for j, px in enumerate(row):
          rgb = pxdisplay.temp_to_rgb(px, tmin, tmax)

          x = k*pxwidth
          y = j*pxwidth

          coords = (y, x, y+pxwidth+1, x+pxwidth+1)

          draw.rectangle(coords, fill=rgb)

          if text:
            draw.text([y+20,x+(pxwidth/2-20)], str(px), fill=(255,255,255), font=font)

      im.save(os.path.join(directory, '{:09d}.png'.format(i)))

  def capture_to_movie(self, capture, filename, width=1920, height=480, tmin=15, tmax=45):
    hz, frames = capture
    tdir = tempfile.mkdtemp()

    self.capture_to_img_sequence(capture, tdir, tmin=tmin, tmax=tmax)

    args = [self._ffmpeg_loc, 
      "-y", 
      "-r", str(fractions.Fraction(hz)),
      "-i", os.path.join(tdir, "%09d.png"),
      "-s", "{}x{}".format(width, height),
      "-sws_flags", "neighbor",
      "-sws_dither", "none",
      '-vcodec', 'qtrle', '-pix_fmt', 'rgb24',
      filename + '_thermal.mov'
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

        j = (i-1) % 7
        if j == 0:
          frame['start_millis'] = int(line)
        elif j == 1:
          frame['movement'] = bool(line)
        elif 1 < j < 6:
          frame['ir'].append(tuple([float(x) for x in line.split("\t")]))
        elif j == 6:
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

  def capture_synced(self, seconds, name, hz=2):
    cap_method = getattr(self._tcam, "capture", None)
    if not callable(cap_method):
      raise "Provided tcam class must support the capture method"

    if self._camera is None:
      raise "No picamera object provided, cannot proceed"

    camera = self._camera
    camera.resolution = (1920, 1080)

    # TODO: Currently produces black images. Need to fix.
    # Wait for analog gain to settle on a higher value than 1
    #while camera.analog_gain <= 1 or camera.digital_gain <= 1:
    #    time.sleep(1)

    # Now fix the values
    #camera.shutter_speed = camera.exposure_speed
    #camera.exposure_mode = 'off'
    #g = camera.awb_gains
    #camera.awb_mode = 'off'
    #camera.awb_gains = g

    import datetime, threading, time

    dir_name = name
    frames = seconds * hz

    buff = []
    imgbuff = [io.BytesIO() for _ in range(frames + 1)]
    fps_avg = []
    lag_avg = []

    try:
      os.mkdir(dir_name)
    except OSError:
      pass

    def trigger(next_call, i):
      if i % (hz * 3) == 0:
        print('{}/{} seconds'.format(i/hz, seconds))

      t1_start = time.time()
      camera.capture(imgbuff[i], 'jpeg', use_video_port=True)
      t1_t2 = time.time()
      buff.append(self._tcam.capture())
      t2_stop = time.time()

      sec = t2_stop - t1_start
      fps_avg.append(sec)
      lag_avg.append(t2_stop - t1_t2)

      if sec > (1.0/float(hz)):
        print('Cannot keep up with frame rate!')

      if frames == i:
        return

      th = threading.Timer( next_call - time.time(), trigger,
        args=[next_call+(1.0/float(hz)), i + 1] )
      th.start()
      th.join()

    trigger(time.time(), 0)

    print('Average time for frame capture = {} seconds'.format(sum(fps_avg)/len(fps_avg)))
    print('Average lag between camera and thermal capture = {} seconds'.format(sum(lag_avg)/len(lag_avg)))

    self.capture_to_file(buff, hz, os.path.join(dir_name, 'output'))

    for i, b in enumerate(imgbuff):
      img_name = os.path.join(dir_name, 'video-{:09d}.jpg'.format(i))
      with open(img_name, 'wb') as f:
        f.write(b.getvalue())

    return (hz, buff)
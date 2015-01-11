import serial
import copy
import Queue as queue
import time
#import picamera
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

class TempCam:
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

      if cmd == "START":
        decoded_packet['start_millis'] = long(val)
      elif cmd == "STOP":
        decoded_packet['stop_millis'] = long(val)
      else:
        try:
          ir.append(tuple(float(x) for x in line.split("\t")))
        except ValueError:
          print(packet)
          print("WARNING: Could not decode corrupted packet") 
          decoded_packet['ir'] = None
          return decoded_packet

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

class Video:
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

  def display(self, limit=0, width=100, block=False, tmin=15, tmax=45):
    self._tmin = tmin
    self._tmax = tmax
    self._limit = limit
    self._dwidth = width

    self._display_stop = False
    self._display_thread = threading.Thread(group=None, target=self._display_thread)
    self._display_thread.daemon = True
    self._display_thread.start()
    if block:
      self._display_thread.join()

  def _temp_to_rgb(self, temp, tmin, tmax):
    OLD_MIN = tmin
    OLD_MAX = tmax

    if temp < OLD_MIN:
      temp = OLD_MIN

    if temp > OLD_MAX:
      temp = OLD_MAX

    v = (temp - OLD_MIN) / (OLD_MAX - OLD_MIN)

    rgb = colorsys.hsv_to_rgb((1-v), 1, v * 0.5)

    return tuple(int(c * 255) for c in rgb)

  def _display_thread(self):
    WIDTH = self._dwidth

    q = self._tcam.subscribe_lifo()
    pygame.init()
    pygame.display.set_caption("Live IR Display")

    size = (16 * WIDTH, 4 * WIDTH)
    screen = pygame.display.set_mode(size)

    while True:
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          pygame.quit()
          return

      if self._display_stop:
        pygame.quit()
        return

      qg = q.get()
      px = qg['ir']

      lag = q.qsize()
      if lag > 0:
        print("WARNING: Dropped " + str(lag) + " frames")

      for i, row in enumerate(px):
        for j, v in enumerate(row):
          rgb = self._temp_to_rgb(v, self._tmin, self._tmax)

          x = i*WIDTH
          y = j*WIDTH

          screen.fill(rgb, (y, x, WIDTH, WIDTH))

      pygame.display.flip()

      if self._limit != 0:
        time.sleep(1.0/self._limit)

  def playback(self, file, tmin=15, tmax=45):
    self._tmin = tmin
    self._tmax = tmax

    def millis_diff(a, b):
      diff = b - a
      return (diff.days * 24 * 60 * 60 + diff.seconds) * 1000 + diff.microseconds / 1000.0

    WIDTH = 100

    hz, playdata = self.file_to_capture(file)

    pygame.init()
    pygame.display.set_caption("Playing back '{}'".format(file))

    size = (16 * WIDTH, 4 * WIDTH)
    screen = pygame.display.set_mode(size)

    background = pygame.Surface(screen.get_size())
    background = background.convert_alpha()

    font = pygame.font.Font(None, 36)

    start = datetime.datetime.now()
    offset = playdata[0]['start_millis']

    for n, frame in enumerate(playdata):
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          pygame.quit()
          return

      if self._display_stop:
        pygame.quit()
        return

      qg = frame
      px = qg['ir']

      for i, row in enumerate(px):
        for j, v in enumerate(row):
          rgb = self._temp_to_rgb(v, self._tmin, self._tmax)

          x = i*WIDTH
          y = j*WIDTH

          screen.fill(rgb, (y, x, WIDTH, WIDTH))

      timestr = 'T+%.3f' % ((qg['start_millis'] - offset)/ 1000.0)

      background.fill((0, 0, 0, 0))
      text = font.render(timestr, 1, (255,255,255))
      background.blit(text, (0,0))

      # Blit everything to the screen
      screen.blit(background, (0, 0))

      pygame.display.flip()

      time.sleep(1.0/float(hz))

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
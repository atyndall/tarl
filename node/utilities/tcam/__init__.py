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

class TempCam:
  tty = None
  baud = None
  #driver = None
  #build = None

  _serial_thread = None
  _serial_stop = False
  _serial_obj = None

  _temps = None

  _queues = []

  def __init__(self, tty, baud=115200):
    self.tty = tty
    self.baud = baud

    self._serial_obj = serial.Serial(port=self.tty, baudrate=self.baud, rtscts=True, dsrdtr=True)
    self._serial_thread = threading.Thread(group=None, target=self._thread_run)
    self._serial_thread.daemon = True
    self._serial_thread.start()

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
          print("WARNING: Could not decode corrupted packet") 
          return {}

    decoded_packet['ir'] = tuple(ir)

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

  def _thread_run(self):
    ser = self._serial_obj

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

      dpct = self._decode_packet(msg)
      if 'ir' in dpct:
        self._temps = dpct

        for q in self._queues:
          q.put(self.get_temps())

      if self._serial_stop:
        ser.close()
        return
 

class Video:
  _display_thread = None
  _display_stop = False
  _tmin = None
  _tmax = None

  _tcam = None

  def __init__(self, tcam):
    self._tcam = tcam

  def display(self, block=False, tmin=15, tmax=35):
    self._tmin = tmin
    self._tmax = tmax

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
    WIDTH = 100

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

  def playback(self, file, hz=1.0, tmin=15, tmax=35):
    self._tmin = tmin
    self._tmax = tmax

    def millis_diff(a, b):
      diff = b - a
      return (diff.days * 24 * 60 * 60 + diff.seconds) * 1000 + diff.microseconds / 1000.0

    WIDTH = 100

    playdata = self.file_to_capture(file)

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

    #while self._display_thread.is_alive(): # Wait for thread to terminate
    #  pass

    self._display_thread = None

  def close(self):
    self.display_close()

  def capture_to_file(self, capture, file):
    with open(file + '.hcap', 'w') as f:
      for frame in capture:
        t = frame['start_millis']
        arr = frame['ir']
        f.write(str(t) + "\n")
        for l in arr:
          f.write('\t'.join([str(x) for x in l]) + "\n")
        f.write("\n")

  def file_to_capture(self, file):
    capture = []
    with open(file + '.hcap', 'r') as f:
      i = 0
      frame = {'ir':[]}

      for line in f:
        j = i % 6
        if j == 0:
          frame['start_millis'] = int(line)
        elif 0 < j < 5:
          frame['ir'].append(tuple([float(x) for x in line.split("\t")]))
        elif j == 5:
          capture.append(frame)
          frame = {'ir':[]}

        i += 1

    return capture

  def capture(self, seconds, file=None, video=False):
    buffer = []
    q = self._tcam.subscribe()

    camera = None

    #if video:
    #  camera = picamera.PiCamera()
    #  camera.resolution = (1280, 720)
    #  camera.start_recording(file + '.h264')

    start = time.time()
    elapsed = 0

    while elapsed <= seconds:
      elapsed = time.time() - start
      buffer.append( q.get() )

    #if video:
    #  camera.stop_recording()

    if file is not None:
      self.capture_to_file(buffer, file)

    return buffer
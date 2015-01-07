import serial
import threading
import copy
import Queue as queue
import time
import pygame
import colorsys
#import picamera
from collections import deque

class TempCam:
  tty = None
  baud = None
  #driver = None
  #build = None

  _serial_thread = None
  _display_thread = None
  _serial_stop = False
  _display_stop = False

  _temps = None

  _queues = []

  _tmin = None
  _tmax = None

  def __init__(self, tty, baud=115200):
    self.tty = tty
    self.baud = baud
    self._serial_thread = threading.Thread(group=None, target=self._thread_run)
    self._serial_thread.start()

  def __del__(self):
    self.close() 

  def _decode_packet(self, packet):
    decoded_packet = {}

    for line in packet:
      parted = line.partition(" ")
      cmd = parted[0]
      val = parted[2]

      if cmd == "CPIX":
        #decoded_packet['cpix'] = int(val)
        pass
      elif cmd == "PTAT":
        #decoded_packet['ptat'] = int(val)
        pass
      elif cmd == "EEPROM":
        #decoded_packet['eeprom'] = list(bytearray(codecs.decode(val, 'hex_codec')))
        pass
      elif cmd == "IRRAW":
        #decoded_packet['ir_raw'] = [int(x) for x in val.split("\t")]
        pass
      elif cmd == "IRCLEAN":
        try:
          decoded_packet['ir_clean'] = [float(x) for x in val.split("\t")]
        except ValueError:
          print("WARNING: Could not decode corrupted packet") 
          return {}

    return decoded_packet

  def display(self, block=False, tmin=15, tmax=35):
    self._tmin = tmin
    self._tmax = tmax

    self._display_stop = False
    self._display_thread = threading.Thread(group=None, target=self._display_thread)
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

    q = self.subscribe_lifo()
    pygame.init()

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

      px = q.get()

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

  def display_close(self):
    if self._display_thread is None:
      return

    self._display_stop = True

    while self._display_thread.is_alive(): # Wait for thread to terminate
      pass

    self._display_thread = None

  def close(self):
    self.display_close()
    self._serial_stop = True

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

  def capture_to_file(self, capture, file):
    with open(file + '.hcap', 'w') as f:
      for t, arr in capture:
        f.write(str(t) + "\n")
        for l in arr:
          f.write('\t'.join([str(x) for x in l]) + "\n")
        f.write("\n")

  def capture(self, seconds, file=None, video=False):
    buffer = []
    q = self.subscribe()

    camera = None

    #if video:
    #  camera = picamera.PiCamera()
    #  camera.resolution = (1280, 720)
    #  camera.start_recording(file + '.h264')

    start = time.time()
    elapsed = 0

    while elapsed <= seconds:
      elapsed = time.time() - start
      buffer.append( (elapsed, q.get()) )

    #if video:
    #  camera.stop_recording()

    if file is not None:
      self.capture_to_file(buffer, file)

    return buffer

  def _thread_run(self):
    ser = serial.Serial(port=self.tty, baudrate=self.baud, rtscts=True, dsrdtr=True)

    while True:
      line = ser.readline().decode("ascii", "ignore").strip()
      msg = []

      #print(line)

      #while line != 

      #if line.startswith("DRIVER"):
      #  self.driver = line.split("DRIVER ")[1]

      #if line.startswith("BUILD"):
      #  self.build = line.split("BUILD ")[1]

      # Capture a whole packet
      while line != "START":
        line = ser.readline().decode("ascii", "ignore").strip()

      while line != "STOP":
        msg.append(line)
        line = ser.readline().decode("ascii", "ignore").strip()

      dpct = self._decode_packet(msg)
      if 'ir_clean' in dpct:
        pct = dpct['ir_clean']

        arr = (
          tuple(pct[0:16]),
          tuple(pct[16:32]),
          tuple(pct[32:48]),
          tuple(pct[48:64])
        )

        self._temps = arr

        for q in self._queues:
          q.put(self.get_temps())

      if self._serial_stop:
        ser.close()
        return
       
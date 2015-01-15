from multiprocessing import Process, Queue
import pygame
import colorsys
import time

def millis_diff(a, b):
  diff = b - a
  return (diff.days * 24 * 60 * 60 + diff.seconds) * 1000 + diff.microseconds / 1000.0

def temp_to_rgb(temp, tmin, tmax):
	OLD_MIN = tmin
	OLD_MAX = tmax

	if temp < OLD_MIN:
	  temp = OLD_MIN

	if temp > OLD_MAX:
	  temp = OLD_MAX

	v = (temp - OLD_MIN) / (OLD_MAX - OLD_MIN)

	rgb = colorsys.hsv_to_rgb((1-v), 1, v * 0.5)

	return tuple(int(c * 255) for c in rgb)

def create_pixel_display(q=None, limit=0, width=100, tmin=15, tmax=45, caption="Display"):
  if q is None:
    q = Queue()

  p = Process(target=_display_process, args=(q, caption, tmin, tmax, limit, width))
  p.daemon = True
  p.start()

  return (q, p)

def _display_process(q, caption, tmin, tmax, limit, pxwidth):
  pygame.init()
  pygame.display.set_caption(caption)

  size = (16 * pxwidth, 4 * pxwidth)
  screen = pygame.display.set_mode(size)

  background = pygame.Surface(screen.get_size())
  background = background.convert_alpha()

  font = pygame.font.Font(None, 36)

  while True:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
        return

    # Keep the event loop running so the windows don't freeze without data
    try:
      qg = q.get(True, 0.3)
    except:
      continue

    px = qg['ir']

    lag = q.qsize()
    if lag > 0:
      print("WARNING: Dropped " + str(lag) + " frames")

    for i, row in enumerate(px):
      for j, v in enumerate(row):
        rgb = temp_to_rgb(v, tmin, tmax)

        x = i*pxwidth
        y = j*pxwidth

        screen.fill(rgb, (y, x, pxwidth, pxwidth))

    if 'text' in qg:
      background.fill((0, 0, 0, 0))
      text = font.render(qg['text'], 1, (255,255,255))
      background.blit(text, (0,0))

      # Blit everything to the screen
      screen.blit(background, (0, 0))

    pygame.display.flip()

    if limit != 0:
      time.sleep(1.0/float(limit))
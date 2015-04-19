import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from thinglib import *

import time
import pygame
from multiprocessing import Process, freeze_support, Queue
from PIL import Image
from pygame.locals import *

key_transform = {
  K_0: 0,
  K_KP0: 0,
  K_1: 1,
  K_KP1: 1,
  K_2: 2,
  K_KP2: 2,
  K_3: 3,
  K_KP3: 3,
  K_4: 4,
  K_KP4: 4,
  K_5: 5,
  K_KP5: 5,
  K_6: 6,
  K_KP6: 6,
  K_7: 7,
  K_KP7: 7,
  K_8: 8,
  K_KP8: 8,
  K_9: 9,
  K_KP9: 9,
}

if __name__ == '__main__':
  freeze_support()
  tcv = cam.Visualizer(None)
  q = Queue()
  feat = features.Features(q, 1)
  caps = tcv.file_to_capture(sys.argv[1]+'/output')[1]
  imgs = [f for f in os.listdir(sys.argv[1]+'/') if os.path.isfile(sys.argv[1]+'/'+f) and f.startswith('video-') and f.endswith('.jpg')]

  truth = []

  mainw = 1920 / 2
  mainh = 1348 / 2
  mainsize = (mainw,mainh)

  pygame.init()

  mainscreen = pygame.display.set_mode(mainsize)
  font = pygame.font.Font(None, 36)

  w = 1920
  h = 1348
  size = (w,h)

  screen = pygame.Surface(size)

  background = pygame.Surface(size)
  background = background.convert_alpha()

  pxwidth = 67

  for seq, (cap, img) in enumerate(zip(caps, imgs)):
    cap['movement'] = False
    if seq > int(sys.argv[2]):
      cap['movement'] = True

    q.put(cap)

    screen.fill((0,0,0,0))

    for i, row in enumerate(cap['ir']):
      for j, v in enumerate(row):
        rgb = pxdisplay.temp_to_rgb(v, 5, 40)

        x = i*pxwidth
        y = j*pxwidth

        screen.fill(rgb, (y, x, pxwidth, pxwidth))

    img = pygame.image.load(sys.argv[1]+'/'+img)
    img = pygame.transform.flip(img, True, False)
    screen.blit(img, (0, 268))

    feats = feat.get_features()

    background.fill((0, 0, 0, 0))
    text = font.render(str(seq), 1, (255,255,255))
    text2 = font.render("Num Active: " + str(feats[0]), 1, (255,255,255))
    text3 = font.render("Num Connected: " + str(feats[1]), 1, (255,255,255))
    text4 = font.render("Size Connected: " + str(feats[2]), 1, (255,255,255))

    background.blit(text, (1850,0))
    background.blit(text2, (1500,40))
    background.blit(text3, (1500,80))
    background.blit(text4, (1500,120))

    # Blit everything to the screen
    screen.blit(background, (0, 0))

    pygame.transform.scale(screen, (mainw,mainh), mainscreen)

    pygame.display.flip()

    pygame.event.clear()
    while True:
      event = pygame.event.wait()
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
      elif event.type == KEYDOWN:
        if event.key in key_transform.keys():
          number = key_transform[event.key]
          truth.append(number)
          break

  tstr = '\n'.join(map(str,truth))

  with open(sys.argv[1]+'/'+'truth', 'w') as f:
    f.write(tstr)

  tcv.close()
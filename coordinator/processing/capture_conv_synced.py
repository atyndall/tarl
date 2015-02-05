from thinglib import *
import time
import sys
import fractions
import subprocess
import os

#ffmpeg_loc="/Users/atyndall/Dropbox/ash-honours/macffmpeg/ffmpeg"
ffmpeg_loc="d:\\Users\\atyndall\\Documents\\ffmpeg-20140703-git-1265247-win64-static\\bin\\ffmpeg.exe"

tcv = cam.Visualizer(ffmpeg_loc=ffmpeg_loc)

name = sys.argv[1]

cap = tcv.file_to_capture(os.path.join(name, 'output'))

tcv.capture_to_movie(cap, name)

args = [ffmpeg_loc, 
  "-y", 
  "-r", str(fractions.Fraction(cap[0])),
  "-i", os.path.join(name, "video-%09d.jpg"),
  "-s", "{}x{}".format(1920, 1080),
  "-sws_flags", "neighbor",
  "-sws_dither", "none",
  '-vcodec', 'qtrle', '-pix_fmt', 'rgb24',
  filename + '_thermal.mov'
  ]

#tcv.capture_to_movie(b, 'cap1')

tcv.close()
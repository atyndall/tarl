from thinglib import *
import time
import sys
import fractions
import subprocess

#ffmpeg_loc="/Users/atyndall/Dropbox/ash-honours/macffmpeg/ffmpeg"
ffmpeg_loc="d:\\Users\\atyndall\\Documents\\ffmpeg-20140703-git-1265247-win64-static\\bin\\ffmpeg.exe"

tcv = cam.Visualizer(ffmpeg_loc=ffmpeg_loc)

name = sys.argv[1]

cap = tcv.file_to_capture(name)

tcv.capture_to_movie(cap, name)

args = [ffmpeg_loc, 
  "-y", 
  "-r", str(fractions.Fraction(cap[0])),
  "-i", sys.argv[1] + "_visual.h264",
  "-vcodec", "copy",
  name + '_visual.mp4'
  ]

subprocess.call(args)

#tcv.capture_to_movie(b, 'cap1')

tcv.close()
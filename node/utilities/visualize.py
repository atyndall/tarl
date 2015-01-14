import tcam
from multiprocessing import Process, freeze_support

if __name__ == '__main__':
  freeze_support()
  tc = tcam.TempCam("COM4")
  tcv = tcam.Video(tc)
  tcv.display(block=True, tmin=15, tmax=35)
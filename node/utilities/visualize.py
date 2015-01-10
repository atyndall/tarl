import tcam

tc = tcam.TempCam("COM4")
tcv = tcam.Video(tc)
tcv.display(block=True, tmin=15, tmax=40)
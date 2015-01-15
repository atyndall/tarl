import tcam

tcv = tcam.Video(ffmpeg_loc="d:\Users\\atyndall\Documents\\ffmpeg-20140703-git-1265247-win64-static\\bin\\ffmpeg.exe")
cap = tcv.file_to_capture('cap1')

tcv.capture_to_movie(cap, 'cap1_temp2.avi')
from ImagePipeline import *
import cv2

p = ImagePipelineParams()

p.video_filename =  '../test_videos/project2/project_video.mp4'

p.show_trackbars = True

cv2.namedWindow(p.trackbar_window)

ip = ImagePipeline(p)

print("image size: {}".format(ip.GetFrameSize()))

ip.Run(lambda x: x)


import threading
import time
import video_capture_gazebo
import feature_match_ardu
import traceback
import signal
import sys
import cv2 as cv
import numpy as np

baseImg = "satellite_image.png"

class main:
	def __init__(self):
		self.lat = 0
		self.lon = 0
		self.alt = 0
		self.alt_ab_ter = 0
		self.video = video_capture_gazebo.Cam_stream()       # Get video frame
		self.feature = feature_match_ardu.Image_Process()
		# t1 = threading.Thread(target=self.video.setup)
		# t1.start()
		self.img1 = []
		self.sift = None
		self.kp = None
		self.des = None
		self.sr_pt = []
		self.frame = []

	def sift_base(self):
		self.img1 = cv.imread(baseImg,0)
		# self.img1 = cv.resize(self.img1, (600,600))
		self.sift = cv.SIFT_create(nfeatures = 10000)
		self.kp, self.des = self.sift.detectAndCompute(self.img1,None)
	def getComp(self):
		while True:
			# print(len(self.frame))
			# self.feature.compParam("satellite_image.png", self.frame)
			if not self.video.frame_available():
				continue
			self.frame = self.video.frame
			self.frame = self.feature.compParam("satellite_image.png", self.frame,(self.kp,self.des))
			if type(self.frame) != type(None):
				cv.imshow('frame', self.frame)

				if cv.waitKey(1) & 0xFF == ord('q'):
					break

	def vid_sleep(self):
		time.sleep(0.3)




m = main()
m.sift_base()
m.getComp()

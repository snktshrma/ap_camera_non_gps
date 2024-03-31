import threading
import time
import video_capture_gazebo
import feature_match_ardu
from pymavlink import mavutil
import traceback
import signal
import sys


the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))


class main:
	def _init_(self):
		self.lat = 0
		self.lon = 0
		self.alt = 0
		self.alt_ab_ter = 0
		self.feature = feature_match_ardu.Image_Process()
		self.video = video_capture_gazebo.Cam_stream()       # Get video frame
		t1 = threading.Thread(target=self.video.setup)
		t2 = threading.Thread(target=self.get_gps)
		t1.start()
		t2.start()

		the_connection.mav.command_long_send(
			the_connection.target_system, the_connection.target_component,
			mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
			33,
			1e6 / 10,
			0, 0, 0, 0,
			0,
		)




	def get_gps(self):
		msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
		if msg:
			self.lat = msg.lat
			self.lon = msg.lon
			self.alt = msg.alt
			self.alt_ab_ter = msg.relative_alt/1000
		#return {self.lat, self.lon}

	def vid_sleep(self):
		time.sleep(0.3)


	def exec(self):
		self.video.check = 1
		metadata = {"Latitude": str(self.lat), "Longitude": str(self.lat), "AMSL": str(self.alt), "Terr_Alt": str(self.alt_ab_ter)}
		self.feature.modifyExif(f"data{self.video.n}.jpeg",metadata)



m = main()

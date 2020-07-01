# Message for holding an image and metadata for the NDVI cameras.
Header header			# stamp is the time at which the data was valid.
				# frame_id is the location of the camera.

string time_reference		# The name of the reference clock for this message's timestamp.

sensor_msgs/Image image      	# Standard ROS image

sensor_msgs/CameraInfo info  	# Standard ROS camera info

float32 shutter              	# Time duration (seconds) the shutter was open.  Given by the FPGA timesync packet.  Negative shutter is an uninitialized (invalid) value.

float32 gain		     	# Gain (in dB) applied to the image.

uint16 white_balance_blue    	# White balance in blue.

uint16 white_balance_red     	# White balance in red.

float32 temperature          	# Temperature reported by the camera (Celsius).
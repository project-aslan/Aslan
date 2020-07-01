# Message for the time synchronization packets for the WFOV cameras.

Header header			# stamp is the time at which the trigger was received.

string time_reference		# The name of the reference clock for this message's timestamp.

time trigger_time		# The trigger time at which the data was valid.

string trigger_time_reference	# The name of the reference clock for this message's trigger timestamp.

uint32 shutter			# Time duration (microseconds) the shutter was open.

uint32 id			# Camera ID Number

uint32 trigger_seq		# Number of triggers sent to this camera ID number
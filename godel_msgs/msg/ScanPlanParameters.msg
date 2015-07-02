# Path Parameters
float64 scan_width           	# (m) actual width of scan tool
float64 margin                  # (m) additional distance to leave near surface boundaries
float64 overlap                 # (m) overlap distance between adjacent paths
float64 approach_distance		# (m) approach distance
float64 traverse_spd            # (m/s) Speed to travel while at traverse height

# QA
string quality_metric			# name associated with scoring metric (ex. rms, ra)
float64 window_width			# (m) width of window to score scan
float64 min_qa_value			# minimum value
float64 max_qa_value			# max value

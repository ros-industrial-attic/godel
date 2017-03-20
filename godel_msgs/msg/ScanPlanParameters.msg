# Path Parameters
float64 scan_width            # (m) actual width of scanner's beam
float64 margin                # (m) additional distance to leave near surface boundaries
float64 overlap               # (m) overlap distance between adjacent paths
float64 approach_distance     # (m) approach distance
float64 traverse_spd          # (m/s) Speed to travel while at traverse height

# Misc
float64 z_adjust              # (m) height adjustment along surface normal to adjust for different tools

# QA Methods
int32 METHOD_RMS=0

# QA Parameters
# These values, combined with your choice of scoring metric, are used to colorize
# points in individual line scans. 'out-of-spec' means that the metric (say RMS) over
# the 'window_width' exceeds the 'max_qa_value'. In this case, the associated point
# will be colored bright red. The 'min_qa_value' field is used to scale the color
# of points with scores in between the min and max values.
int32 quality_metric          # line scanner scoring metric (see QA Methods enumeration)
float64 window_width          # (m) width of window to score scan
float64 min_qa_value          # minimum value (units are function of QA method)
float64 max_qa_value          # max value (units are function of QA method)

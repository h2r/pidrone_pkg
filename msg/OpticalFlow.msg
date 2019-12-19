# Message to be used by the optical flow code to republish them for
# displaying in the web dashboard.

Header header

uint32 rows         # a fraction of the corresponding image's height
uint32 columns       # a fraction of the corresponding image's width

float64[] flow_x      # x components of the optical flow field stored in a 1D array, the index of a vector at (x, y) is (y * width + x)
float64[] flow_y      # the corresponding y components of the vectors
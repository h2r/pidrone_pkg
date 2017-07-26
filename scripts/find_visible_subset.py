import numpy as np


ux = 0.003921216794351938 
uy = 0.003921216794351938
A = np.array([[ux, 0, 0, 0], [0, uy, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
RT_intrinsic = np.array([[-1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
camera_corners = [(0,0), (320,0), (320,240),(0,240)]


def pixel_to_global(xy, RT, d, A=A, RT_intrinsic=RT_intrinsic, wh=(320,240)):
	# normalize the pixel coordinates so they are centered around the middle of the image
    npx = xy[0] - 0.5*wh[0]
    npy = xy[1] - 0.5*wh[1]
    # place the pixel in 3D space in the camera's coordinate frame
    Ax = np.dot(A, np.array([[npx * d, npy * d, d, 1]]).T)
    # take the pixel back to 3D in the global coordinate frame
    return  np.dot(RT, np.dot(RT_intrinsic, Ax))

def find_bounding_box_global(RT, d, box_scale = 1.0, w=320, h=240):
	center = np.array([w,h])/2.0 # pixel coordinates of the camera center
	global_corners = []	
	camera_corners = [np.array([0,0]),
					  np.array([w,0]),
					  np.array([w,h]),
					  np.array([0,h])]

	for corner in camera_corners:
		corner = (corner - center) * scale + center # rescale the camera corners to grow the box
		global_corners.append(pixel_to_global(corner, RT, d)) # find the corner in global coords

	return global_corners

def kp_in_box(corners, kp):
	all_positive = True
	all_negative = True

	for i in range(4):
		i_next = (i+1)%4 # loop the next corner index
		edge_vector = np.array([corners[i_next][0] - corners[i][0], corners[i_next][1] - corners[i][1]])
		point_vector = np.array([kp[0] - corners[i][0], kp[1], corners[i][1]])

		is_positive = np.cross(edge_vector,point_vector) > 0 # determine if the cross product is positive
		all_positive = all_positive and is_positive			 # update the truth statements
		all_negative = all_negative and not is_positive

	return all_positive or all_negative	# if the crosses are all pos or all neg, it's in the box



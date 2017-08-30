'''pseudocode for estimateRigidTransform slam thingy'''

# this allows us to decide when to add a new frame to the ref set
overlap_thresh = 0.8

# we start off with a frame taken at a known altitude facing straight down. 
# this allows us to set this position to zero and know the corners of the frame
# on the ground
start_img = takePicture()
start_corners = [
	(-.4, -.3),
	(.4, -.3),
	(.4, .3),
	(-.4, .3)
]
pos = (0,1,0)

start_frame = {
	'img': start_img,
	'corners': start_corners
}

ref_set = [start_frame] # start off our set of reference images with this frame

# now we run the algorithm in a loop
while True:
	update_pos_estimate(pos) # using velocity+orientation+input and time elapsed since prev frame

	frame = {
		'img': takePicture(),
		'corners': estimate_frame_corners(pos) # gives the corners of a frame based on pos
	}

	max_overlap = (0, None) # so we can keep of the ref frame with the most overlap
	rt = None # will store an affine transform if we get one

	# go through each frame in the reference set of frames
	for ref_frame in ref_set:
		overlap = estiamte_overlap(frame, ref_frame)

		# this is optional:
		# above some threshold for image overlap we might be fairly confident that the
		# rigid transform calculation will succeed, so we don't have to go through the 
		# rest of the ref set, we can just try the image transform 
		if overlap > overlap_thresh:
			rt = cv2.estimateRigidTransform(frame['img'], ref_frame['img'], fullAffine=False)
			if rt: 
				max_overlap = (overlap, ref_frame)
				break 

		# store the max overlap and corresponding the ref_frame
		if overlap > max_overlap:
			max_overlap = (overlap, ref_frame)

	# if we got a frame above the overlap_thresh, then we don't need to store it because it's similar
	# to existing frames in the set. If not, then we need to run rigid transform on the frame
	# with the most overlap and we want to save the new frame in the ref_set
	if rt: add_frame_to_ref_set = False 
	else: 
		rt = cv2.estimateRigidTransform(frame['img'], max_overlap[1]['img'], fullAffine=False) 
		add_frame_to_ref_set = True

	if rt:
		refine_pos_estimate(rt, max_overlap[1]) # use the rigid transform and ref_frame to figure out where we are
		if add_frame_to_ref_set:
			refine_frame_corners(frame, pos) # use the improved position estimate to refine the corners of the frame
			ref_set.insert(0, frame)
	else:
		print 'SKIPPED. AHHHHH'




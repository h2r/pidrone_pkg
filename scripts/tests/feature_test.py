import cv2
import numpy as np
import math
import time


def drawMatches(img1, kp1, img2, kp2, matches, mask):
    """
    Implementation of cv2.drawMatches as OpenCV 2.4.9
    does not have this function available but it's supported in
    OpenCV 3.0.0
    This function takes in two images with their associated
    keypoints, as well as a list of DMatch data structure (matches)
    that contains which keypoints matched in which images.
    An image will be produced where a montage is shown with
    the first image followed by the second image beside it.
    Keypoints are delineated with circles, while lines are connected
    between matching keypoints.
    img1,img2 - Grayscale images
    kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint
              detection algorithms
    matches - A list of matches of corresponding keypoints through any
              OpenCV keypoint matching algorithm
    """

    # Create a new output image that concatenates the two images together
    # (a.k.a) a montage
    rows1 = img1.shape[0]
    cols1 = img1.shape[1]
    rows2 = img2.shape[0]
    cols2 = img2.shape[1]

    # Create the output image
    # The rows of the output are the largest between the two images
    # and the columns are simply the sum of the two together
    # The intent is to make this a colour image, so make this 3 channels
    out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

    # Place the first image to the left
    out[:rows1,:cols1] = np.dstack([img1[:, :, 0], img1[:, :, 1], img1[:, :, 2]])

    # Place the next image to the right of it
    out[:rows2,cols1:] = np.dstack([img2[:, :, 0], img2[:, :, 1], img2[:, :, 2]])

    # For each pair of points we have between both images
    # draw circles, then connect a line between them
    for i, mat in enumerate(matches):

        if mask is not None and mask[i] == 1:
            # Get the matching keypoints for each of the images
            img1_idx = mat.queryIdx
            img2_idx = mat.trainIdx

            # x - columns
            # y - rows
            (x1,y1) = kp1[img1_idx].pt
            (x2,y2) = kp2[img2_idx].pt

            # Draw a small circle at both co-ordinates
            # radius 4
            # colour blue
            # thickness = 1
            cv2.circle(out, (int(x1), int(y1)), 3, (0, 255, 0), 2)
            cv2.circle(out, (int(x2)+cols1, int(y2)), 3, (0, 255, 0), 2)

            # Draw a line in between the two points
            # thickness = 1
            # colour blue
            cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (0, 255, 0), 1)

    # Show the image
    cv2.imshow('Matched Features', out)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Also return the image if you'd like a copy
    return out


MIN_MATCH_COUNT = 5
MAP_PIXEL_WIDTH = 2048  # in pixel
MAP_PIXEL_HEIGHT = 1616
MAP_REAL_WIDTH = 1.40  # in meter
MAP_REAL_HEIGHT = 1.07
METER_TO_PIXEL = (MAP_PIXEL_WIDTH / MAP_REAL_WIDTH + MAP_PIXEL_HEIGHT / MAP_REAL_HEIGHT) / 2.

angle_x = 0.0
angle_y = 0.0
z = 0.0
z = math.sqrt(z**2 / (1 + math.tan(angle_x)**2 + math.tan(angle_y)**2))

offset_x = np.tan(angle_x) * z
offset_y = np.tan(angle_y) * z

# img1 = cv2.imread('/Users/baichuanhuang/Downloads/IMG_0131.JPG')
# img1 = cv2.resize(img1, (0,0), fx=0.2, fy=0.2)
img1 = cv2.imread('/Users/baichuanhuang/Downloads/img2.JPG')
img2 = cv2.imread('map.JPG')
# img2 = cv2.imread('/Users/baichuanhuang/Downloads/IMG_0129.JPG')
# img2 = cv2.resize(img2, (0, 0), fx=0.2, fy=0.2)

# # Just estimateRigidTransform
# start_time = time.time()
# transform = cv2.estimateRigidTransform(img1, img2, False)
# print("--- %s seconds ---" % (time.time() - start_time))
# print transform

# ORB FLANN knnMatch find object
detector = cv2.ORB(nfeatures=150, scoreType=cv2.ORB_FAST_SCORE)
detector2 = cv2.ORB(nfeatures=500, scoreType=cv2.ORB_FAST_SCORE)
detector3 = cv2.GridAdaptedFeatureDetector(detector2, maxTotalKeypoints=10000, gridCols=4, gridRows=4)


index_params = dict(algorithm=6,
                    table_number=6,  # 12
                    key_size=12,     # 20
                    multi_probe_level=1)  # 2
search_params = dict(checks=50)   # or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params, search_params)

kp2 = detector3.detect(img2, None)
kp2, des2 = detector2.compute(img2, kp2)
img2 = cv2.drawKeypoints(img2, kp2, color=(0, 255, 0), flags=0)
print len(kp2), len(des2)

start_time1 = time.time()
kp1, des1 = detector.detectAndCompute(img1, None)
img1 = cv2.drawKeypoints(img1, kp1, color=(0, 255, 0), flags=0)
end_time1 = time.time()
print len(kp1), len(des1)

start_time2 = time.time()
for _ in range(50):
    matches = flann.knnMatch(des1, des2, k=2)
    good = []
    for match in matches:
        if len(match) > 1 and match[0].distance < 0.7 * match[1].distance:
            good.append(match[0])
end_time2 = time.time()
print len(good)

if len(good) > MIN_MATCH_COUNT:
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    matchesMask = mask.ravel().tolist()

    h, w, _ = img1.shape
    pts = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
    dst = cv2.perspectiveTransform(pts, M)

    cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.CV_AA)

    center = np.float32([(w-1)/2, (h-1)/2]).reshape(-1, 1, 2)
    dcenter = cv2.perspectiveTransform(center, M)
    cv2.circle(img2, (int(dcenter[0][0][0]), int(dcenter[0][0][1])), 3, (255, 0, 0), 2)
    dcenter = [dcenter[0][0][0] / float(METER_TO_PIXEL), (MAP_PIXEL_HEIGHT - dcenter[0][0][1]) / float(METER_TO_PIXEL)]
    print dcenter

    start_time3 = time.time()
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
    dcenter = cv2.transform(center, transform)
    cv2.circle(img2, (int(dcenter[0][0][0]), int(dcenter[0][0][1])), 3, (0, 0, 255), 2)
    dcenter = [dcenter[0][0][0] / float(METER_TO_PIXEL), (MAP_PIXEL_HEIGHT - dcenter[0][0][1]) / float(METER_TO_PIXEL)]
    end_time3 = time.time()
    print transform
    print dcenter
    yaw = np.arctan2(transform[1, 0], transform[0, 0])
    global_offset_x = math.cos(yaw) * offset_x + math.sin(yaw) * offset_y
    global_offset_y = math.sin(yaw) * offset_x + math.cos(yaw) * offset_y
    dcenter = [dcenter[0] + global_offset_x, dcenter[1] + global_offset_y]
    print dcenter
    print yaw

else:
    print "Not enough matches are found - %d/%d" % (len(good), MIN_MATCH_COUNT)
    matchesMask = None

out = drawMatches(img1, kp1, img2, kp2, good, matchesMask)

print "compute feature", end_time1 - start_time1
print "find match", end_time2 - start_time2
print "transform", end_time3 - start_time3

# cv2.imwrite('mapping44.jpg', out)

# # ORB BF knnMatch
# detector = cv2.ORB(nfeatures=120, scoreType=cv2.ORB_FAST_SCORE)
#
# bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#
# kp1, des1 = detector.detectAndCompute(img1, None)
#
# start_time = time.time()
# kp2, des2 = detector.detectAndCompute(img2, None)
# matches = bf.knnMatch(des1, des2, k=1)
# print len(matches)
#
# good = []
# for match in matches:
#     if len(match) == 1:
#         good.append(match[0])
#
# src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
# dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
#
# transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
# print("--- %s seconds ---" % (time.time() - start_time))
# print transform

# # ORB flann match
# detector = cv2.ORB_create()
#
# index_params= dict(algorithm = 6,
#                    table_number = 6, # 12
#                    key_size = 12,     # 20
#                    multi_probe_level = 1) #2search_params = dict(checks=50)
# search_params = dict(checks=50)   # or pass empty dictionary
#
# flann = cv2.FlannBasedMatcher(index_params, search_params)
#
# kp1, des1 = detector.detectAndCompute(img1, None)
#
# start_time = time.time()
# kp2, des2 = detector.detectAndCompute(img2, None)
# matches = flann.match(des1,des2)
# matches = sorted(matches, key = lambda x:x.distance)
#
# src_pts = np.float32([ kp1[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
# dst_pts = np.float32([ kp2[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)
#
# transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
# print("--- %s seconds ---" % (time.time() - start_time))
# print transform

# # SIFT
# detector = cv2.xfeatures2d.SIFT_create()
#
# FLANN_INDEX_KDTREE = 0
# index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
# search_params = dict(checks=50)   # or pass empty dictionary
#
# flann = cv2.FlannBasedMatcher(index_params, search_params)
#
# kp1, des1 = detector.detectAndCompute(img1, None)
#
# start_time = time.time()
# kp2, des2 = detector.detectAndCompute(img2, None)
# matches = flann.knnMatch(des1,des2,k=2)
#
# good = []
# for m, n in matches:
#     if m.distance < 0.7*n.distance:
#         good.append(m)
#
# src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
# dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
#
# transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
# print("--- %s seconds ---" % (time.time() - start_time))
# print transform

# # SURF
# detector = cv2.xfeatures2d.SURF_create()
#
# FLANN_INDEX_KDTREE = 0
# index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
# search_params = dict(checks=50)   # or pass empty dictionary
#
# flann = cv2.FlannBasedMatcher(index_params, search_params)
#
# kp1, des1 = detector.detectAndCompute(img1, None)
#
# start_time = time.time()
# kp2, des2 = detector.detectAndCompute(img2, None)
# matches = flann.knnMatch(des1,des2,k=2)
#
# good = []
# for m, n in matches:
#     if m.distance < 0.7*n.distance:
#         good.append(m)
#
# src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
# dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
#
# transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
# print("--- %s seconds ---" % (time.time() - start_time))
# print transform
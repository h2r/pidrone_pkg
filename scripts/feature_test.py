import cv2
import numpy as np
import time


img1 = cv2.imread('img1.jpg')
img2 = cv2.imread('img2.jpg')

# just estimateRigidTransform
start_time = time.time()
transform = cv2.estimateRigidTransform(img1, img2, False)
print("--- %s seconds ---" % (time.time() - start_time))
print transform

# ORB flann knnMatch
detector = cv2.ORB_create(nfeatures=200)

index_params= dict(algorithm = 6,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2search_params = dict(checks=50)
search_params = dict(checks=50)   # or pass empty dictionary

flann = cv2.FlannBasedMatcher(index_params, search_params)

kp1, des1 = detector.detectAndCompute(img1, None)

for i in range(5):
    start_time = time.time()
    kp2, des2 = detector.detectAndCompute(img2, None)
    matches = flann.knnMatch(des1,des2,k=2)

    good = []

    for match in matches:
        if len(match) > 1 and match[0].distance < 0.7*match[1].distance:
            good.append(match[0])

    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
    print("--- %s seconds ---" % (time.time() - start_time))
    print transform

# ORB BF match
detector = cv2.ORB_create()

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

kp1, des1 = detector.detectAndCompute(img1, None)

start_time = time.time()
kp2, des2 = detector.detectAndCompute(img2, None)
matches = bf.match(des1,des2)
matches = sorted(matches, key = lambda x:x.distance)

src_pts = np.float32([ kp1[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
dst_pts = np.float32([ kp2[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)

transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
print("--- %s seconds ---" % (time.time() - start_time))
print transform


# ORB flann match
detector = cv2.ORB_create()

index_params= dict(algorithm = 6,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2search_params = dict(checks=50)
search_params = dict(checks=50)   # or pass empty dictionary

flann = cv2.FlannBasedMatcher(index_params, search_params)

kp1, des1 = detector.detectAndCompute(img1, None)

start_time = time.time()
kp2, des2 = detector.detectAndCompute(img2, None)
matches = flann.match(des1,des2)
matches = sorted(matches, key = lambda x:x.distance)

src_pts = np.float32([ kp1[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
dst_pts = np.float32([ kp2[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)

transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
print("--- %s seconds ---" % (time.time() - start_time))
print transform

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
#
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
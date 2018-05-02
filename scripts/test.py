import cv2
import numpy as np

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


CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_CENTER = np.float32([(CAMERA_WIDTH - 1) / 2., (CAMERA_HEIGHT - 1) / 2.]).reshape(-1, 1, 2)
MAP_PIXEL_WIDTH = 2048  # in pixel
MAP_PIXEL_HEIGHT = 1616
MAP_REAL_WIDTH = 1.40  # in meter
MAP_REAL_HEIGHT = 1.07
METER_TO_PIXEL = (MAP_PIXEL_WIDTH / MAP_REAL_WIDTH + MAP_PIXEL_HEIGHT / MAP_REAL_HEIGHT) / 2.  # assume a pixel in x and y has the same length

# ----- parameters for features -----
ORB_GRID_SIZE_X = 4
ORB_GRID_SIZE_Y = 3
MAP_GRID_SIZE_X = ORB_GRID_SIZE_X * 3
MAP_GRID_SIZE_Y = ORB_GRID_SIZE_Y * 3
CELL_X = float(MAP_PIXEL_WIDTH) / MAP_GRID_SIZE_X
CELL_Y = float(MAP_PIXEL_HEIGHT) / MAP_GRID_SIZE_Y

MIN_MATCH_COUNT = 10


# class Particle(object):
#     """
#     each particle holds its estimation of the robot's location and weight of particle
#     z is currently not used
#     """
#
#     def __init__(self, x, y, z, yaw):
#         self.position = [x, y, z, yaw]
#         self.weight = 0.1
#
#     def __str__(self):
#         return str(self.position) + str(self.weight)
#
#     def __repr__(self):
#         return str(self.position) + str(self.weight)
#
#
# abc = []
# abc.append(Particle(1, 2, 3, 4))
# abc.append(Particle(2, 3, 4, 5))
# abc.append(Particle(3, 4, 5, 6))
# print abc

def norm_pdf(x, mu, sigma):
    u = (x - mu) / float(abs(sigma))
    y = (1 / (np.sqrt(2 * np.pi) * abs(sigma))) * np.exp(-u*u / 2.)
    return y


def create_map(file_name):
    """
    create a feature map, extract features from each bigger cell.
    :param file_name: the image of map
    :return: a list of center of bigger cells (kp and des), each bigger cell is a 3 by 3 grid (9 cells).
    """

    # read image and extract features
    image = cv2.imread(file_name)
    # the edgeThreshold and patchSize can be tuned if the gap between cell is too large
    detector = cv2.ORB(nfeatures=500, scoreType=cv2.ORB_FAST_SCORE, scaleFactor=1.4)
    maxTotalKeypoints = 500 * ORB_GRID_SIZE_X * ORB_GRID_SIZE_Y
    detector_grid = cv2.GridAdaptedFeatureDetector(detector, maxTotalKeypoints=maxTotalKeypoints,
                                                   gridCols=ORB_GRID_SIZE_X, gridRows=ORB_GRID_SIZE_Y)
    kp = detector_grid.detect(image, None)
    kp, des = detector.compute(image, kp)

    # rearrange kp and des into grid
    grid_kp = [[[] for y in range(MAP_GRID_SIZE_Y)] for x in range(MAP_GRID_SIZE_X)]
    grid_des = [[[] for y in range(MAP_GRID_SIZE_Y)] for x in range(MAP_GRID_SIZE_X)]
    for i in range(len(kp)):
        x = int(kp[i].pt[0] / CELL_X)
        y = MAP_GRID_SIZE_Y - 1 - int(kp[i].pt[1] / CELL_Y)
        grid_kp[x][y].append(kp[i])
        grid_des[x][y].append(des[i])

    # group every 3 by 3 grid, so we can access each center of grid and its 8 neighbors easily
    map_grid_kp = [[[] for y in range(MAP_GRID_SIZE_Y)] for x in range(MAP_GRID_SIZE_X)]
    map_grid_des = [[[] for y in range(MAP_GRID_SIZE_Y)] for x in range(MAP_GRID_SIZE_X)]
    for i in range(MAP_GRID_SIZE_X):
        for j in range(MAP_GRID_SIZE_Y):
            for k in range(-1, 2):
                for l in range(-1, 2):
                    x = i + k
                    y = j + l
                    if 0 <= x < MAP_GRID_SIZE_X and 0 <= y < MAP_GRID_SIZE_Y:
                        map_grid_kp[i][j].extend(grid_kp[x][y])
                        map_grid_des[i][j].extend(grid_des[x][y])
            map_grid_des[i][j] = np.array(map_grid_des[i][j])

    return map_grid_kp, map_grid_des


# img1 = cv2.imread('/Users/baichuanhuang/Downloads/img3.jpg')
# detector = cv2.ORB(nfeatures=100, scoreType=cv2.ORB_FAST_SCORE)
# kp1, des1 = detector.detectAndCompute(img1, None)
# img1 = cv2.drawKeypoints(img1, kp1, color=(0, 255, 0), flags=0)
#
# img2 = cv2.imread('/Users/baichuanhuang/Downloads/map.jpg')
# for i in range(MAP_GRID_SIZE_X):
#     cv2.line(img2, (i*int(CELL_X), 0), (i*int(CELL_X), MAP_PIXEL_HEIGHT), (255, 0, 0), 1, 1)
# for i in range(MAP_GRID_SIZE_Y):
#     cv2.line(img2, (0, i * int(CELL_Y)), (MAP_PIXEL_WIDTH, i * int(CELL_Y)), (255, 0, 0), 1, 1)
# kp2, des2 = create_map('/Users/baichuanhuang/Downloads/map.jpg')
# kp2 = kp2[7][1]
# des2 = des2[7][1]
# cv2.drawKeypoints(img2, kp2, img2)
#
# index_params = dict(algorithm=6,
#                     table_number=6,  # 12
#                     key_size=12,     # 20
#                     multi_probe_level=1)  # 2
# search_params = dict(checks=50)   # or pass empty dictionary
# flann = cv2.FlannBasedMatcher(index_params, search_params)
# matches = flann.knnMatch(des1, des2, k=2)
# good = []
# for match in matches:
#     if len(match) > 1 and match[0].distance < 0.7 * match[1].distance:
#         good.append(match[0])
#
# print len(good)
# if len(good) > MIN_MATCH_COUNT:
#     src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
#     dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
#
#     M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
#     matchesMask = mask.ravel().tolist()
#
#     h, w, _ = img1.shape
#     pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
#     dst = cv2.perspectiveTransform(pts, M)
#
#     cv2.polylines(img2, [np.int32(dst)], True, 255, 3, cv2.CV_AA)
#
#     center = np.float32([(w - 1) / 2, (h - 1) / 2]).reshape(-1, 1, 2)
#     dcenter = cv2.perspectiveTransform(center, M)
#     cv2.circle(img2, (int(dcenter[0][0][0]), int(dcenter[0][0][1])), 3, (255, 0, 0), 2)
#     dcenter = [dcenter[0][0][0] / float(METER_TO_PIXEL), (MAP_PIXEL_HEIGHT - dcenter[0][0][1]) / float(METER_TO_PIXEL)]
#     print dcenter
#
#     transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
#     dcenter = cv2.transform(center, transform)
#     cv2.circle(img2, (int(dcenter[0][0][0]), int(dcenter[0][0][1])), 3, (0, 0, 255), 2)
#     dcenter = [dcenter[0][0][0] / float(METER_TO_PIXEL), (MAP_PIXEL_HEIGHT - dcenter[0][0][1]) / float(METER_TO_PIXEL)]
#     print transform
#     print dcenter
#     yaw = np.arctan2(transform[1, 0], transform[0, 0])
#     print yaw
#
# else:
#     print "Not enough matches are found - %d/%d" % (len(good), MIN_MATCH_COUNT)
#     matchesMask = None
#
# out = drawMatches(img1, kp1, img2, kp2, good, matchesMask)
# cv2.imwrite('test.jpg', out)

img1 = cv2.imread('/Users/baichuanhuang/Downloads/img5.jpg')
detector = cv2.ORB(nfeatures=80, scoreType=cv2.ORB_FAST_SCORE)
kp1, des1 = detector.detectAndCompute(img1, None)
cv2.drawKeypoints(img1, kp1, img1)

img2 = cv2.imread('/Users/baichuanhuang/Downloads/img6.jpg')
kp2, des2 = detector.detectAndCompute(img2, None)
cv2.drawKeypoints(img2, kp2, img2)

index_params = dict(algorithm=6,
                    table_number=6,  # 12
                    key_size=12,     # 20
                    multi_probe_level=1)  # 2
search_params = dict(checks=50)   # or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params, search_params)
matches = flann.knnMatch(des1, des2, k=2)
good = []
for match in matches:
    if len(match) > 1 and match[0].distance < 0.7 * match[1].distance:
        good.append(match[0])
print len(good)

src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
transform = cv2.estimateRigidTransform(src_pts, dst_pts, False)
print transform[0, 2], transform[1, 2]

cv2.imshow('img1', img1)
cv2.imshow('img2', img2)
cv2.waitKey(0)
cv2.destroyAllWindows()


# transformed_center = cv2.transform(CAMERA_CENTER, transform)
# print transformed_center
#
# transform = [-transform[0, 2] * 0.2, transform[1, 2] * 0.2]

import cv2
import numpy as np

# Load two images
img1 = cv2.imread('terr1.png')
img2 = cv2.imread('terr2.png')
img2 = cv2.resize(img2, (1778,866))

# Detect keypoints and descriptors in both images
sift = cv2.xfeatures2d.SIFT_create()
kp1, des1 = sift.detectAndCompute(img1, None)
kp2, des2 = sift.detectAndCompute(img2, None)

# Match keypoints between the two images
matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
matches = matcher.match(des1, des2)

# Calculate homography matrix using RANSAC
src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

# Warp the second image onto the first image
h, w, _ = img1.shape
warped_img2 = cv2.warpPerspective(img2, M, (w, h))

# Transform a set of points from the second image to the first image
src_points = np.array([[0, 0], [0, img2.shape[0]], [img2.shape[1], img2.shape[0]], [img2.shape[1], 0]], dtype=np.float32).reshape(-1, 1, 2)
dst_points = cv2.perspectiveTransform(src_points, M)
cv2.namedWindow("Result", cv2.WINDOW_NORMAL)

# Draw the warped image and the matching points
draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, flags=2)
result_img = cv2.drawMatches(img1, kp1, warped_img2, kp2, matches, None, **draw_params)
result_img = cv2.polylines(result_img, [np.int32(dst_points)], True, (255, 0, 0), 3, cv2.LINE_AA)
cv2.imshow('Result', result_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
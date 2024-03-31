import cv2
import numpy as np
import os
import glob

# Read and resize image
def read_and_resize(image_path, resize_max_dim=None):
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if resize_max_dim and (image.shape[0] > resize_max_dim or image.shape[1] > resize_max_dim):
        ratio = resize_max_dim / max(image.shape[:2])
        image = cv2.resize(image, (int(ratio * image.shape[1]), int(ratio * image.shape[0])))
    return image

def detect_and_describe(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    orb = cv2.SIFT_create()
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    return keypoints, descriptors

def match_keypoints(desc1, desc2, ratio=0.75):
    index_params = dict(algorithm = 1, trees = 5)
    search_params = dict(checks = 50)
    bf = cv2.FlannBasedMatcher(index_params, search_params)
    rawMatches = bf.knnMatch(desc1, desc2, k=2)
    matches = [m for m, n in rawMatches if m.distance < ratio * n.distance]
    return matches

def stitch_images(img1, img2, kp1, kp2, matches):
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)

    for i, match in enumerate(matches):
        points1[i, :] = kp1[match.queryIdx].pt
        points2[i, :] = kp2[match.trainIdx].pt

    H, status = cv2.findHomography(points1, points2, cv2.RANSAC)
    result = cv2.warpPerspective(img1, H, (img1.shape[1] + img2.shape[1], img2.shape[0]))
    result[0:img2.shape[0], 0:img2.shape[1]] = img2

    return result

resize_max_dim = 1000
image_folder_path = '/media/snkt/eab60d35-b90d-4cbc-ae6e-c45f2d159c56/home/sanket/Downloads/highaltitude-nongps-testpics-30Mar2024/tests'
image_files = sorted(glob.glob(os.path.join(image_folder_path, '*.jpg')))
images = [read_and_resize(image_path, resize_max_dim) for image_path in image_files]

keypoints = []
descriptors = []
for image in images:
    kp, desc = detect_and_describe(image)
    keypoints.append(kp)
    descriptors.append(desc)

imgA = images[0]
kpA = keypoints[0]
descA = descriptors[0]

for imgB, kpB, descB in zip(images[1:], keypoints[1:], descriptors[1:]):
    matches = match_keypoints(descA, descB)
    stitched_img = stitch_images(imgA, imgB, kpA, kpB, matches)
    
    imgA = stitched_img
    kpA, descA = detect_and_describe(stitched_img)

output_path = '/media/snkt/eab60d35-b90d-4cbc-ae6e-c45f2d159c56/home/sanket/Downloads/stitched_output.jpg'
cv2.imwrite(output_path, stitched_img)


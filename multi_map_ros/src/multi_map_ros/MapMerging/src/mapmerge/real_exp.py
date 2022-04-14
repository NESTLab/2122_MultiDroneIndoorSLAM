import numpy as np
import cv2
from keypoint_merge import sift_mapmerge, orb_mapmerge
from hough_merge import hough_mapmerge
from ros_utils import pgm_to_numpy
from merge_utils import acceptance_index
import matplotlib.pyplot as plt
from merge_utils import apply_warp

def draw_matches(img1, img2):
    """
    Visualize Keypoint Matches Between Images
    """
    sift = cv2.SIFT_create()
    keypoints_1, descriptors_1 = sift.detectAndCompute(img1,None)
    keypoints_2, descriptors_2 = sift.detectAndCompute(img2,None)
    # index_params = dict(algorithm = 0, trees = 5)
    # search_params = dict(checks=5000)
    # flann = cv2.FlannBasedMatcher(index_params,search_params)
    # matches = flann.knnMatch(descriptors_1,descriptors_2,k=2)
    # good_matches = []
    # for m, n in matches:
    #     # lowes ratio
    #     if m.distance < 0.7 * n.distance:
    #         good_matches.append(m)
    descriptors_1 = descriptors_1 / np.linalg.norm(descriptors_1, ord=1, axis=1, keepdims=True)
    descriptors_1 = np.sqrt(descriptors_1)
    descriptors_2 = descriptors_2 / np.linalg.norm(descriptors_2, ord=1, axis=1, keepdims=True)
    descriptors_2 = np.sqrt(descriptors_2)
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = bf.match(descriptors_1,descriptors_2)
    matches = sorted(matches, key = lambda x:x.distance)
    img3 = cv2.drawMatches(img1, keypoints_1, img2, keypoints_2, matches[:30], img2, flags=2)
    return img3

def evaluate_merge(M1, M2, GT, vis=False, method="sift"):
    """
    Align M1 to GT
    Align M2 to M1
    Evaluate IoU of M1-GT, M2-GT, and Merge-GT
    "Successful" merge if IoU increases
    """
    cv2_shape = (GT.shape[1], GT.shape[0])
    merge_fn = None
    if method == "sift":
        merge_fn = orb_mapmerge
    else:
        merge_fn = hough_mapmerge
    M1 = cv2.resize(M1, dsize=cv2_shape, interpolation=cv2.INTER_NEAREST)
    M2 = cv2.resize(M2, dsize=cv2_shape, interpolation=cv2.INTER_NEAREST)
    M1_align, _ = merge_fn(GT, M1)
    M2_align, _ = merge_fn(GT, M2)
    merge, _ = merge_fn(M1, M2)
    merge, _ = merge_fn(GT, merge)
    m1_iou = acceptance_index(M1_align, GT)
    m2_iou = acceptance_index(M2_align, GT)
    merge_iou = acceptance_index(merge, GT)
    result = None
    if merge_iou > m1_iou and merge_iou > m2_iou:
        result = 1
    else:
        result = 0
    return result, merge, M1_align, M2_align

def load_samples(i):
    r0_fname = f"multi_map_ros/src/multi_map_ros/MapMerging/src/mapmerge/Data/Gazebo/robot0_map{i}.pgm"
    r1_fname = r0_fname.replace("robot0", "robot1")
    r2_fname = r1_fname.replace("robot1", "robot2")
    R0 = pgm_to_numpy(r0_fname)
    R1 = pgm_to_numpy(r1_fname)
    R2 = pgm_to_numpy(r2_fname)
    return R0, R1, R2

def show_images(images):
    n = len(images)
    fig, axes = plt.subplots(nrows=1, ncols=n)
    for i in range(n):
        axes[i].imshow(images[i], cmap="gray")
    plt.show()

def dist_transform(img):
    dist_img = np.where(img == 127, 255, 0)
    dist_img = np.where(dist_img == 255, 0, 255)
    dist_img = np.expand_dims(dist_img, -1).astype(np.uint8)
    dist_img = cv2.distanceTransform(dist_img, cv2.DIST_L2, 5)
    dist_img = dist_img / np.max(dist_img)
    dist_img *= 255
    dist_img = dist_img.astype(np.uint8)
    return dist_img

if __name__ == "__main__":
    R0, R1, R2 = load_samples(50)
    C1 = (640, 160)
    C2 = (425, 205)
    R1_dist = dist_transform(R1)
    R2_dist = dist_transform(R2)
    show_images([R1, R2, R1_dist, R2_dist])
    GT = np.loadtxt("multi_map_ros/src/multi_map_ros/MapMerging/src/mapmerge/Data/Gazebo/PARSED_MONSTER.txt").astype(np.uint8)
    # GT = np.flipud(GT)
    # GT = np.fliplr(GT)
    show_images([R0, R1, R2, GT])
    t1 = draw_matches(R1, R2)
    plt.imshow(t1)
    plt.show()
    result, merge, M0_align, M1_align = evaluate_merge(R1, R2, GT, method="sift")
    show_images([R1, R2, merge, GT])
    t0 = draw_matches(R0, GT)
    plt.imshow(t0)
    plt.show()
    
    print("")


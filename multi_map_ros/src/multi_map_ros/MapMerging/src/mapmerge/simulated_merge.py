from PIL.Image import NEAREST
from constants import FREE, OCCUPIED, UNKNOWN
import matplotlib
import numpy as np
import cv2
from keypoint_merge import sift_mapmerge, orb_mapmerge
from hough_merge import hough_mapmerge
from merge_utils import acceptance_index, augment_map, combine_aligned_maps
import matplotlib.pyplot as plt
from merge_utils import apply_warp, load_mercer_map
from mapmerge.merge_utils import apply_warp, acceptance_index, median_filter
import itertools
from tqdm import tqdm

# 1. load maps
# 2. set "starting area"
# 3. pick k points which are at the greatest bfs distance from entry pixel
# 4. undo-exploration from that point for n steps
# 5. record % of area which is entrance: explore until this area is 10/20/30..90% of the area known
# 6. we are left with maps with various % overlap, and we know what the GT map is

ENTRANCE = 37

import pickle

INTEL_MAP = "multi_map_ros/src/multi_map_ros/MapMerging/src/mapmerge/Data/Mercer/intel.txt"
intel = load_mercer_map("intel.txt")
corridor = load_mercer_map("LongCorridor2.txt")
other = cv2.imread("/home/connor/2122_MultiDroneIndoorSLAM/coms/coms/tests/test_data/halmstad/E5_14.png", 0)

# plt.imshow(intel, cmap='gray')
# plt.show()
def load_maps_with_entrance(show_map=False, show_entrance=False, show_explorer_starts=False, map="corridor"):
    intel = load_mercer_map("intel.txt")
    intel[350:445,340:445,] = UNKNOWN
    corridor = load_mercer_map("LongCorridor2.txt")
    if show_map:
        if map == "intel":
            plt.imshow(intel, cmap="gray")
            plt.axis("off")
            plt.show()
        else:
            plt.imshow(corridor, cmap="gray")
            plt.axis("off")
            plt.show()

    # intel parameters
    xmin, ymin, xmax, ymax = 112, 595, 250, 690
    entrance_intel = (xmin, ymin, xmax, ymax)
    x2, y2 = 282, 150
    x3, y3 = 575, 175
    x5, y5 = 625, 360
    x6, y6 = 400, 150
    x6, y6 = 645, 450
    x7, y7 = 385, 150
    starts_intel = [(x2, y2), (x3, y3), (x5, y5), (x6, y6),(x7, y7)]

    # corridor parameters
    xmin, ymin, xmax, ymax = 260, 8, 315, 140
    entrance_corridor = (xmin, ymin, xmax, ymax)
    x1, y1 = 270, 720
    x2, y2 = 740, 130
    starts_corridor = [(x1, y1), (x2, y2)]

    # 3rd set
    xmin, ymin, xmax, ymax = 400, 980, 530, 1140
    entrance_other = (xmin, ymin, xmax, ymax)
    x1, y1 = 300, 500
    x2, y2 = 515, 500
    x3, y3 = 1110, 470
    x4, y4 = 1350, 710
    x5, y5 = 1000, 1080
    starts_other = [(x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5)]

    maps = dict()
    maps["intel"] = {"entrance":entrance_intel, "starts": starts_intel, "map": intel}
    maps["corridor"] = {"entrance":entrance_corridor, "starts": starts_corridor, "map": corridor}
    maps["other"] = {"entrance":entrance_other, "starts": starts_other, "map": other}

    starts, entrance, grid = maps[map]["starts"], maps[map]["entrance"], maps[map]["map"]
    
    grid_rgb = np.empty(shape=(grid.shape + (3,)), dtype=np.uint8)
    for c in range(3):
        grid_rgb[:,:,c] = grid
    if show_entrance:
        xmin, ymin, xmax, ymax = entrance
        grid_rgb[ymin:ymax, xmin:xmax] = np.where(grid_rgb[ymin:ymax, xmin:xmax] == FREE, [0, 255, 0], grid_rgb[ymin:ymax, xmin:xmax])
    if show_explorer_starts:
        r = 7
        for x, y in starts:
            grid_rgb[y-r:y+r, x-r:x+r] = [255, 0, 0]
    if show_entrance or show_explorer_starts:
        plt.imshow(grid_rgb)
        plt.axis("off")
        plt.show()
    return grid, entrance, starts

def unexplore_bfs(map, start_x, start_y, ignore_tiles=[ENTRANCE, UNKNOWN], num_steps=1):
    """
    Mark start tile + num_steps tiles as unknown, skipping entrance/unknown tiles
    """
    def get_neighbors(map, x, y, seen={}, ignore_tiles=ignore_tiles):
        neighbors = [(x-1, y+1), (x-1, y), (x-1, y-1), (x, y+1), (x, y), (x, y-1), (x+1, y+1), (x+1, y), (x+1, y-1)]
        valid_neighbors = [n for n in neighbors if map[n[1], n[0]] not in ignore_tiles and n not in seen ]
        return valid_neighbors
    
    # start with empty stack
    stack = get_neighbors(map, start_x, start_y)
    seen = dict()
    steps_taken = 0
    while steps_taken < num_steps:
        x, y = stack.pop(0)
        if (x, y) in seen:
            continue
        else:
            seen[(x,y)] = True
            steps_taken += 1
            # mark as unknown
            map[y, x] = UNKNOWN
            stack = stack + get_neighbors(map, x, y, seen)
    median_filter = cv2.medianBlur(map, ksize=3)
    return median_filter

def generate_unexplored_map(map, entrance_coords, start, ratio=0.5):
    """
    generate map with ratio % overlap (excluding entrance)
    """
    start_x, start_y = start
    xmin, ymin, xmax, ymax = entrance_coords
    map_masked = np.copy(map)
    map_masked[ymin:ymax, xmin:xmax] = ENTRANCE
    num_entrance = np.sum(map_masked == ENTRANCE)
    num_explored = np.sum((map_masked != ENTRANCE) & (map_masked != UNKNOWN))
    num_explored_corrected = num_explored - num_entrance
    num_to_unexplore = (1 - ratio) * num_explored_corrected
    unexplored_map = unexplore_bfs(map_masked, start_x, start_y, num_steps=num_to_unexplore)
    unexplored_map = np.where(map_masked == ENTRANCE, map, unexplored_map)
    return unexplored_map


def dist_transform(img):
    dist_img = np.where(img == 127, 255, 0)
    dist_img = np.where(dist_img == 255, 0, 255)
    dist_img = np.expand_dims(dist_img, -1).astype(np.uint8)
    dist_img = cv2.distanceTransform(dist_img, cv2.DIST_L2, 3)
    dist_img = dist_img / np.max(dist_img)
    dist_img *= 255
    dist_img = dist_img.astype(np.uint8)
    return dist_img

def generate_training_data(grid, entrance_coords, starts, ratios=[0.95]):
    print("RATIOS", ratios)
    # ratios = [0.90, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]
    # ratios = [0.95]#[0.95, ]#0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]
    ratio_maps = {r: [] for r in range(len(ratios))}
    # grid = cv2.medianBlur(grid, ksize=3)
    for r_idx in tqdm(range(len(ratios))):
        for i in range(len(starts)):
            explore = generate_unexplored_map(grid, entrance_coords, starts[i], ratio=ratios[r_idx])
            ratio_maps[r_idx].append(explore)
    return grid, ratio_maps

def calculate_iou(original, merged):
    intersection = np.sum((original == merged) & (merged != UNKNOWN))
    disagreement = np.sum((original != merged) & (merged != UNKNOWN))
    union = intersection + disagreement
    iou = intersection / (union + 1e-10)
    return iou

def safe_center_crop(map):
    free_x_left, free_x_right = 0, 0
    free_y_top, free_y_bot = 0, 0
    for x in range(map.shape[0]):
        if np.sum(map[:,x]) == map.shape[1]*UNKNOWN:
            free_x_left += 1
        else:
            break
    for x in range(map.shape[0]):
        if np.sum(map[:,map.shape[0]-(x+1)]) == map.shape[1]*UNKNOWN:
            free_x_right += 1
        else:
            break
    for y in range(map.shape[1]):
        if np.sum(map[y,:]) == map.shape[0]*UNKNOWN:
            free_y_top += 1
        else:
            break
    for y in range(map.shape[1]):
        if np.sum(map[map.shape[1]-(y+1),:]) == map.shape[0]*UNKNOWN:
            free_y_bot += 1
        else:
            break
    num_crop_x = min(free_x_right, free_x_left) // 2
    num_crop_y = min(free_y_bot, free_y_top) // 2
    x_max = map.shape[0] - num_crop_x
    y_max = map.shape[1] - num_crop_y
    cropped = map[num_crop_y:y_max, num_crop_x:x_max]
    cropped = cv2.resize(cropped, dsize=(map.shape), interpolation=cv2.INTER_NEAREST)
    # fig, axes = plt.subplots(1, 2)
    # axes[0].imshow(map)
    # axes[1].imshow(cropped)
    # plt.show()
    return cropped, map
    

def pairwise_merge_data(grid, entrance_coords, starts, methods=["sift", "orb", "hough"], ratios=[0.95]):
    grid, map_scales = generate_training_data(grid, entrance_coords, starts, ratios)
    sift_ious = []
    orb_ious = []
    hough_ious = []
    for i in range(5):
        for scale in range(len(map_scales.keys())):
            maps = map_scales[scale]
            pairs = [p for p in itertools.combinations(maps, 2)]
            pair_idx = 0
            for pair in pairs:
                pair_idx += 1
                map1, map2 = pair

                map2 = median_filter(map2, ksize=7)

                # map1 = cv2.resize(map1, dsize=(1000, 1000), interpolation=cv2.INTER_NEAREST)
                # map1 = median_filter(map1)
                # map2 = cv2.resize(map2, dsize=(1000, 1000), interpolation=cv2.INTER_NEAREST)
                # map2 = median_filter(map2)
                # intel_merge = cv2.resize(intel, dsize=(1000, 1000), interpolation=cv2.INTER_NEAREST)
                # intel_merge = median_filter(intel_merge)

                # map1 = cv2.resize(map1, dsize=(512, 512), interpolation=cv2.INTER_NEAREST)
                # map2 = cv2.resize(map2, dsize=(512, 512), interpolation=cv2.INTER_NEAREST)
                # intel_merge = cv2.resize(intel_merge, dsize=(512, 512), interpolation=cv2.INTER_NEAREST)
                
                # map1 = median_filter(map1)
                # map1 = median_filter(map1)
                map2_aug, _ = augment_map(map2, shift_limit=0, rotate_limit=45, scale_noise=False)

                # map1 = median_filter(map1)
                # map1 = median_filter(map1)

                # plt.imshow(map1, cmap="gray")
                # plt.show()
                # map1 = median_filter(map1)
                pepper_noise = np.random.rand(map1.shape[0], map1.shape[1])
                map1 = np.where((map1 == FREE) & (pepper_noise < 0.01), OCCUPIED, map1)

                # plt.imshow(map1, cmap="gray")
                # plt.show()
                if "sift" in methods:
                    align_map2, M = sift_mapmerge(map1, map2_aug)
                    iou = calculate_iou(intel, align_map2)
                    sift_ious.append(iou)
                    # fig, axes = plt.subplots(1, 3)
                    # axes[0].imshow(map1)
                    # axes[1].imshow(map2_aug)
                    # axes[2].imshow(align_map2)
                    # plt.show()
                    # print("SIFT:", iou)
                    # plt.imshow(align_map2, cmap="gray")
                    # plt.show()
                if "orb" in methods:
                    align_map2, M = orb_mapmerge(map1, map2_aug)
                    iou = calculate_iou(intel, align_map2)
                    orb_ious.append(iou)
                    # print("ORB:", iou)
                    # plt.imshow(align_map2, cmap="gray")
                    # plt.show()
                if "hough" in methods:
                    align_map2, M = hough_mapmerge(map1, map2_aug)
                    iou = calculate_iou(intel, align_map2)
                    hough_ious.append(iou)
                    # print("Hough:", iou)
                    # plt.imshow(align_map2, cmap="gray")
                    # plt.show()

                # fig, axes = plt.subplots(1, 5)
                # plt.axis("off")
                # axes[0].imshow(map1, cmap="gray")
                # axes[0].set_title("Map1")
                # axes[0].axis("off")
                # axes[1].imshow(map2, cmap="gray")
                # axes[1].set_title("Map2")
                # axes[1].axis("off")
                # axes[2].imshow(map2_aug, cmap="gray")
                # axes[2].set_title("Map2 + Transformation")
                # axes[2].axis("off")
                # axes[3].imshow(merged, cmap="gray")
                # axes[3].set_title("Merged Map")
                # axes[3].axis("off")
                # merged_rgb = np.empty(shape=(merged.shape + (3,)), dtype=np.uint8)
                # for i in range(3):
                #     merged_rgb[:,:,i] = merged
                # merged_rgb[((map2 == merged) & (map2 != UNKNOWN))] = [0, 255, 0]
                # merged_rgb[((map2 != merged) & (map2 != UNKNOWN))] = [255, 0, 0]
                # axes[4].imshow(merged_rgb)
                # axes[4].set_title("Marked Correct/Incorrect Merge")
                # axes[4].axis("off")
                # plt.show()  
                # break
            # break
    sift_ious = np.asarray(sift_ious)
    orb_ious = np.asarray(orb_ious)
    hough_ious = np.asarray(hough_ious)

    for res in [sift_ious, orb_ious, hough_ious]:
        print(np.mean(res))
        print(np.sum(res > 0.9) / np.sum(res > 0))


if __name__ == "__main__":
    # visualize maps
    grid, entrance_coords, starts = load_maps_with_entrance(show_map=False, show_entrance=False, show_explorer_starts=False, map="intel")

    
    # ratios = [0.5] #0.5, 0.25]
    # fig, axes = plt.subplots(nrows=len(ratios), ncols=len(starts)+1, figsize=(12, 10))
    # maps = { ratio: [] for ratio in ratios}  # dict of overlap ratio : corresponding maps

    # for r_idx in range(len(ratios)):
    #     ratio = ratios[r_idx]
    #     axes[r_idx][0].imshow(grid, cmap="gray")
    #     axes[r_idx][0].xaxis.set_ticklabels([])
    #     axes[r_idx][0].yaxis.set_ticklabels([])
    #     axes[r_idx][0].set_title("GT Map")
    #     for i in range(len(starts)):
    #         explore = generate_unexplored_map(grid, entrance_coords, starts[i], ratio=ratios[r_idx])
    #         maps[ratios[r_idx]].append(explore)
    #         axes[r_idx][i+1].imshow(explore, cmap="gray")
    #         axes[r_idx][i+1].set_title(f"Exp{i+1}")
    #         axes[r_idx][i+1].xaxis.set_ticklabels([])
    #         axes[r_idx][i+1].yaxis.set_ticklabels([])
    # plt.show()


    # print("gah")
    pairwise_merge_data(grid, entrance_coords, starts, ratios=[0.95])
    pairwise_merge_data(grid, entrance_coords, starts, ratios=[0.9])
    pairwise_merge_data(grid, entrance_coords, starts, ratios=[0.8])
    pairwise_merge_data(grid, entrance_coords, starts, ratios=[0.7])
    pairwise_merge_data(grid, entrance_coords, starts, ratios=[0.6])
    pairwise_merge_data(grid, entrance_coords, starts, ratios=[0.5])
    pairwise_merge_data(grid, entrance_coords, starts, ratios=[0.4])
    pairwise_merge_data(grid, entrance_coords, starts, ratios=[0.3])



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
import itertools
from tqdm import tqdm

# 1. load maps
# 2. set "starting area"
# 3. pick k points which are at the greatest bfs distance from entry pixel
# 4. undo-exploration from that point for n steps
# 5. record % of area which is entrance: explore until this area is 10/20/30..90% of the area known
# 6. we are left with maps with various % overlap, and we know what the GT map is

ENTRANCE = 37

INTEL_MAP = "/root/catkin_ws/src/coms/coms/src/mapmerge/intel.txt"
def load_maps_with_entrance(show_map=False, show_entrance=False, show_explorer_starts=False, map="intel"):
    intel = load_mercer_map(INTEL_MAP)
    # corridor = load_mercer_map("LongCorridor2.txt")
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

    maps = dict()
    maps["intel"] = {"entrance":entrance_intel, "starts": starts_intel, "map": intel}
    maps["corridor"] = {"entrance":entrance_corridor, "starts": starts_corridor, "map": corridor}

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

def generate_training_data(grid, entrance_coords, starts):
    # ratios = [0.90, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]
    ratios = [0.6]#[0.95, ]#0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]
    ratio_maps = {r: [] for r in range(len(ratios))}
    grid = cv2.medianBlur(grid, ksize=3)
    for r_idx in tqdm(range(len(ratios))):
        for i in range(len(starts)):
            explore = generate_unexplored_map(grid, entrance_coords, starts[i], ratio=ratios[r_idx])
            ratio_maps[r_idx].append(explore)
    return grid, ratio_maps

def calculate_iou(original, merged):
    intersection = np.sum((original == merged) & (original != UNKNOWN))
    disagreement = np.sum((original != merged) & (original != UNKNOWN))
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
    

def pairwise_merge_data(grid, entrance_coords, starts, methods=["sift", "rootsift"]):
    grid, map_scales = generate_training_data(grid, entrance_coords, starts)
    (xmin, ymin, xmax, ymax) = entrance_coords
    # kp_entrance, desc_entrance = cv2.SIFT_create().detectAndCompute(grid[ymin-8:ymax+8, xmin-8:xmax+8], None)
    entrance_map = grid[ymin-8:ymax+8, xmin-8:xmax+8]
    sift_ious_scale = {r: [] for r in range(len(map_scales.keys()))}
    rootsift_ious_scale = {r: [] for r in range(len(map_scales.keys()))}
    prior_ious_scale = {r: [] for r in range(len(map_scales.keys()))}
    orb_ious_scale = {r: [] for r in range(len(map_scales.keys()))}
    hough_ious_scale = {r: [] for r in range(len(map_scales.keys()))}
    sift_ious_scale_crop = {r: [] for r in range(len(map_scales.keys()))}
    rootsift_ious_scale_crop = {r: [] for r in range(len(map_scales.keys()))}
    orb_ious_scale_crop = {r: [] for r in range(len(map_scales.keys()))}
    hough_ious_scale_crop = {r: [] for r in range(len(map_scales.keys()))}
    for i in tqdm(range(10)):
        for scale in range(len(map_scales.keys())):
            maps = map_scales[scale]
            pairs = [p for p in itertools.combinations(maps, 2)]
            pair_idx = 0
            for pair in pairs:
                pair_idx += 1
                map1, map2 = pair
                map2_aug, _ = augment_map(map2, shift_limit=0, rotate_limit=45, scale_noise=False)
                entrance_aug, _ = augment_map(entrance_map, shift_limit=0, rotate_limit=45, scale_noise=False)
                map2_aug_crop, map2_aug = safe_center_crop(map2_aug)
                _, M, align_map2, align_resize = None, None, None, None
                if "sift" in methods:
                    _, M_resize = sift_mapmerge(map1, map2_aug_crop)
                    align_map2, M = sift_mapmerge(map1, map2_aug)
                    aligned_entrance, M = sift_mapmerge(entrance_map, entrance_aug)
                    align_resize = apply_warp(map2_aug, M_resize)
                    merged = combine_aligned_maps(map1, align_map2)
                    merged_entrance = combine_aligned_maps(entrance_map, aligned_entrance)
                    merged_resized = combine_aligned_maps(map1, align_resize)
                    iou = calculate_iou(map2, merged)
                    iou_prior = calculate_iou(entrance_map, merged_entrance)
                    iou_resize = calculate_iou(map2, merged_resized)
                    sift_ious_scale[scale].append(iou)
                    sift_ious_scale_crop[scale].append(iou_resize)
                    prior_ious_scale[scale].append(iou_prior)
                if "rootsift" in methods:
                    _, M_resize = sift_mapmerge(map1, map2_aug_crop, rootsift=True)
                    align_map2, M = sift_mapmerge(map1, map2_aug, rootsift=True)
                    align_resize = apply_warp(map2_aug, M_resize)
                    merged = combine_aligned_maps(map1, align_map2)
                    merged_resized = combine_aligned_maps(map1, align_resize)
                    iou = calculate_iou(map2, merged)
                    iou_resize = calculate_iou(map2, merged_resized)
                    rootsift_ious_scale[scale].append(iou)
                    rootsift_ious_scale_crop[scale].append(iou_resize)
                if "orb" in methods:
                    _, M_resize = orb_mapmerge(map1, map2_aug_crop)
                    align_resize = apply_warp(map2_aug, M_resize)
                    align_map2, M = orb_mapmerge(map1, map2_aug)
                    merged = combine_aligned_maps(map1, align_map2)
                    merged_resized = combine_aligned_maps(map1, align_resize)
                    iou = calculate_iou(map2, merged)
                    iou_resize = calculate_iou(map2, merged_resized)
                    orb_ious_scale[scale].append(iou)
                    orb_ious_scale_crop[scale].append(iou_resize)
                if "hough" in methods:
                    _, params = hough_mapmerge(map1, map2_aug_crop)
                    M_rotation, M_translation = params
                    align_resize = apply_warp(map2_aug, M_rotation)
                    align_map2, M = hough_mapmerge(map1, map2_aug)
                    merged = combine_aligned_maps(map1, align_map2)
                    merged_resized = combine_aligned_maps(map1, align_resize)
                    iou = calculate_iou(map2, merged)
                    iou_resize = calculate_iou(map2, merged_resized)
                    hough_ious_scale[scale].append(iou)
                    hough_ious_scale_crop[scale].append(iou_resize)

                # if scale == 1 and pair_idx == len(pairs)-4:
            
                #     fig, axes = plt.subplots(1, 5)
                #     plt.axis("off")
                #     axes[0].imshow(map1, cmap="gray")
                #     axes[0].set_title("Map1")
                #     axes[0].axis("off")
                #     axes[1].imshow(map2, cmap="gray")
                #     axes[1].set_title("Map2")
                #     axes[1].axis("off")
                #     axes[2].imshow(map2_aug, cmap="gray")
                #     axes[2].set_title("Map2 + Noise")
                #     axes[2].axis("off")
                #     axes[3].imshow(merged, cmap="gray")
                #     axes[3].set_title("Merged Map")
                #     axes[3].axis("off")
                #     merged_rgb = np.empty(shape=(merged.shape + (3,)), dtype=np.uint8)
                #     for i in range(3):
                #         merged_rgb[:,:,i] = merged
                #     merged_rgb[((map2 == merged) & (map2 != UNKNOWN))] = [0, 255, 0]
                #     merged_rgb[((map2 != merged) & (map2 != UNKNOWN))] = [255, 0, 0]
                #     axes[4].imshow(merged_rgb)
                #     axes[4].set_title("Marked Correct/Incorrect Merge")
                #     axes[4].axis("off")
                #     plt.show()  
                # break
            # break
    # hough_means = [np.mean(hough_ious_scale[scale]) for scale in sift_ious_scale.keys()] if "hough" in methods else None
    sift_means = [np.mean(sift_ious_scale[scale]) for scale in sift_ious_scale.keys()] if "sift" in methods else None
    rootsift_means = [np.mean(rootsift_ious_scale[scale]) for scale in sift_ious_scale.keys()] if "rootsift" in methods else None
    # orb_means = [np.mean(orb_ious_scale[scale]) for scale in sift_ious_scale.keys()] if "hough" in methods else None
    # hough_mins = [np.min(hough_ious_scale[scale]) for scale in sift_ious_scale.keys()] if "hough" in methods else None
    sift_mins = [np.min(sift_ious_scale[scale]) for scale in sift_ious_scale.keys()] if "sift" in methods else None
    rootsift_mins = [np.min(rootsift_ious_scale[scale]) for scale in sift_ious_scale.keys()] if "rootsift" in methods else None
    # orb_mins = [np.min(orb_ious_scale[scale]) for scale in sift_ious_scale.keys()] if "hough" in methods else None
    # hough_means_crop = [np.mean(hough_ious_scale_crop[scale]) for scale in sift_ious_scale.keys()] if "hough" in methods else None
    # sift_means_crop = [np.mean(sift_ious_scale_crop[scale]) for scale in sift_ious_scale.keys()] if "hough" in methods else None
    # orb_means_crop = [np.mean(orb_ious_scale_crop[scale]) for scale in sift_ious_scale.keys()] if "hough" in methods else None
    # hough_mins_crop = [np.min(hough_ious_scale_crop[scale]) for scale in sift_ious_scale.keys()] if "hough" in methods else None
    # sift_mins_crop = [np.min(sift_ious_scale_crop[scale]) for scale in sift_ious_scale.keys()] if "hough" in methods else None
    # orb_mins_crop = [np.min(orb_ious_scale_crop[scale]) for scale in sift_ious_scale.keys()] if "orb" in methods else None
    print("hello")
    # print("Original Augmentation")
    # for i in range(len(hough_means)):
    #     print(f"{np.round(sift_means[i], 4)}, {np.round(orb_means[i], 4)}, {np.round(hough_means[i], 4)}, {np.round(sift_mins[i], 4)}, {np.round(orb_mins[i], 4)}, {np.round(hough_mins[i], 4)}")
    # print("Crop Augmentation")
    # for i in range(len(hough_means)):
    #     print(f"{np.round(sift_means_crop[i], 4)}, {np.round(orb_means_crop[i], 4)}, {np.round(hough_means_crop[i], 4)}, {np.round(sift_mins_crop[i], 4)}, {np.round(orb_mins_crop[i], 4)}, {np.round(hough_mins_crop[i], 4)}")
    # print("")


if __name__ == "__main__":
    # visualize maps
    grid, entrance_coords, starts = load_maps_with_entrance(show_map=True, show_entrance=True, show_explorer_starts=True, map="intel")
    
    ratios = [0.95, 0.85, 0.55, 0.15] #0.5, 0.25]
    fig, axes = plt.subplots(nrows=len(ratios), ncols=len(starts)+1, figsize=(12, 10))
    for r_idx in range(len(ratios)):
        ratio = ratios[r_idx]
        axes[r_idx][0].imshow(grid, cmap="gray")
        axes[r_idx][0].xaxis.set_ticklabels([])
        axes[r_idx][0].yaxis.set_ticklabels([])
        axes[r_idx][0].set_title("GT Map")
        for i in range(len(starts)):
            explore = generate_unexplored_map(grid, entrance_coords, starts[i], ratio=ratios[r_idx])
            axes[r_idx][i+1].imshow(explore, cmap="gray")
            axes[r_idx][i+1].set_title(f"Exp{i+1}")
            axes[r_idx][i+1].xaxis.set_ticklabels([])
            axes[r_idx][i+1].yaxis.set_ticklabels([])
    plt.show()


    # pairwise_merge_data(grid, entrance_coords, starts)



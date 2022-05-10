from statistics import median
from mapmerge.hough_merge import hough_mapmerge
from mapmerge.merge_utils import apply_warp, pad_maps, median_filter
import numpy as np
from mapmerge.keypoint_merge import sift_mapmerge, orb_mapmerge
from mapmerge.merge_utils import resize_map, combine_aligned_maps, acceptance_index

import cv2 
from datetime import datetime

# SELECT SCALES TO USE IN SCALE PROCESS (Test-Time Augmentation)
SCALES = [0.5, 0.75, 1, 1.25, 2.0]  # classic scale regime for TTA
SCALES_FAST = [0.5, 0.75, 1, 1.25]  # exclude 2x scale for faster runtime

def mapmerge_pipeline(map1, map2, method="hough", scale_process=False, median_process=False):
    """
    end-to-end map merge pipeline for testing
    
    args: 
    - map1 (original map)
    - map2 (foreign map, the one to be transformed onto map1)
    - method: one of ["sift", "orb", "hough"]. Default: "hough"
    - scale_process: boolean, whether or not to run merges with rescaled maps for finer results (at cost of speed). Default: False
    - median_process: whether or not to apply median filter to reduce noise. Default: True
    """
    merge_fn = None
    if method == "sift":
        merge_fn = sift_mapmerge 
    elif method == "orb":
        merge_fn = orb_mapmerge
    else:
        merge_fn = hough_mapmerge
    map1, map2 = pad_maps(map1, map2)
    if scale_process:  # do not use for now
        ious = []
        merges = []
        for scale in SCALES_FAST:
            map1_scale = resize_map(map1, dsize=(int(map1.shape[0] * scale), int(map1.shape[1] * scale)))
            map2_scale = resize_map(map2, dsize=(int(map2.shape[0] * scale), int(map2.shape[1] * scale)))
            if median_process:
                map1_scale = median_filter(map1_scale)
                map2_scale = median_filter(map2_scale)
            _, M_scale = merge_fn(map1_scale, map2_scale)
            # use M from scale process on original maps
            transformed_map2 = apply_warp(map2, M_scale)
            merged_map = combine_aligned_maps(transformed_map2, map1)
            ious.append(acceptance_index(map1, merged_map))
            merges.append(merged_map)
        return merges[np.argmax(ious)]
    else:
        M = None
        if median_process:
            transformed_map2, M, acpt = merge_fn(median_filter(map1), median_filter(map2))
        else:
            transformed_map2, M, acpt = merge_fn(map1, map2)
        #transformed_map2 = apply_warp(map2, M)
        # log errors
        if acpt < 0.9:
            cv2.imwrite(f"map1_acpt_{acpt}_time_{datetime.now().png}", map1)
            cv2.imwrite(f"map2_acpt_{acpt}_time_{datetime.now().png}", map2)
            cv2.imwrite(f"aligned_acpt_{acpt}_time_{datetime.now().png}", transformed_map2)
        merged_map = combine_aligned_maps(transformed_map2, map1)
        return merged_map

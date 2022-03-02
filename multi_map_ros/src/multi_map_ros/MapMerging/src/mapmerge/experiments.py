import numpy as np
import matplotlib.pyplot as plt
from time import time
from tqdm import tqdm

from mapmerge.constants import *
from mapmerge.merge_utils import acceptance_index, combine_aligned_maps, get_training_sample, resize_map
from mapmerge.hough_merge import hough_mapmerge
from mapmerge.keypoint_merge import sift_mapmerge, orb_mapmerge
from mapmerge.ros_utils import pgm_to_numpy

def paper_benchmark(n_iters=1000, shift_limit=0.1, rotate_limit=360, maps=None, silent=False):
    """
    NOTE: passing in 'maps' should be an actual numpy map, NOT the path to a file
    Similar benchmark to that in Carpin et al.

    Inputs:
        n_iters: number of iterations (augmented map pairs) to test
        shift_limit: proportion of image width/height to set as translation limit
        rotate_limit: (degrees) maximum image rotation
    """

    if maps is None:
        filename1 = TRAIN_FILENAMES[0]
        filename2 = TRAIN_FILENAMES[5]
        filename3 = TRAIN_FILENAMES[6]
        filename4 = TEST_FILENAMES[0]

        maps = [filename1, filename2, filename3, filename4]

    SIFT_RESULTS = [[], [], [], []]
    SIFT_TIMES = [[], [], [], []]
    ORB_RESULTS = [[], [], [], []]
    ORB_TIMES = [[], [], [], []]
    HOUGH_RESULTS = [[], [], [], []]
    HOUGH_TIMES = [[], [], [], []]
    for i in tqdm(range(n_iters)):
        for m_idx in range(len(maps)):
            if maps and len(maps) == 2:
                map1, map2 = maps[0], maps[1]
            else:
                map1, map2 = get_training_sample(maps[m_idx])
            # sift
            sift_start = time()
            sift_map = sift_mapmerge(map1, map2)
            sift_end = time()
            sift_elapsed = sift_start - sift_end
            SIFT_RESULTS[m_idx].append(acceptance_index(map1, sift_map))
            SIFT_TIMES[m_idx].append(sift_elapsed)
            # hough 
            hough_start = time()
            hough_map = hough_mapmerge(map1, map2, robust=True)
            hough_end = time()
            hough_elapsed = hough_start - hough_end
            HOUGH_RESULTS[m_idx].append(acceptance_index(map1, hough_map))
            HOUGH_TIMES[m_idx].append(hough_elapsed)
            # orb 
            orb_start = time()
            orb_map = orb_mapmerge(map1, map2)
            orb_end = time()
            orb_elapsed = orb_start - orb_end
            ORB_RESULTS[m_idx].append(acceptance_index(map1, orb_map))
            ORB_TIMES[m_idx].append(orb_elapsed)
    
    if not silent:
        SIFT_MU, SIFT_STD, SIFT_TIME = np.mean(SIFT_RESULTS, axis=1), np.std(SIFT_RESULTS, axis=1), np.mean(SIFT_TIMES, axis=1)
        print("(Average Acc, Std Acc, Average Time) per map: (SIFT)")
        print(SIFT_MU, SIFT_STD, SIFT_TIME)
        ORB_MU, ORB_STD, ORB_TIME = np.mean(ORB_RESULTS, axis=1), np.std(ORB_RESULTS, axis=1), np.mean(ORB_TIMES, axis=1)
        print("(Average Acc, Std Acc, Average Time) per map: (ORB)")
        print(ORB_MU, ORB_STD, ORB_TIME)
        HOUGH_MU, HOUGH_STD, HOUGH_TIME = np.mean(HOUGH_RESULTS, axis=1), np.std(HOUGH_RESULTS, axis=1), np.mean(HOUGH_TIMES, axis=1)
        print("(Average Acc, Std Acc, Average Time) per map: (HOUGH)")
        print(HOUGH_MU, HOUGH_STD, HOUGH_TIME)

    return {
        "SIFT_RESULTS": SIFT_RESULTS,
        "SIFT_TIMES": SIFT_TIMES,
        "ORB_RESULTS": ORB_RESULTS,
        "ORB_TIMES": ORB_TIMES,
        "HOUGH_RESULTS": HOUGH_RESULTS,
        "HOUGH_TIMES": HOUGH_TIMES,
    }

def dan_map_benchmark():
    """
    Sanity check merge using three sets of pgm maps from Dan's example
    currently fails due to minimal (non-existant?) overlap
    """
    # load maps
    gt, map0, map1, map2 = [pgm_to_numpy(BASE_PGM_MAP_PATH+filename) for filename in PGM_FILENAMES]
    # map0 has different shape, resize to match
    map0 = resize_map(map0, dsize=gt.shape)
    # align map1 and map2 w.r.t. maps
    print(gt.shape, map0.shape, map1.shape, map2.shape)
    map1_aligned = orb_mapmerge(map0, map1)
    map01 = combine_aligned_maps(map0, map1_aligned)
    fig, (ax0, ax1, ax2, ax3, ax4) = plt.subplots(1, 5)
    ax0.imshow(gt, cmap="gray")
    ax0.set_title("Ground Truth")
    ax1.imshow(map0, cmap="gray")
    ax1.set_title("Map Part 1")
    ax2.imshow(map1, cmap="gray")
    ax2.set_title("Map Part 2")
    ax3.imshow(map1_aligned, cmap="gray")
    ax3.set_title("Map Part 2 - Attempted Alignment")
    ax4.imshow(map01, cmap="gray")
    ax4.set_title("Attempted Merge")
    plt.show()
    # map2_aligned = sift_mapmerge(map01, map2)
    # map012 = combine_aligned_maps(map01, map2_aligned)
    # plt.imshow(map0, cmap="gray")
    # plt.title("map0")
    # plt.show()
    # plt.imshow(map1, cmap="gray")
    # plt.title("map1")
    # plt.show()
    # plt.imshow(map2, cmap="gray")
    # plt.title("map2")
    # plt.show()
    # plt.imshow(map012, cmap="gray")
    # plt.title("map012")
    # plt.show()
    return

if __name__ == "__main__":
    # paper_benchmark()

    dan_map_benchmark()

    
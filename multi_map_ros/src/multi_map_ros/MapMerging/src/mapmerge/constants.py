from pathlib import Path
_package_root = str(Path(__file__).parent.resolve())

# map values (float)
OCCUPIED_FLOAT = 0
FREE_FLOAT = 1
UNKNOWN_FLOAT = 0.5
# map values (uint8)
FREE = 255
OCCUPIED = 0
UNKNOWN = 127

# data paths
TRAIN_FILENAMES = ["robot0_cast.txt", "robot1_cast.txt", "robot2_cast.txt", "robot3_cast.txt", "LongRun6.txt", "LongRun7.txt", "LongCorridor2.txt", "LongCorridor1.txt"]
TEST_FILENAMES = ["intel.txt", "intel1000.txt"]
BASE_MERCER_MAP_PATH = _package_root+"/Data/Mercer/"
BASE_PGM_MAP_PATH = _package_root+"/Data/PGM/"

PGM_FILENAMES = ["full_map.pgm", "map_part0.pgm", "map_part1.pgm", "map_part2.pgm"]
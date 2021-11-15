from Communications.strategy import Evaluation, Scinario, Full_Robot, DAG_Robot
from hough_sift_mapmerge import experiment, TRAIN_FILENAMES

# Test Maps
m1, m2 = TRAIN_FILENAMES[0], TRAIN_FILENAMES[1]

robots = [Full_Robot, DAG_Robot]

if __name__ == "__main__":
    for robot in robots:
        scene = Scinario(str(robot("", "").__class__.__name__), m1, m2, robot)
        results = Evaluation(scene).start()
        print(results)
    

""" TODO
- Add metrics in evalutation function
- Create the full map transfer non-stategy
- Test mock network -> (maybe try the unused Network implimentation)
- Create the Equality and Error Correction startegy
- Create the DAG strategy
"""
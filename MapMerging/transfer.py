from Communications.strategy import Evaluation, Scinario
from Robots.full import Full_Robot
from Robots.merkle_dag import DAG_Robot
from constants import TRAIN_FILENAMES

# Test Maps
m1, m2 = TRAIN_FILENAMES[0], TRAIN_FILENAMES[1]

robots = [DAG_Robot, Full_Robot]

if __name__ == "__main__":
    for robot in robots:
        scene = Scinario(str(robot().__class__.__name__), m1, m2, robot)
        results = Evaluation(scene).start()
        print(results)
    

""" TODO
- Create the Equality and Error Correction startegy
- Create the DAG strategy
"""
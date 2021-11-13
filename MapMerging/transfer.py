from hough_sift_mapmerge import TRAIN_FILENAMES, load_mercer_map
from Strategies.strategy import Robot, Map, Strategy, Evaluation

# Test Maps
m1, m2 = TRAIN_FILENAMES[0], TRAIN_FILENAMES[1]

if __name__ == "__main__":
    
    rob_a = Robot('A', Map(load_mercer_map(m1)))
    rob_b = Robot('B', Map(load_mercer_map(m2)))
    strat = Strategy("Full Map", rob_a, rob_b)

    e = Evaluation(strat).start()
    print(e)
    

""" TODO
- Add metrics in evalutation function
- Create the full map transfer non-stategy
- Test mock network -> (maybe try the unused Network implimentation)
- Create the Equality and Error Correction startegy
- Create the DAG strategy
"""
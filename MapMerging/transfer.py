import hough_sift_mapmerge as merge
from Strategies.strategy import Robot, Map, Strategy, Evaluation

if __name__ == "__main__":
    m1, m2 = merge.TRAIN_FILENAMES[0], merge.TRAIN_FILENAMES[1]
    rob_a = Robot('A', Map(merge.load_mercer_map(m1)))
    rob_b = Robot('B', Map(merge.load_mercer_map(m2)))
    strat = Strategy("testing", rob_a, rob_b)

    e = Evaluation(strat).start()
    print(e)
    

""" TODO
- Add metrics in evalutation function
- Create the full map transfer non-stategy
- Test mock network -> (maybe try the unused Network implimentation)
- Create the Equality and Error Correction startegy
- Create the DAG strategy
"""
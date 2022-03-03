
if __name__ == '__main__':

    gridCells1 = list()
    gridCells2 = list()
    gridCells3 = list()
    gridCells4 = list()


    gridCells1.append((0, 0, 0))
    gridCells1.append((1, 0, 0))
    gridCells2.append((2, 0, 0))
    gridCells2.append((3, 0, 0))
    gridCells3.append((4, 0, 0))
    gridCells3.append((5, 0, 0))
    gridCells3.append((6, 0, 0))
    gridCells3.append((7, 0, 0))
    gridCells4.append((100, 100, 100))
    gridCells4.append((30, 20, 3))

    all_frontier_clusters = dict()
    all_frontier_clusters["one"] = gridCells1
    all_frontier_clusters["two"] = gridCells2
    all_frontier_clusters["three"] = gridCells3
    all_frontier_clusters["four"] = gridCells4


    length_dict = {key: len(value) for key, value in all_frontier_clusters.items()}
    new_list = sorted(length_dict, key=length_dict.get, reverse=True)
    print(new_list)
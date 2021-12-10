#!/usr/bin/env python3.8
import copy

import numpy as np

if __name__ == '__main__':
    gridCells = list()
    cell_threshold = 3
    gridCells.append((0, 0, 0))
    gridCells.append((1, 0, 0))
    gridCells.append((2, 0, 0))
    gridCells.append((3, 0, 0))
    gridCells.append((4, 0, 0))
    gridCells.append((5, 0, 0))
    gridCells.append((6, 0, 0))
    gridCells.append((7, 0, 0))
    gridCells.append((100, 100, 100))
    gridCells.append((30, 20, 3))

    all_frontier_clusters = dict()  # keeps track of clusters and the parent
    remaining_cells = copy.deepcopy(gridCells)  # cells remaining to be assigned to a group

    for a_cell in gridCells:
        assigned = False
        if all_frontier_clusters:
            for key in all_frontier_clusters:
                cells_in_cluster = all_frontier_clusters.get(key)
                iterable_cells = copy.deepcopy(cells_in_cluster)
                for other_cell in iterable_cells:
                    #if True in (abs(np.subtract(a_cell, other_cell)) <= cell_threshold):
                    if np.linalg.norm(np.asarray(a_cell)-np.asarray(other_cell)) <= cell_threshold:
                        if a_cell not in cells_in_cluster:
                            assigned = True
                            cells_in_cluster.append(a_cell)
                            remaining_cells.remove(a_cell)
                            all_frontier_clusters[key] = cells_in_cluster

                        else:
                            assigned = True

                        for potential_cell in remaining_cells:  # in gridCells
                            # if True in (abs(np.subtract(a_cell, potential_cell)) <= cell_threshold):
                            if np.linalg.norm(np.asarray(a_cell) - np.asarray(potential_cell)) <= cell_threshold:
                                if potential_cell not in cells_in_cluster:
                                    cells_in_cluster.append(potential_cell)
                                    remaining_cells.remove(potential_cell)
                                    all_frontier_clusters[key] = cells_in_cluster

            if not assigned:
                first_list = list()
                first_list.append(a_cell)
                remaining_cells.remove(a_cell)
                all_frontier_clusters[a_cell] = first_list


        else:
            first_list = list()
            first_list.append(a_cell)
            remaining_cells.remove(a_cell)
            all_frontier_clusters[a_cell] = first_list

    print(str(all_frontier_clusters))

    all_values = all_frontier_clusters.values()

    max_value = max(all_values)
    print(max_value)

    max_key, max_value = max(all_frontier_clusters.items(), key=lambda x: len(set(x[1])))
    print(max_key)
    print(max_value)
# MQP_MapMerging

WPI MQP 2021 - Multi-Robot Indoor Mapping

Connor Mclaughlin, Tyler Ferrara, Peter Nikopoulos

Data contains the set of maps from UC Mercered dataset, see the readme contained on how to use them.

## Initial Experiements:

Hough Line Detection Merge (Carpin et al. 2008)
SIFT/ORB Keypoint-Based Homography Merge

## Usage

It's advised to build and test these experiments in a static environment.
Therefore, we encourage execution within a contianer.

```shell
$ docker build . -t mapmerge-experiment
$ docker run mapmerge-experiment
```
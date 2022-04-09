#!/usr/bin/env python3
import argparse
import random as rand

parser = argparse.ArgumentParser(description="Parse networking arguments");
parser.add_argument('--type', type=str, default='Khepera', help="Type of the fake robot");
parser.add_argument('-n', '--number', type=int, default=3, help='IP of argos server.')
parser.add_argument('-i', '--iterations', type=int, default=100, help='Number of iterations to generate.')
args= parser.parse_args()

with open('test.csv', 'w') as f:
	for i in range(args.number):
		f.write(args.type + "_" + str(i+1))
		if i+1 != args.number:
			f.write(',')

	f.write("\n")
	for j in range(args.iterations):
		for i in range(args.number):
			pos = [str(rand.uniform(-.9, .9)) for x in range(2)]
			pos.append("0.01")
			rot = "0 0 0 1"
			f.write(" ".join(pos)+" " +rot)
			if i+1 != args.number:
				f.write(',')
		if j+1 != args.iterations:
			f.write("\n")

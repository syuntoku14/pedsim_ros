#!/usr/bin/env python

import pdb
import png
import cv2
import numpy as np
import argparse
import os
import xml.etree.ElementTree as xml

parser = argparse.ArgumentParser()
parser.add_argument('filepath', type=str)
args = parser.parse_args()

if not os.path.isfile(args.filepath):
    raise ValueError(args.filepath + " doesn't exist")

filepath = args.filepath
print("Reading file: " + filepath)

tree = xml.parse(filepath)
root = tree.getroot()

obstacles = []  # (xmin, xmax, ymin, ymax)
for obstacle in root.findall("obstacle"):
    obstacle = np.array((float(obstacle.get("x1")), float(obstacle.get(
        "x2")), float(obstacle.get("y1")), float(obstacle.get("y2"))))
    obstacles.append(obstacle)

obstacles = np.array(obstacles).astype(np.int)
# set min to 0
obstacles[:, :2] -= np.min(obstacles[:, 0])
obstacles[:, 2:] -= np.min(obstacles[:, 2])

xmin = np.min(obstacles[:, 0])
xmax = np.max(obstacles[:, 1])
ymin = np.min(obstacles[:, 2])
ymax = np.max(obstacles[:, 3])

print("Map dimensions: (%f, %f) -- (%f, %f)" % (xmin, ymin, xmax, ymax))
width = xmax + 1
height = ymax + 1

grid = np.zeros((height, width), np.uint8)

for obstacle in obstacles:
    grid = cv2.line(
        grid, (obstacle[0], obstacle[2]), (obstacle[1], obstacle[3]), 255, 3)

outputFilename = filepath
outputFilename = outputFilename.replace(".xml", ".png")

print "Writing output to " + outputFilename
f = open(outputFilename, 'wb')      # binary mode is important
w = png.Writer(width, height, greyscale=True)
w.write(f, grid)
f.close()

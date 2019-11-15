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

obstacles = []
for obstacle in root.findall("obstacle"):
    xlimits = float(obstacle.get("x1")), float(obstacle.get("x2"))
    ylimits = float(obstacle.get("y1")), float(obstacle.get("y2"))
    obstacle = (np.floor(min(xlimits)), np.floor(min(ylimits)),
                np.ceil(max(xlimits)), np.ceil(max(ylimits)))
    obstacles.append(obstacle)

obstacles = np.array(obstacles).astype(np.int)
print(obstacles)
xmin = np.min(obstacles[:, 0])
ymin = np.min(obstacles[:, 1])
xmax = np.max(obstacles[:, 2])
ymax = np.max(obstacles[:, 3])

print "Map dimensions: (%f, %f) -- (%f, %f)" % (xmin, ymin, xmax, ymax)
width = xmax + 1
height = ymax + 1

grid = numpy.uint8(numpy.zeros((height, width)))

for obstacle in obstacles:
    for x in xrange(obstacle[0], obstacle[2] + 1):
        for y in xrange(obstacle[1], obstacle[3] + 1):
            grid[x, y] = 255

outputFilename = filepath
outputFilename = outputFilename.replace(".xml", ".png")

print "Writing output to " + outputFilename
f = open(outputFilename, 'wb')      # binary mode is important
w = png.Writer(width, height, greyscale=True)
w.write(f, grid)
f.close()

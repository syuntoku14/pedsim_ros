#!/usr/bin/env python

import pdb
import png
import cv2
import argparse
import os
import yaml
import numpy as np
import xml.etree.ElementTree as xml

parser = argparse.ArgumentParser()
parser.add_argument('filepath', type=str)
args = parser.parse_args()

if not os.path.isfile(args.filepath):
    raise ValueError(args.filepath + " doesn't exist")

filepath = os.path.abspath(args.filepath)
print("Reading file: " + filepath)

tree = xml.parse(filepath)
root = tree.getroot()

obstacles = []  # (xmin, xmax, ymin, ymax)
for obstacle in root.findall("obstacle"):
    obstacle = np.array((float(obstacle.get("x1")), float(obstacle.get(
        "x2")), float(obstacle.get("y1")), float(obstacle.get("y2"))))
    obstacles.append(obstacle)

obstacles = (np.array(obstacles) * 2).astype(int)  # 1m == 2pixels
xmin = np.min(obstacles[:, :2]) / 2.0
ymin = np.min(obstacles[:, 2:]) / 2.0

# set min to 0
obstacles[:, :2] -= np.min(obstacles[:, 0])
obstacles[:, 2:] -= np.min(obstacles[:, 2])

xmax = np.max(obstacles[:, 1])
ymax = np.max(obstacles[:, 3])

width = xmax + 1
height = ymax + 1

grid = np.full((height, width), 255, np.uint8)

for obstacle in obstacles:
    grid = cv2.line(
        grid, (obstacle[0], obstacle[2]), (obstacle[1], obstacle[3]), 0, 3)

grid = cv2.flip(grid, 0)

outputFilename = filepath
outputFilename = outputFilename.replace(".xml", ".png")

print "Writing output to " + outputFilename
with open(outputFilename, 'wb') as f:
    w = png.Writer(width, height, greyscale=True)
    w.write(f, grid)

origin = [float(xmin), float(ymin), 0]
map_yaml = {"image":outputFilename, "resolution":0.5,
    "origin": origin, "occupied_thresh": 0.65,
    "free_thresh": 0.196, "negate": 0}

yamlFilename = filepath
yamlFilename = yamlFilename.replace(".xml", ".yaml")
print "Writing yaml to " + yamlFilename
with open(yamlFilename, 'w') as f:
    documents = yaml.dump(map_yaml, f)
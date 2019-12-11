#!/usr/bin/env python

import pdb
import cv2
import argparse
import os
import yaml
import numpy as np
import xml.etree.ElementTree as xml
import pdb
from copy import deepcopy

parser = argparse.ArgumentParser()
parser.add_argument('filepath', type=str)
parser.add_argument('--safe_distance', type=float, default=2.0)
args = parser.parse_args()

resolution = 0.1
max_velocity = 10  # 1.0 meter / sec
unknown_space = 30  # expand map 30x30m

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

obstacles = (np.array(obstacles) / resolution).astype(int)  # 1m == 1/resolution pixels
xmin = np.min(obstacles[:, :2]) * resolution
ymin = np.min(obstacles[:, 2:]) * resolution

# set min to 0
obstacles[:, :2] -= np.min(obstacles[:, 0])
obstacles[:, 2:] -= np.min(obstacles[:, 2])

xmax = np.max(obstacles[:, 1])
ymax = np.max(obstacles[:, 3])

width = xmax + 1
height = ymax + 1

grid = np.zeros((height, width), np.uint8)

for obstacle in obstacles:
    grid = cv2.line(
        grid, (obstacle[0], obstacle[2]), (obstacle[1], obstacle[3]), 255, int(1 / resolution))

grid = cv2.flip(grid, 0)

# draw velocity raw map
vel_grid = deepcopy(grid)
vel_grid = vel_grid.astype(np.float)
kernel = np.ones((3,3),np.uint8)

for i in range(1, int(args.safe_distance / resolution)):
    tmp = cv2.dilate(grid, kernel=kernel, iterations=i)
    vel_grid += tmp

vel_grid = vel_grid / np.max(vel_grid) * max_velocity 
vel_grid = np.abs(vel_grid - max_velocity + np.clip(grid, 0, 100))
vel_grid = np.clip(vel_grid, 0, 100)
vel_grid = vel_grid.astype(np.int8)

padding = int(unknown_space / resolution)
vel_grid = np.pad(vel_grid, (padding, padding), 'constant', constant_values=(-1, -1))

# save map
outputFilename = filepath
outputFilename = outputFilename.replace(".xml", ".png")

print "Writing output to " + outputFilename
cv2.imwrite(outputFilename, cv2.bitwise_not(grid))

origin = [float(xmin), float(ymin), 0]
map_yaml = {"image":outputFilename, "resolution":resolution,
    "origin": origin, "occupied_thresh": 0.65,
    "free_thresh": 0.196, "negate": 0}

yamlFilename = filepath
yamlFilename = yamlFilename.replace(".xml", ".yaml")
print "Writing yaml to " + yamlFilename
with open(yamlFilename, 'w') as f:
    documents = yaml.dump(map_yaml, f)


# save velocity map
outputFilename = filepath
outputFilename = outputFilename.replace(".xml", "_vel.png")

print "Writing velocity map to " + outputFilename
cv2.imwrite(outputFilename, vel_grid)

origin = [-unknown_space + float(xmin), -unknown_space + float(ymin), 0]
map_yaml = {"image":outputFilename, "resolution":resolution,
    "occupied_thresh": 0.65, "free_thresh": 0.196, "negate": 0,
    "origin": origin, "mode": "raw"}

yamlFilename = filepath
yamlFilename = yamlFilename.replace(".xml", "_vel.yaml")
print "Writing velocity yaml to " + yamlFilename
with open(yamlFilename, 'w') as f:
    documents = yaml.dump(map_yaml, f)



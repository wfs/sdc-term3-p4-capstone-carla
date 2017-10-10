#!/usr/bin/env python
import matplotlib.pyplot as plt
import csv
from math import cos, sin

wp_x = []
wp_y = []
path = './data/wp_yaw_const.csv'


def closest_waypoint(waypoints, pos):
    best_distance = float('inf')
    best_wp_index = 0
    best_wp = None

    for i, wp in enumerate(waypoints):
        dist = ((pos[0] - wp[0]) ** 2 + (pos[1] - wp[1]) ** 2) ** 0.5
        if dist < best_distance:
            best_distance = dist
            best_wp, best_wp_index = wp, i

    yaw = waypoints[best_wp_index][2]
    nx = pos[0] - waypoints[best_wp_index][0]
    ny = pos[1] - waypoints[best_wp_index][1]

    # Check if waypoint is ahead of stop line
    ahead = nx * cos(0 - yaw) - ny * sin(0 - yaw)
    if ahead < 0:
        best_wp_index -= 1
    print (best_wp_index)
    return best_wp_index


def all_stop_line_closest_wp(waypoints, traffic_light_posts):
    stop_wp = []
    stop_x = []
    stop_y = []
    for pos in traffic_light_posts:
        print (pos)
        best_index = closest_waypoint(waypoints, pos)
        # print (best_index)
        stop_x.append(waypoints[best_index][0])
        stop_y.append(waypoints[best_index][1])

    return stop_x, stop_y


traffic_light_pos = [[1148.56, 1184.65], \
                     [1559.2, 1158.43], \
                     [2122.14, 1526.79], \
                     [2175.237, 1795.71], \
                     [1493.29, 2947.67], \
                     [821.96, 2905.8], \
                     [161.76, 2303.82], \
                     [351.84, 1574.65]]
# IMPORT WAYPOINTS
waypoints = []
with open(path, 'r') as file:
    reader = csv.reader(file, delimiter=',')
    for i in reader:
        # print (i)
        x, y, yaw = float(i[0]), \
                    float(i[1]), \
                    float(i[3])
        waypoints.append([x, y, yaw])
        wp_x.append(x)
        wp_y.append(y)

# print (waypoints)

# IMPORT TRAFFIC LIGHT POSITIONS
light_x = []
light_y = []
for pos in traffic_light_pos:
    light_x.append(pos[0])
    light_y.append(pos[1])
# FIND CLOSET WAYPOINT TO TRAFFIC LIGHT
stop_x, stop_y = all_stop_line_closest_wp(waypoints, traffic_light_pos)

# print ("X stops", stop_x)
# print ("Y stops", stop_y)

# print ("Waypoint Index: ", closest_waypoint(waypoints, traffic_light_pos[0]))
# print ("Waypoint Index: ", closest_waypoint(waypoints, traffic_light_pos[1]))

plt.plot(wp_x, wp_y, label='Waypoints Path')
plt.plot(light_x, light_y, 'ro', label='Traffic Light Posts')
plt.plot(stop_x, stop_y, 'g^', label='Stopping Waypoints')
plt.plot
plt.xlabel('x')
plt.ylabel('y')
plt.title('Simulator path')
plt.legend()
plt.show()

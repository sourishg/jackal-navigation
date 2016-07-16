import numpy as np
from matplotlib import pyplot as plt

elas = []
pcl = []
scan = []

data = open('/home/cobot-m/catkin_ws/src/jackal_nav/data/dmap_time.txt', 'r')

for line in data:
  elas.append(float(line.strip()))

data.close()

data = open('/home/cobot-m/catkin_ws/src/jackal_nav/data/point_cloud_time.txt', 'r')

for line in data:
  pcl.append(float(line.strip()))

data.close()

data = open('/home/cobot-m/catkin_ws/src/jackal_nav/data/obstacle_scan_time.txt', 'r')

for line in data:
  scan.append(float(line.strip()))

data.close()

#Stack the data
plt.figure()
plt.hist([elas,pcl,scan], stacked=True)
plt.show()
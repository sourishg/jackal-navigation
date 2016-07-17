import matplotlib.pyplot as plt
import math

data = open('/home/cobot-m/catkin_ws/src/jackal_nav/data/dmap_time.txt', 'r')

dmap = []

for line in data:
  dmap.append(float(line.strip()))

data.close()

data = open('/home/cobot-m/catkin_ws/src/jackal_nav/data/obstacle_scan_time.txt', 'r')

scan = []

for line in data:
  scan.append(float(line.strip()))

data.close()

data = open('/home/cobot-m/catkin_ws/src/jackal_nav/data/point_cloud_time.txt', 'r')

pcl = []

for line in data:
  pcl.append(float(line.strip()))

data.close()

fps = []

for i in range(len(scan)):
  if (len(pcl) == 0):
    fps.append((scan[i] + dmap[i]))
  else:
    fps.append((scan[i] + dmap[i] + pcl[i]))

plt.hist([f for f in fps])
plt.show()

print "AVG: " + str(sum(fps) / len(fps))

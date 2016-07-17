import matplotlib.pyplot as plt
import math

data = open('/home/cobot-m/catkin_ws/src/jackal_nav/data/point_cloud_time.txt', 'r')

fps = []

for line in data:
  fps.append(float(line.strip()))

data.close()

plt.hist([f for f in fps])
plt.show()

print "AVG: " + str(sum(fps) / len(fps))

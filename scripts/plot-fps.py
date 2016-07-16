import matplotlib.pyplot as plt
import math

data = open('/home/cobot-m/catkin_ws/src/jackal_nav/data/dmap_time.txt', 'r')

fps = []

for line in data:
  fps.append(int(1.0 / float(line.strip())))

data.close()

plt.hist([f for f in fps], bins = range(200))
plt.show()

print "AVG: " + str(sum(fps) / len(fps))

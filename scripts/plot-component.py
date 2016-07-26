import matplotlib.pyplot as plt
import math
import sys

data = open(sys.argv[1], 'r')

times = []

for line in data:
  times.append(float(line.strip()))

data.close()

plt.hist([t for t in times])
plt.show()

print "AVG: " + str(sum(times) / len(times))

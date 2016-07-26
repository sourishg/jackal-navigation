import numpy as np
from matplotlib import pyplot as plt
import sys

components = []
for i in range(len(sys.argv)-1):
	data = open(sys.argv[i+1], 'r')
	times = []
	for line in data:
		times.append(float(line.strip()))
	components.append(times)

plt.figure()
plt.hist([x for x in components], stacked=True)
plt.show()
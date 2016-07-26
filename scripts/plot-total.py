import matplotlib.pyplot as plt
import math
import sys

components = []
for i in range(len(sys.argv)-1):
	data = open(sys.argv[i+1], 'r')
	times = []
	for line in data:
		times.append(float(line.strip()))
	components.append(times)

total = []
for i in range(len(components[0])):
	x = 0
	for j in range(len(sys.argv)-1):
		x += components[j][i]
	total.append(x)

plt.hist([f for f in total])
plt.show()

print "AVG: " + str(sum(total) / len(total))

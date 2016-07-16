import numpy as np
from matplotlib import pyplot as plt

# create 3 data sets with 1,000 samples
mu, sigma = 0, 0.04

x1 = mu + sigma*np.random.randn(1000,1)
x2 = mu + sigma*np.random.randn(1000,1)
x3 = mu + sigma*np.random.randn(1000,1)

#Stack the data
plt.figure()
plt.hist([x1,x2,x3], stacked=True)
plt.show()
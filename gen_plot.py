import sys
import matplotlib.pyplot as plt
import numpy as np

left_x = np.load('left_x.npy')
left_y = np.load('left_y.npy')
center_x = np.load('center_x.npy')
center_y = np.load('center_y.npy')
right_x = np.load('right_x.npy')
right_y = np.load('right_y.npy')

fig, axes = plt.subplots(nrows=1, ncols=3)
ax0, ax1, ax2 = axes.flat

ax0.scatter(left_x, left_y, s=10, alpha=1.0, edgecolors='none')
ax0.set_title('Left')
ax0.set_xlabel('x')
ax0.set_ylabel('y')

ax1.scatter(center_x, center_y, s=10, alpha=1.0, edgecolors='none')
ax1.set_title('Center')
ax1.set_xlabel('x')
ax1.set_ylabel('y')

ax2.scatter(right_x, right_y, s=10, alpha=1.0, edgecolors='none')
ax2.set_title('Right')
ax2.set_xlabel('x')
ax2.set_ylabel('y')

print left_x[0]
print left_y[0]

fig.set_figwidth(20)
plt.savefig('plot1.pdf')
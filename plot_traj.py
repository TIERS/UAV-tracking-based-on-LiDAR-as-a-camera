from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
import os





data1 = np.load("./traj.npy",allow_pickle=True)
num=data1.size
datax = data1[:, 0]
datay = data1[:, 1]
dataz = data1[:, 2]
numx=datax.size
step =  np.arange(0,numx,1) 


# new a figure and set it into 3d
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

### set figure information
ax.set_title("3D_Curve")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

## draw the figure, the color is r = read
figure = ax.plot(datax, datay, dataz, c='r')
plt.ylim(-1,1)
plt.show()

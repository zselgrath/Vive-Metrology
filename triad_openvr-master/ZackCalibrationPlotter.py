import triad_openvr as vr
import pylab as plt
from mpl_toolkits import mplot3d
import numpy as np
from matplotlib import gridspec
from pyquaternion import Quaternion
from scipy.optimize import leastsq

v = vr.triad_openvr()
data = v.devices["controller_1"].sample(2500, 250)
samples = 200

size = len(data.x)

for i in range(1,size):
    data.x[i] = data.x[i] - data.x[0]
    data.y[i] = data.y[i] - data.y[0]
    data.z[i] = data.z[i] - data.z[0]

data.x[0] = 0
data.y[0] = 0
data.z[0] = 0

#at this point, all position data is relative and can be operated 1

#tipOffset = Quaternion(0, 0, 0, 0.0435)
tipOffset = Quaternion(0, 0, 0, 0)
for i in range(0, size):
    #poseVect = v.devices["tracker_1"].get_pose_quaternion()
    poseQuat = Quaternion(data.r_w[i], data.r_x[i], data.r_y[i], data.r_z[i])
    tipCorrected = (poseQuat*tipOffset)*poseQuat.inverse
    data.x[i] = data.x[i] + tipCorrected.vector[0]
    data.y[i] = data.y[i] + tipCorrected.vector[1]
    data.z[i] = data.z[i] + tipCorrected.vector[2]
11
iPoint = [0, 0, 0]
fPoint = [0, 0, 0]

iPoint[0] = np.average(data.x[:samples])
iPoint[1] = np.average(data.y[:samples])
iPoint[2] = np.average(data.z[:samples])

fPoint[0] = np.average(data.x[size-samples:size])
fPoint[1] = np.average(data.y[size-samples:size])
fPoint[2] = np.average(data.z[size-samples:size])

print("iPoint:")
print(iPoint)
print("fPoint:")
print(fPoint)

dist = pow(pow((iPoint[0]-fPoint[0]), 2) + pow((iPoint[1]-fPoint[1]), 2) + pow((iPoint[2]-fPoint[2]), 2), .5)


fig = plt.figure(1)
#fig.title('Controller X,Y,Z Coordinate')
# set height ratios for sublots
gs = gridspec.GridSpec(3, 1)
ax0 = plt.subplot(gs[0])
line0 = ax0.plot(data.time, data.x, color='r')
ax1 = plt.subplot(gs[1], sharex=ax0)
line1 = ax1.plot(data.time, data.z, color='g')
ax2 = plt.subplot(gs[2], sharex=ax1)
line2 = ax2.plot(data.time, data.y, color='b')
print("Total:")
print(dist)
print(dist*1000/25.4)
#plt.setp(ax0.get_xticklabels(), visible=False)11
# remove last tick label for the second subplot
#yticks = ax1.yaxis.get_major_ticks()
#yticks[-1].label1.set_visible(False)

# put lened on first subplot
#ax2.legend((line0, line1, line2), ('X', 'Y', 'Z'), loc='lower left')

# remove vertical gap between subplots
plt.subplots_adjust(hspace=.0)

fig2 = plt.figure(2)
ax = plt.axes(projection='3d')
# Data for three-dimensional scattered points
# WARNING: Y AND Z ARE SWAPPED FROM NOW ON TO AGREE WITH WORLD Z UP
zdata = data.y
xdata = data.x
ydata = data.z
ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='hsv')

plt.show()

coords=np.array((data.x,data.y,data.z)).transpose()
#guess at where the center is???
p0 = [0, 0, 0, 1]
def fitfunc(p, coords):
    x0, y0, z0, R = p
    x, y, z = coords.T
    return np.sqrt((x-x0)**2 + (y-y0)**2 + (z-z0)**2)

errfunc = lambda p, x: fitfunc(p, x) - p[3]
p1, flag = leastsq(errfunc, p0, args=(coords,)) #where p1 is returned as x,y,z,r values
print(p1)

ax.scatter(p1[0], p1[2], p1[1], s=20, c='b',rasterized=True)
plt.show()
x0, z0, y0, r = p1
u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
x=np.cos(u)*np.sin(v)*r
y=np.sin(u)*np.sin(v)*r
z=np.cos(v)*r
x = x + x0
y = y + y0
z = z + z0

#3D plot of Sphere
#ax.scatter(correctX, correctY, correctZ, zdir='z', s=20, c='b',rasterized=True)
ax.plot_wireframe(x, y, z, color="r")
ax.set_xlabel('$x$ (mm)',fontsize=16)
ax.set_ylabel('\n$y$ (mm)',fontsize=16)
ax.set_zlabel('\n$z$ (mm)',fontsize=16)
plt.show()


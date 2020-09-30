import triad_openvr as vr
import pylab as plt
v = vr.triad_openvr()
data = v.devices["controller_1"].sample(1000,250)
plt.plot(data.time,data.x)
plt.title('Controller X Coordinate')
plt.xlabel('Time (seconds)')
plt.ylabel('X Coordinate (meters)')

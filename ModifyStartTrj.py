#!/usr/bin/python

import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

file = open('./resource/gait/start.txt','r')

line = file.readline();

data = []
while line:
    nums = filter(None, line.strip('\n').split(' '))
    nums = [int(item) for item in nums]
    data.append(nums)
    line = file.readline()
print nums
file.close()
dataArr = numpy.array(data)
time = dataArr[:,0]
axis = []
for i in range(1, 19):
    print i
    axis.append(dataArr[:,i].copy())

print len(time)
#tplt = time / 1.0
tplt = numpy.linspace(0, len(time), num=len(time))
offset = numpy.linspace(120000, 0,  num=len(time))

dataArr[:, 13] = dataArr[:, 13] + offset
dataArr[:, 14] = dataArr[:, 14] + offset
dataArr[:, 15] = dataArr[:, 15] + offset

plt.plot(tplt, axis[12])
plt.plot(tplt, axis[13])
plt.plot(tplt, axis[14])

plt.plot(tplt, dataArr[:,13])
plt.plot(tplt, dataArr[:,14])
plt.plot(tplt, dataArr[:,15])

plt.grid(True)
plt.show()
file = open('./resource/gait/new_start.txt', 'w')

for i in range(len(time)):
    line = "       "
    for j in range(19):
        line = line + "  " + str(dataArr[i, j])
    print line
    line = line + '\n'
    file.write(line)

file.flush()
file.close()

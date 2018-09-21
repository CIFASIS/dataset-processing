#!/usr/bin/python
import matplotlib.pyplot as plt


x1 = []
y1 = []
x2 = []
y2 = []
x3 = []
y3 = []
fr = open("/home/martin/src/definitives/ground_truth/seq4/gps.log","r")
for line in fr:
    data = line.split(" ")
    x1.append(float(data[1]))
    y1.append(float(data[2]))

fr = open("/home/martin/src/definitives/ground_truth/seq4/gps_aligned.log","r")
for line in fr:
    data = line.split(" ")
    x2.append(float(data[1]))
    y2.append(float(data[2]))


fr = open("/home/martin/src/definitives/ground_truth/seq4/sptam_short.log","r")
for line in fr:
    data = line.split(" ")
    x3.append(float(data[1]))
    y3.append(float(data[2]))

plt.figure(1)
plt.title('Trayectoria GPS')
plt.plot(x1, y1,'b')
plt.plot(x2, y2,'r')
plt.plot(x3, y3,'-g')
plt.gca().set_aspect('equal', adjustable='box')
plt.xlim(-200,200)
plt.ylim(-200,200)
plt.legend(['GPS','GPS aligned','sptam'])

plt.show()

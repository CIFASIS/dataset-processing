#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import os
angle = []
for filename in sorted(os.listdir("/home/martin/Desktop/lineas_rectas")):
  fr = open("/home/martin/Desktop/lineas_rectas/"+filename,"r")
  for line in fr:
    data = line.split(",")
    angle.append(int(data[16]))
    print len(angle)

fw = open("/home/martin/Desktop/lineas_rectas/results.log","w+")
fw.write("media = " + str(np.mean(angle)) + "\n")
fw.write("mediana = " + str(np.median(angle))+ "\n")
fw.write("desviacion_estandar = " + str(np.std(angle))+ "\n")
fw.close()
print np.mean(angle)
print np.median(angle)
print np.std(angle)
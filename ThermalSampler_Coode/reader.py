#!/bin/python3

import serial
import pandas as pd

s = serial.Serial('/dev/ttyUSB0', baudrate=115200)
num = []
temp = []
hum = []


while(True):
	try:
		x = str(s.readline(3000),'utf-8')
		if(x.find('addr') > 0):
			f = x.split()
			d = (int(eval('0x' + f[5])),int(f[6]),int(f[7]))
			num.append(d[0])
			temp.append(d[1])
			hum.append(d[2])
			print(d)
	except KeyboardInterrupt:
		df = pd.DataFrame({'id':num, 'temperature':temp, 'humidity' : hum})
		df.to_csv('output.csv')
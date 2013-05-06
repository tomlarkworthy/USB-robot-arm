from lwpr import *
from numpy import *
import numpy as np
from random import *
from math import *

rand = Random()


#data derived stats
max_x = np.ones(29)*-100000
min_x = np.ones(29)* 100000

prev_data = None

with open("data.txt","r") as datafile:
	line = datafile.readline()
	while line != "":
		
		#we have spurious , at end of lines 
		data = [float(f) for f in line[:-2].split(",")]
		
		if prev_data != None:
			
			x = np.array(prev_data[0:29])
			y = np.array(data[0:8])
			
			max_x = map(max, x, max_x)
			min_x = map(min, x, min_x)

		prev_data = data
		line = datafile.readline()



model = LWPR(29,8)

x = zeros(29)
y = zeros(8)

model.init_D = 0.05*eye(29)
model.update_D = False
#model.init_alpha = 250*ones([29,29])

model.norm_in = np.array(np.array(max_x) - np.array(min_x))
print model.norm_in

prev_data = None

count = 0
with open("data.txt","r") as datafile:
	line = datafile.readline()
	while line != "":
		
		print count
		
		#we have spurious , at end of lines 
		data = [float(f) for f in line[:-2].split(",")]
		
		if prev_data != None:
			
			x = np.array(prev_data[0:29])
			y = np.array(data[0:8])
			model.update(x,y) 
		prev_data = data
		
		line = datafile.readline()
		count += 1
		
	
print model
print model.kernel

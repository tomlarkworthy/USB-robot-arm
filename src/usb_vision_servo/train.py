'''
1. train a bunch of training set (train1.txt) (train2.txt)

'''

def is_float(str):
	try:
		float(str)
		return True
	except ValueError:
		return False

from lwpr import *
from numpy import *
from random import *
from math import *

INPUT_DIM  = 8 + 5 #(8 dim homolography matrix) + (5 DOF arm - 1 DOF hand)
OUTPUT_DIM = 8  #(8 dim homolography matrix + its derivatives and double derivatives)

SET_REDUCTION = 1 # make the sets smaller jsut to speed training

model = LWPR(INPUT_DIM, OUTPUT_DIM) #input, output dimentions, correlate current state + action to new current state

R = Random()
x = zeros(INPUT_DIM)
y = zeros(OUTPUT_DIM)


#LOAD DATA FROM FILES
def load_data_in_array(filename, fill_array):
	with open(filename, "r") as f:
		for line in f:
			vals_str = line.split(",")
			vals = [float(x) for x in vals_str if is_float(x)]

			if len(vals) == INPUT_DIM + OUTPUT_DIM:
				fill_array.append(array(vals))
	return fill_array

data = load_data_in_array("data1.txt", [])
train_data = data[::2]
test_data = data[1::2]

print "normalising"

col_mean = mean(train_data, axis=0)
col_var = var(train_data, axis=0)
col_max = amax(train_data, axis=0)
col_min = amin(train_data, axis=0)
col_rge = ptp(train_data, axis=0)
print "col_mean", col_mean
print "col_var ", col_var
print "col_max ", col_max
print "col_min ", col_min
print "col_rge ", col_rge

print "train_data size = ", len(train_data)
print "test_data size = ", len(test_data)

model.norm_in = col_var[0:INPUT_DIM] + 10
model.norm_out = col_var[INPUT_DIM:INPUT_DIM+OUTPUT_DIM] + 10

print "setup"

model.update_D = True
model.init_D = 10 * eye(INPUT_DIM) #found by empirical trial and arror starting at 0.01, with updateD false
model.diag_only = False
#model.init_alpha = ones([INPUT_DIM, INPUT_DIM])
#model.kernel = "BiSquare"
model.meta = True
for iter in range(1):
	print "training..."
	for i in range((len(train_data) - 1) / SET_REDUCTION):
		x = train_data[i * SET_REDUCTION][0:INPUT_DIM]
		y = train_data[i * SET_REDUCTION][INPUT_DIM:INPUT_DIM+OUTPUT_DIM]
		model.update(x, y)

	print "trained"
	print model
	print "num_rfs ", model.num_rfs
	print "norm_in, ", model.norm_in
	print "norm_out ", model.norm_out
	print "mean_x ", model.mean_x
	print "var_x ", model.var_x
	print "rfs ", len(model.num_rfs)
	#print model.write_XML("model.xml") #check for NaNs

	print "test set performance..."
	sum = 0
	n = 0
	for i in range((len(train_data) - 1) / SET_REDUCTION):
		x = train_data[i * SET_REDUCTION][0:INPUT_DIM]
		y_real = train_data[i * SET_REDUCTION][INPUT_DIM:INPUT_DIM+OUTPUT_DIM]
		y = model.predict(x)

		#print y
		n += 1
		sum += linalg.norm(y_real - y)
	print "average squared error: ", sum / n

	print "validating..."
	sum = 0
	n = 0
	for i in range((len(test_data) - 1) / SET_REDUCTION):
		x = test_data[i * SET_REDUCTION][0:INPUT_DIM]
		y_real = test_data[i * SET_REDUCTION][INPUT_DIM:INPUT_DIM+OUTPUT_DIM]

		y = model.predict(x)
		'''
		if i%10 == 0:
			print "---"
			print y_real
			print y
		'''

		n += 1
		sum += linalg.norm(y_real - y)

	print "average squared error: ", sum / n



	x_base = x[0:INPUT_DIM-5]
	for dof in range(5):
		motor = zeros(5)
		motor[dof] = 0.5
		x = hstack([x_base, motor])
		y = model.predict(x)
		print motor
		print y


		motor[dof] = -0.5
		x = hstack([x_base, motor])
		y = model.predict(x)
		print motor
		print y

print model

#model.write_XML("model.xml")

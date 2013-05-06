#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from itertools import product
import roslib; roslib.load_manifest('usb_vision_servo')
import rospy
from std_msgs.msg import Float64MultiArray

import threading
import armcontrol
import time
import numpy as np
import wx

from scipy.optimize import *

h = [np.zeros(8), np.zeros(8), np.zeros(8), np.zeros(8)]
latest = np.zeros(8)
prev_time = time.time()
t = 0


from train import model, INPUT_DIM, OUTPUT_DIM
from record_moves import applyMove, setup_apply, setup_recording

class Controller:

	def __init__(self, model, gui):
		self.prev_state = None
		self.state = None
		self.model = model

		self.control = False
		self.gui = gui

		self.goto_thread = None


	def cost_position(self, action):
		x = np.hstack([latest, action])

		y = self.model.predict(x) #predicts the change


		error = np.zeros(len(y) - np.isnan(self.target).sum())
		y_important = np.zeros(len(y) - np.isnan(self.target).sum())

		subindex = 0
		for index in range(len(y)):
			if np.isnan(self.target[index]): continue

			error[subindex] = self.target[index] - latest[index]
			y_important[subindex] = y[index]
			subindex+=1

		cost = np.linalg.norm(error-y_important) + np.sum(np.multiply(np.power(action,2),0.1))

		#print cost

		return cost


	def go_to(self, homography):
		self.target = homography[:8]

		if self.goto_thread is not None:
			self.control = False
			print "joining"
			self.goto_thread.join()
			print "joined"


		self.control = True
		def loop():
			action=np.zeros(5)
			while self.control:
				#action = (self.cost_position, action, maxiter=1000, maxfun=10000)
				#action = fmin_bfgs(f=self.cost_position, x0=action, gtol=0.1, epsilon=0.001)
				#action = fmin_ncg(f=self.cost_position, fprime=None, x0=action, epsilon=0.001)
				(action, nfeval, rc) = fmin_tnc(self.cost_position, approx_grad=True, fprime=None, x0=action, epsilon=0.001)

				print action


				#exit()

				pred_y = model.predict(np.hstack([latest, action]))

				action[4] = 0
				action[0] = 0

				'''
				if len(np.zeros(action)) == 5:
					print "no data for command, doing a random"
					action = np.random.rand(5) - 0.5
					action[4] = 0
					action[0] = 0
				'''

				x = np.hstack([latest, action])
				y = applyMove(action)


				print " self.target ",  self.target
				print " x           ",  x
				print " pred dx     ",  pred_y
				print " actual      ",  y

				model.update(x,y)

		self.goto_thread = threading.Thread(target=loop)
		self.goto_thread.start()




	def callback(self, data):
		global h, prev_time, datafile, frame, delay, latest


		for index in range(9):
			i = index % 3
			j = index / 3
			self.gui.setCurrentHomography((i,j), data.data[index])

		prev_time = time.time()

		h = h[1:] #remove first element from list
		latest = np.array(data.data[:8])
		h.append(latest)





def getMotorCommands():
	commands = [
		armcontrol.speed_control.baseSpeed,
		armcontrol.speed_control.baseVertSpeed,
		armcontrol.speed_control.elbowSpeed,
		armcontrol.speed_control.armSpeed,
		armcontrol.speed_control.handSpeed,
		]
	return commands

class AdaptiveControlGui(wx.Panel):
	def __init__(self, parent):
		wx.Panel.__init__(self, parent)

		# create some sizers
		grid = wx.GridBagSizer(hgap=0, vgap=0)

		# A multiline TextCtrl - This is here to show how the events work in this program, don't pay too much attention to it
		#self.logger = wx.TextCtrl(self, size=(400,55), style=wx.TE_MULTILINE | wx.TE_READONLY)
		#grid.Add(self.logger, pos=(4,0), span=(1,6))

		#GO button
		self.addBoundButton("GO!", self.go, grid, (0,0), (2,6), (300,100))

		#up down left right zUp zDown

		self.homography_labels_current = {}
		self.homography_labels_desired = {}
		self.current_value = {}
		self.desired_value = {}

		for i,j in product(range(3), range(3)):
			self.addHomographyPanel(coords=(i,j), grid=grid, pos=(i+2,j), span=(1,1))

		self.SetSizerAndFit(grid)

		self.timer = wx.Timer(self)
		self.Bind(wx.EVT_TIMER, self.poll, self.timer)
		self.timer.Start(1000)  # x100 milliseconds

	def setControllor(self, controller):
		self.controller = controller

	def addHomographyPanel(self, coords, grid, pos, span=(1,1)):
		panel = wx.Panel(self)
		panel_grid = wx.GridBagSizer(hgap=0, vgap=0)

		#button_plus = wx.ToggleButton(panel, label="+", size=(50,50))
		#button_neg  = wx.ToggleButton(panel, label="-", size=(50,50))

		#self.Bind(wx.EVT_TOGGLEBUTTON, lambda evt: self.plus(evt, coords), button_plus)
		#self.Bind(wx.EVT_TOGGLEBUTTON, lambda evt: self.neg(evt, coords), button_neg)

		self.homography_labels_current[coords] = wx.TextCtrl(panel, size=(100,50))
		self.homography_labels_desired[coords] = wx.TextCtrl(panel, size=(100,50))

		panel_grid.Add(self.homography_labels_current[coords], pos=(1,1), span=(1,2))
		panel_grid.Add(self.homography_labels_desired[coords], pos=(2,1), span=(1,2))

		#panel_grid.Add(button_plus, pos=(3,1), span=(1,1))
		#panel_grid.Add(button_neg , pos=(3,2), span=(1,1))

		panel.SetSizerAndFit(panel_grid)

		grid.Add(panel, pos=pos, span=span)

	def addBoundButton(self, label, callback, grid, pos, span=(1,1), size=(50,50)):
		button = wx.ToggleButton(self, label=label, size=size)
		self.Bind(wx.EVT_TOGGLEBUTTON, callback, button)
		grid.Add(button, pos=pos, span=span)

	def plus(self, evt, coords):
		print "plus ", coords

	def neg(self, evt, coords):
		print "neg ", coords

	def setCurrentHomography(self, coords, value):
		self.current_value[coords] = value

	def setDesiredHomography(self, coords, value):
		self.desired_value[coords] = value

	def go(self, evt):
		'''user pressing go, we should try to work out the desired homogrph and travel there'''

		desired_homography = np.zeros(9)


		#fill in missing desired values with the current values
		for coord, value in self.current_value.items():

			#get the desired value out the text box and assign it to the numerical model
			#if we can;t (its not there or poorly formatted)
			#then we ignore
			try:
				self.desired_value[coord] = float(self.homography_labels_desired[coord].GetValue())
			except:
				self.desired_value[coord] = np.NaN

			#self.homography_labels_desired[coord].SetValue("%s"%self.desired_value[coord])


		for index in range(9):
			i = index % 3
			j = index / 3
			try:
				desired_homography[index] = self.desired_value[(i,j)]
			except:
				desired_homography[index] = np.NaN

		print "go to ", desired_homography
		self.controller.go_to(desired_homography)

		self.poll(None)

	def poll(self, event):
		for coord, value in self.current_value.items():
			self.homography_labels_current[coord].SetValue("%s"%value)

		self.Refresh()

app = None
def setup_gui():
	global app
	app = wx.App(False)
	frame = wx.Frame(None)
	frame.SetWindowStyle( frame.GetWindowStyle()| wx.STAY_ON_TOP )
	panel = AdaptiveControlGui(frame)
	frame.Show()
	return panel


def run_gui():
	app.MainLoop()

if __name__ == "__main__":
	print "adaptive armcontrol"

	armcontrol.setup_gui()
	gui = setup_gui()

	setup_recording("adaptive_control.txt")

	rospy.init_node('listener', anonymous=True)

	setup_apply()
	controller = Controller(model, gui)
	gui.setControllor(controller)
	rospy.Subscriber("homography", Float64MultiArray, controller.callback)

	threading.Thread(target=rospy.spin).start()
	threading.Thread(target=armcontrol.run_gui).start()
	threading.Thread(target=run_gui).start()
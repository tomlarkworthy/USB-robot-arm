#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import roslib; roslib.load_manifest('usb_vision_servo')
import rospy
from std_msgs.msg import Float64MultiArray

import time
import math

import wx

import numpy as np

import threading

import armcontrol

h = [np.zeros(8), np.zeros(8), np.zeros(8), np.zeros(8)]
prev_time = time.time()

			
t = 0

delay = 5
frame = 0

inmove = False
record = False

random = False

#apply a move, and waits till the camera settles before returning the resultant homography settles
def applyMove(motor_cmds, step=400):
	global h, prev_time, datafile, frame, delay, inmove, record

	print inmove

	if inmove: #doesn;t occur if driven by GUI thread
		print "error, in move already"
		return

	inmove = True

	initial = h[3]
	print "initial", initial

	frame = 0
	h = [np.zeros(8), np.zeros(8), np.zeros(8), np.zeros(8)]

	armcontrol.speed_control.step = step

	armcontrol.speed_control.setBaseSpeed(motor_cmds[0])
	armcontrol.speed_control.setBaseVertSpeed(motor_cmds[1])
	armcontrol.speed_control.setElbowSpeed(motor_cmds[2])
	armcontrol.speed_control.setArmSpeed(motor_cmds[3])
	armcontrol.speed_control.setHandSpeed(motor_cmds[4])

	time_started = time.time()

	while((prev_time - time_started) < 0.2): #100ms min wait
		time.sleep(0.001)

	settled = False

	while not settled and (time.time()-time_started)<2:
		latest = h[3]
		settled = True
		for prev in h[0:2]:
			if np.linalg.norm(prev - latest) > 5:
				settled = False
		time.sleep(0.001)

	if record:
		np.concatenate([initial, motor_cmds, (latest-initial)]).tofile(datafile, sep=",")
		datafile.write("\n")

	#check homography healthy
	print "h", latest[2], ",",latest[5]


	print "move done"
	inmove = False


	print "latest", latest

	return (latest-initial)


def callback(data):
	
	global h, prev_time, datafile, frame, delay

	prev_time = time.time()

	h.append(np.array(data.data[:8]))
	h = h[1:] #remove first element from list


class ArmControl(wx.Panel):
	def __init__(self, parent):
		wx.Panel.__init__(self, parent)

		# create some sizers
		grid = wx.GridBagSizer(hgap=0, vgap=0)

		# A multiline TextCtrl - This is here to show how the events work in this program, don't pay too much attention to it
		self.logger = wx.TextCtrl(self, size=(400,55), style=wx.TE_MULTILINE | wx.TE_READONLY)
		grid.Add(self.logger, pos=(4,0), span=(1,6))

		#up down left right zUp zDown
		self.addBoundButton("b←",  self.baseLeft , grid, (3,0))
		self.addBoundButton("b→",  self.baseRight, grid, (3,2))

		self.addBoundButton("b↑",  self.baseUp,   grid, (2,1))
		self.addBoundButton("b↓",  self.baseDown, grid, (3,1))

		self.addBoundButton("e↑",  self.elbowUp,   grid, (2,4))
		self.addBoundButton("e↓",  self.elbowDown, grid, (3,4))

		self.addBoundButton("a↑",  self.armUp,   grid, (2,0))
		self.addBoundButton("a↓",  self.armDown, grid, (2,2))

		self.addBoundButton("h←→", self.handOpen, grid, (3,5))
		self.addBoundButton("h→←", self.handClose, grid, (3,3))

		self.addBoundButton("L",   self.light, grid,   (2,3))

		self.SetSizerAndFit(grid)

		self.timer = wx.Timer(self)
		self.Bind(wx.EVT_TIMER, self.poll, self.timer)
		self.timer.Start(1000)  # x100 milliseconds

	def addBoundButton(self, label, callback, grid, pos, span=(1,1), size=(50,50)):
		button = wx.ToggleButton(self, label=label, size=size)
		self.Bind(wx.EVT_TOGGLEBUTTON, callback, button)
		grid.Add(button, pos=pos, span=span)

	def processMoveButton(self, button, cmd, cmd_param):
		#if button.GetValue() :
		#button.SetBackgroundColour(wx.Color(0,200,0))
		cmd(cmd_param)
	#else :
	#	button.SetBackgroundColour(None)
	#	cmd(0)

	def baseLeft(self, event):
		#self.processMoveButton(event.EventObject, moveBase, 'cc')
		applyMove([0.6,0,0,0,0])

	def baseRight(self, event):
		#self.processMoveButton(event.EventObject, moveBase, 'cl')
		applyMove([-0.6,0,0,0,0])

	def baseUp(self, event):
		applyMove([0,0.6,0,0,0])

	def baseDown(self, event):
		applyMove([0,-0.6,0,0,0])

	def elbowUp(self, event):
		applyMove([0,0,0.6,0,0])

	def elbowDown(self, event):
		applyMove([0,0,-0.6,0,0])

	def armUp(self, event):
		applyMove([0,0,0,0.7,0])

	def armDown(self, event):
		applyMove([0,0,0,-0.8,0])

	def handOpen(self, event):
		applyMove([0,0,0,0,0.4])

	def handClose(self, event):
		applyMove([0,0,0,0,-0.4])

	def light(self, event):
		self.processMoveButton(event.EventObject, light, 'on')

	def poll(self, event):
		print "h", h[3]
		if h[3][2] > 280:
			print "too high ", "h", h[3][2], ",",h[3][5]
			self.baseDown(None)
		elif h[3][2] < 50:
			print "too low", "h", h[3][2], ",",h[3][5]
			self.baseUp(None)
		elif h[3][5] >300:
			print "too left", "h", h[3][2], ",",h[3][5]
			self.baseRight(None)
		elif h[3][5] < -40:
			print "too right", "h", h[3][2], ",",h[3][5]
			self.baseLeft(None)
		elif random:
			command = np.random.rand(5)*2 - 1
			print command
			command[4] = 0
			command[0] = 0

			applyMove(command)




		self.Refresh()


def setup_apply():
	global datafile, random
	rospy.Subscriber("homography", Float64MultiArray, callback)

	threading.Thread(target=rospy.spin).start()


def setup_gui():
	global app

	app = wx.App(False)
	frame = wx.Frame(None)
	frame.SetWindowStyle( frame.GetWindowStyle()| wx.STAY_ON_TOP )
	panel = ArmControl(frame)
	frame.Show()


def run_gui():
	app.MainLoop()

def setup_recording(filename):
	global record, datafile
	record = True
	datafile = open(filename, "a")
	
if __name__ == "__main__":
	global random

	
	import argparse
	parser=argparse.ArgumentParser()
	parser.add_argument('outputfile', help = 'outputfile', type=str)
	args=parser.parse_args()
	print(args)

	setup_gui()
	setup_apply()


	setup_recording(args.outputfile)

	random = True

	rospy.init_node('listener', anonymous=True)
	threading.Thread(target=run_gui).start()


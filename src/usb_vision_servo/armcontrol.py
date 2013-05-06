#!/usr/bin/env python
# -*- coding: UTF-8 -*-
#import roslib; roslib.load_manifest('usb_vision_servo')
#import rospy
import usb.core
import usb.util
import time

import wx

import threading

	
ba0 = 0
ba1 = 0
ba2 = 0	
app = None

	
	
class SpeedControllor(threading.Thread):
	
	def __init__(self):
		threading.Thread.__init__(self)
		self.steps = 200
		self.sleep = 0.0001

		self.active = True
		self.fix_speed = False

		self.setBaseSpeed(0)
		self.setBaseVertSpeed(0)
		self.setElbowSpeed(0)
		self.setHandSpeed(0)
		self.setArmSpeed(0)

		self.step = 0

		self.start()

		self.speed_multipler = 1


	def setBaseSpeed(self, speed):
		'''-1 to +1'''
		if not self.fix_speed:
			self.baseSpeed = speed
	def setBaseVertSpeed(self, speed):
		'''-1 to +1'''
		if not self.fix_speed:
			self.baseVertSpeed = speed
	def setElbowSpeed(self, speed):
		'''-1 to +1'''
		if not self.fix_speed:
			self.elbowSpeed = speed
	def setArmSpeed(self, speed):
		'''-1 to +1'''
		if not self.fix_speed:
			self.armSpeed = speed
	def setHandSpeed(self, speed):
		'''-1 to +1'''
		if not self.fix_speed:
			self.handSpeed = speed
	
	def _applySpeed(self, targetSpeed, step, fun, dev):
		targetSpeed = targetSpeed*self.speed_multipler
		if targetSpeed > 0 and targetSpeed > float(step)/self.steps:
			fun('cc', dev)
		elif targetSpeed < 0 and -targetSpeed > float(step)/self.steps:
			fun('cl', dev)	
		else:
			fun('', dev)

	def run(self):
		dev = usb.core.find(idVendor=0x1267, idProduct=0)

		while True :
			time_beginning = time.time()

			if(self.step == 0):
				baseSpeed = self.baseSpeed
				baseVertSpeed = self.baseVertSpeed
				elbowSpeed = self.elbowSpeed
				handSpeed = self.handSpeed
				armSpeed = self.armSpeed
				self.baseSpeed = 0
				self.baseVertSpeed = 0
				self.elbowSpeed = 0
				self.handSpeed = 0
				self.armSpeed = 0

			if self.active:
				self._applySpeed(baseSpeed, self.step, moveBase, dev)
				self._applySpeed(baseVertSpeed, self.step, moveBaseV, dev)
				self._applySpeed(elbowSpeed, self.step, moveElbow, dev)
				self._applySpeed(handSpeed, self.step, moveHand, dev)
				self._applySpeed(armSpeed, self.step, moveArm, dev)


			self.step = (self.step + 1) % self.steps

			time_end = time.time()

			time_taken = time_end - time_beginning
			time.sleep(max(0, self.sleep - time_taken))

			

speed_control = SpeedControllor()				
	
	
def moveBase(dir, dev = None):
	if dev == None:
		dev = usb.core.find(idVendor=0x1267, idProduct=0)
	
	global ba1
	
	prev = ba1
	
	#clear bits
	ba1 &=(0xFF ^ 0x01)
	ba1 &=(0xFF ^ 0x02)		
		
	if(dir == 'cl'):   
		ba1 |= 0x01
	elif(dir == 'cc'):
		ba1 |= 0x02
		
	if prev != ba1:
		dev.ctrl_transfer(0x40, 6, 0x100, 0, bytearray([ba0,ba1,ba2]))
	
def moveBaseV(dir, dev = None):
	if dev == None:
		dev = usb.core.find(idVendor=0x1267, idProduct=0)
	
	global ba0
	prev = ba0
	#clear bits
	ba0 &=(0xFF ^ 0x80)
	ba0 &=(0xFF ^ 0x40)		
		
	if(dir == 'cl'):   
		ba0 |= 0x80
	elif(dir == 'cc'):
		ba0 |= 0x40
	if prev != ba0:	
		dev.ctrl_transfer(0x40, 6, 0x100, 0, bytearray([ba0,ba1,ba2]))
	
def moveHand(dir, dev = None):
	if dev == None:
		dev = usb.core.find(idVendor=0x1267, idProduct=0)
	
	global ba0
	prev = ba0
	#clear bits
	ba0 &=(0xFF ^ 0x01)
	ba0 &=(0xFF ^ 0x02)		
		
	if(dir == 'cl'):   
		ba0 |= 0x01
	elif(dir == 'cc'):
		ba0 |= 0x02
	if prev != ba0:		
		dev.ctrl_transfer(0x40, 6, 0x100, 0, bytearray([ba0,ba1,ba2]))
	
def moveElbow(dir, dev = None):
	if dev == None:
		dev = usb.core.find(idVendor=0x1267, idProduct=0)
	
	global ba0
	prev = ba0
	#clear bits
	ba0 &=(0xFF ^ 0x08)
	ba0 &=(0xFF ^ 0x04)		
		
	if(dir == 'cl'):   
		ba0 |= 0x08
	elif(dir == 'cc'):
		ba0 |= 0x04
	if prev != ba0:		
		dev.ctrl_transfer(0x40, 6, 0x100, 0, bytearray([ba0,ba1,ba2]))
def moveArm(dir, dev = None):
	if dev == None:
		dev = usb.core.find(idVendor=0x1267, idProduct=0)
	
	global ba0
	prev = ba0
	#clear bits
	ba0 &=(0xFF ^ 0x10)
	ba0 &=(0xFF ^ 0x20)		
		
	if(dir == 'cl'):   
		ba0 |= 0x10
	elif(dir == 'cc'):
		ba0 |= 0x20
	if prev != ba0:		
		dev.ctrl_transfer(0x40, 6, 0x100, 0, bytearray([ba0,ba1,ba2]))
	
def light(dir, dev = None):
	if dev == None:
		dev = usb.core.find(idVendor=0x1267, idProduct=0)
	
	global ba2
	#clear bits
	ba2 &=(0xFF ^ 0x01)	
		
	if(dir == 'on'):   
		ba2 |= 0x01
		
	dev.ctrl_transfer(0x40, 6, 0x100, 0, bytearray([ba0,ba1,ba2]))
	
	
def move(j1='', j2='', j3='', j4='', hand='', light=''):

	dev = usb.core.find(idVendor=0x1267, idProduct=0)
	
	ba0 = 0
	ba1 = 0
	ba2 = 0
	
	if(j2 == 'cl'): ba0 += 64
	elif(j2 == 'cc'): ba0 += 128

	if(j3 == 'cl'): ba0 += 16
	elif(j3 == 'cc'): ba0 += 32

	if(j4 == 'cl'): ba0 += 4
	elif(j4 == 'cc'): ba0 += 8

	if(hand == 'open'): ba0 += 2
	elif(hand == 'close'): ba0 += 1

	if(j1 == 'cl'): ba1 = 1
	elif(j1 == 'cc'): ba1 = 2

	if(light == 'on'): ba2 = 1

	dev.ctrl_transfer(0x40, 6, 0x100, 0, bytearray([ba0,ba1,ba2]))




class ArmControl(wx.Panel):
	def __init__(self, parent):
		
		self.dev = usb.core.find(idVendor=0x1267, idProduct=0)
		
		wx.Panel.__init__(self, parent)

		# create some sizers
		grid = wx.GridBagSizer(hgap=0, vgap=0)

		# A multiline TextCtrl - This is here to show how the events work in this program, don't pay too much attention to it
		self.logger = wx.TextCtrl(self, size=(400,55), style=wx.TE_MULTILINE | wx.TE_READONLY)
		grid.Add(self.logger, pos=(4,0), span=(1,6))
		
		#STOP button
		self.addBoundButton("STOP", self.stop, grid, (0,0), (2,6), (300,100))
		
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
		
	def goTo(self, event):
		gotoAbs(
			float(self.xIn.GetValue()),
			float(self.yIn.GetValue()),
			float(self.zIn.GetValue()),
			self.logger)
		
	def stop(self, event):
		move()
		speed_control.setBaseSpeed(0)
		speed_control.setBaseVertSpeed(0)
		speed_control.setElbowSpeed(0)
		speed_control.setHandSpeed(0)
			
	def processMoveButton(self, button, cmd, cmd_param):
		#if button.GetValue() :
		#button.SetBackgroundColour(wx.Color(0,200,0))
		cmd(cmd_param)
		#else :
		#	button.SetBackgroundColour(None)
		#	cmd(0)
	
	def baseLeft(self, event):
		#self.processMoveButton(event.EventObject, moveBase, 'cc')
		self.processMoveButton(event.EventObject, speed_control.setBaseSpeed, 0.6)
	
	def baseRight(self, event):
		#self.processMoveButton(event.EventObject, moveBase, 'cl')
		self.processMoveButton(event.EventObject, speed_control.setBaseSpeed,-0.6)
	
	def baseUp(self, event):
		self.processMoveButton(event.EventObject, speed_control.setBaseVertSpeed, 0.6)
	
	def baseDown(self, event):
		self.processMoveButton(event.EventObject, speed_control.setBaseVertSpeed, -0.6)
	
	def elbowUp(self, event):
		self.processMoveButton(event.EventObject, speed_control.setElbowSpeed, 0.6)
	
	def elbowDown(self, event):
		self.processMoveButton(event.EventObject, speed_control.setElbowSpeed, -0.6)
	
	def armUp(self, event):
		self.processMoveButton(event.EventObject, speed_control.setArmSpeed, 0.6)
	
	def armDown(self, event):
		self.processMoveButton(event.EventObject, speed_control.setArmSpeed, -0.6)
	
	def handOpen(self, event):
		self.processMoveButton(event.EventObject, speed_control.setHandSpeed, 0.6)
	
	def handClose(self, event):
		self.processMoveButton(event.EventObject, speed_control.setHandSpeed, -0.6)
	
	def light(self, event):
		self.processMoveButton(event.EventObject, light, 'on')
						
	def poll(self, event):
		self.Refresh()

def setup_gui():
	global app
	
	app = wx.App(False)
	frame = wx.Frame(None)
	frame.SetWindowStyle( frame.GetWindowStyle()| wx.STAY_ON_TOP ) 
	panel = ArmControl(frame)
	frame.Show()


def run_gui():
	app.MainLoop()

	
if __name__ == "__main__":
	print "starting armcontrol"
	setup_gui()
	run_gui()

	speed_control.speed_multipler = 100
	 


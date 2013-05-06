#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import roslib; roslib.load_manifest('usb_vision_servo')
import rospy
import wx
import threading
import time

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from hwu_plotter.srv import GalilCmd

"""
This builds a gui that can communicate galil commands to the plotter
x and y are mixed up (thats how the hardware is setup) so thats detangled here

"""
def sendCmds(cmds, gfxlog=None, log=True):
	for cmd in cmds.split(";"):
		sendCmd(cmd, gfxlog, log)

def sendCmd(cmd, gfxlog=None, log=True):
	if log: rospy.loginfo("sending %s" % cmd)
		
	try:
		cmd_srv = rospy.ServiceProxy('galil_cmd', GalilCmd)
		rsp = cmd_srv(cmd)
		if log: rospy.loginfo("recieve %s" % rsp)
		
		if gfxlog:
			if log: gfxlog.AppendText('sent: %s\n' % cmd)
			if log: gfxlog.AppendText('resp: %s\n' % rsp)
		return rsp.res
	except Exception as e:
		if log: rospy.loginfo("%s" % e)
		if gfxlog:
			if log: gfxlog.AppendText("%s\n" % e)
		
def gotoAbs(x,y,z, gfxLog=None, log=True):
	scale = rospy.get_param('/scale', {'x':1,'y':1,'z':1})
		
	x = float(x)* scale['x'];
	y = float(y)* scale['y'];
	z = float(z)* scale['z'];
	
	sendCmds("PA %0.2f,%0.2f,%0.2f;BGA;BGB;BGC" % (x,y,z), gfxLog, log);
	
def getAbs():
	pos = sendCmd("PA ?, ?, ?", log=False).split(",")
	scale = rospy.get_param('/scale', {'x':1,'y':1,'z':1})
	x = float(pos[0]) / scale['x'];
	y = float(pos[1]) / scale['y'];
	z = float(pos[2]) / scale['z'];
	return (x,y,z)
	
class PlotterControl(wx.Panel):
	def __init__(self, parent):
		wx.Panel.__init__(self, parent)

		# create some sizers
		grid = wx.GridBagSizer(hgap=0, vgap=0)

		# A multiline TextCtrl - This is here to show how the events work in this program, don't pay too much attention to it
		self.logger = wx.TextCtrl(self, size=(400,55), style=wx.TE_MULTILINE | wx.TE_READONLY)
		grid.Add(self.logger, pos=(4,0), span=(1,6))
		
		# stop button
		self.stopButton =wx.Button(self, label="STOP", size=(150,100))
		self.Bind(wx.EVT_BUTTON, self.stop,self.stopButton)
		grid.Add(self.stopButton, pos=(0,0), span=(2,3))

		# zero button
		self.zeroButton =wx.Button(self, label="zero", size=(50,50))
		self.Bind(wx.EVT_BUTTON, self.zero, self.zeroButton)
		grid.Add(self.zeroButton, pos=(1,5), span=(1,1))
		
		#up down left right zUp zDown
		self.upButton =wx.ToggleButton(self, label="↑", size=(50,50))
		self.Bind(wx.EVT_TOGGLEBUTTON, self.moveUp,self.upButton)
		grid.Add(self.upButton, pos=(2,1))
		
		self.downButton =wx.ToggleButton(self, label="↓", size=(50,50))
		self.Bind(wx.EVT_TOGGLEBUTTON, self.moveDown,self.downButton)
		grid.Add(self.downButton, pos=(3,1))
		
		self.leftButton =wx.ToggleButton(self, label="←", size=(50,50))
		self.Bind(wx.EVT_TOGGLEBUTTON, self.moveLeft,self.leftButton)
		grid.Add(self.leftButton, pos=(3,0), span=(1,1))
		
		self.rightButton =wx.ToggleButton(self, label="→", size=(50,50))
		self.Bind(wx.EVT_TOGGLEBUTTON, self.moveRight,self.rightButton)
		grid.Add(self.rightButton, pos=(3,2), span=(1,1))
		
		self.zDownButton =wx.ToggleButton(self, label="z↓", size=(50,50))
		self.Bind(wx.EVT_TOGGLEBUTTON, self.moveZDown,self.zDownButton)
		grid.Add(self.zDownButton, pos=(2,0), span=(1,1))
		
		self.zUpButton =wx.ToggleButton(self, label="z↑", size=(50,50))
		self.Bind(wx.EVT_TOGGLEBUTTON, self.moveZUp,self.zUpButton)
		grid.Add(self.zUpButton, pos=(2,2), span=(1,1))
		
		#current position
		self.status =wx.TextCtrl(self, size=(300,-1), style=wx.TE_READONLY)
		grid.Add(self.status, pos=(0,3), span=(1,3))
		
		#go to functionality in its own panel
		goToPanel = wx.Panel(self)
		
		w=40
		self.GO =wx.Button(goToPanel, label="GO", size=(w,w))
		self.Bind(wx.EVT_BUTTON, self.goTo,self.GO)
		
		self.xIn = wx.TextCtrl(goToPanel, value="0", pos=( 0+w,5), size=(50,-1), style=wx.TE_PROCESS_ENTER )
		self.yIn = wx.TextCtrl(goToPanel, value="0", pos=(50+w,5), size=(50,-1))
		self.zIn = wx.TextCtrl(goToPanel, value="0", pos=(100+w,5), size=(50,-1))
		
		grid.Add(goToPanel, pos=(2,3), span=(1,3))
		
		self.Bind(wx.EVT_TEXT_ENTER, self.goTo,self.xIn)
		self.Bind(wx.EVT_TEXT_ENTER, self.goTo,self.yIn)
		self.Bind(wx.EVT_TEXT_ENTER, self.goTo,self.zIn)
		
		
		#limit switches
		"""
		limitPanel = wx.Panel(self)
		limitGrid = wx.GridBagSizer(hgap=2, vgap=2)

		self.lfx = wx.StaticText(limitPanel, label="fx")
		self.lrx = wx.StaticText(limitPanel, label="rx")
		self.lfy = wx.StaticText(limitPanel, label="fy")
		self.lry = wx.StaticText(limitPanel, label="ry")
		self.lfz = wx.StaticText(limitPanel, label="fz")
		self.lrz = wx.StaticText(limitPanel, label="rz")
		
		limitGrid.Add(self.lfx, pos=(0,0))
		limitGrid.Add(self.lrx, pos=(1,0))
		limitGrid.Add(self.lfy, pos=(0,1))
		limitGrid.Add(self.lry, pos=(1,1))
		limitGrid.Add(self.lfz, pos=(0,2))
		limitGrid.Add(self.lrz, pos=(1,2))
		limitPanel.SetSizerAndFit(limitGrid)
		
		grid.Add(limitPanel, pos=(3,3), span=(1,2))
		"""
		
		self.SetSizerAndFit(grid)

		#initialize the speed low
		sendCmd("SP 1000, 1000, 10000", self.logger)
		
		self.timer = wx.Timer(self)
		self.Bind(wx.EVT_TIMER, self.poll, self.timer)
		self.timer.Start(1000)  # x100 milliseconds
		
	def goTo(self, event):
		gotoAbs(
			float(self.xIn.GetValue()),
			float(self.yIn.GetValue()),
			float(self.zIn.GetValue()),
			self.logger)
		
	def stop(self, event):
		sendCmd("ST", self.logger)
	
	def zero(self, event):
		sendCmd("DP 0, 0, 0", self.logger)
		
	def processMoveButton(self, button, goCommand, stopCommand):
		if button.GetValue() :
			button.SetBackgroundColour(wx.Color(0,200,0))
			sendCmds(goCommand, self.logger)
		else :
			button.SetBackgroundColour(None)
			sendCmds(stopCommand, self.logger)
	
	def moveLeft(self, event):
		self.processMoveButton(self.leftButton, "PR , -10000, ;BGB", "STB")
	
	def moveRight(self, event):
		self.processMoveButton(self.rightButton, "PR , 10000, ;BGB", "STB")
	
	def moveUp(self, event):
		self.processMoveButton(self.upButton, "PR  10000, , ;BGA", "STA")

	def moveDown(self, event):
		self.processMoveButton(self.downButton, "PR -10000, , ;BGA", "STA")

	def moveZDown(self, event):
		self.processMoveButton(self.zDownButton, "PR , , 100000 ;BGC", "STC")

	def moveZUp(self, event):
		self.processMoveButton(self.zUpButton, "PR , , -100000 ;BGC", "STC")
		
	def EvtRadioBox(self, event):
		self.logger.AppendText('EvtRadioBox: %d\n' % event.GetInt())
	def EvtComboBox(self, event):
		self.logger.AppendText('EvtComboBox: %s\n' % event.GetString())
	def OnClick(self,event):
		self.logger.AppendText(" Click on object with Id %d\n" %event.GetId())
	def EvtText(self, event):
		self.logger.AppendText('EvtText: %s\n' % event.GetString())
	def EvtChar(self, event):
		self.logger.AppendText('EvtChar: %d\n' % event.GetKeyCode())
		event.Skip()
	def EvtCheckBox(self, event):
		self.logger.AppendText('EvtCheckBox: %d\n' % event.Checked())
			
	def poll(self, event):
		prev = getAbs()
		
		self.status.SetValue("(%5.2f %5.2f %5.2f)"%prev)
		self.Refresh()
		
		
		
def main():
	app = wx.App(False)
	frame = wx.Frame(None)
	frame.SetWindowStyle( frame.GetWindowStyle()| wx.STAY_ON_TOP ) 
	panel = PlotterControl(frame)
	frame.Show()
	
	#poller = Poller(panel)
	#poller.setDaemon(True)
	#poller.start()
	
	app.MainLoop()
	
	
if __name__ == "__main__":
	main()

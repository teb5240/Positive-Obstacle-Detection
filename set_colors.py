#!/usr/bin/env python

import rospy
import sys
import numpy as np
import serial
from geometry_msgs.msg import PointStamped

class UltrasonicLEDSerialNode():
	def __init__(self,ultrasonic_port,LED_port,baud_rate,number_of_ultrasonics,Dgreen,Dred):
		rospy.init_node('serial', anonymous=True)
		#connect to ultrasonic port at specified baud rate
		self.ultrasonic_ser = self.connect(ultrasonic_port,baud_rate)
		#connect to LED port at specified baud rate
		self.LED_ser = self.connect(LED_port,baud_rate)
		#setup list of PointStamped Messages to hold the distance readings
		self.UltrasonicMessages = self.setupUltrasonicMessages(number_of_ultrasonics)
		#setup ROS publishers to publish distance readings to ROS topics
		self.publishers = self.setupPublishers(number_of_ultrasonics)
		#distance threshold beyond which LED's should turn green
		self.Dgreen = Dgreen
		#distance threshold within which LED's dhould turn red
		self.Dred = Dred
		#setup list to hold Red and Green binary values for all distance readings
		self.RGValues = self.setupLEDMessages(number_of_ultrasonics)

	#create serial object
	def connect(self,port,baud_rate):
		ser = serial.Serial(port = port, baudrate = baud_rate)
		return ser
	#read one line of data from serial port

	def read(self):
		return self.ultrasonic_ser.readline()
	#write one line of data to serial port
	def write(self,data):
		for k in range(0,len(data)):
			self.LED_ser.write(bytes(data[k]))
			self.LED_ser.write(',')
		self.LED_ser.write('\n')
	#flush serial buffers.  May not be needed
	def flush(self):
		self.ultrasonic_ser.reset_input_buffer()
		self.LED_ser.reset_output_buffer()
	
	def CalculateRGValues(self,distance):
		#list to hold Red and Green values for one distance reading
		RGValues = []
		#when distance is greater than green threshold, turn LED's green
		if self.Dgreen < distance:
			#G = 255, R = 0
			RGValues.append(0b00000000)
			RGValues.append(0b11111111)
		#when distance is between green and red thresholds, turn LED's varying shade of yellow
		if self.Dgreen > distance and self.Dred < distance:
			RGValues.append(int(255+(255*self.Dred/(self.Dgreen-self.Dred))-(255/(self.Dgreen-self.Dred)*distance)))
			RGValues.append(int((255/(self.Dgreen-self.Dred)*distance)-(255*self.Dred/(self.Dgreen-self.Dred))))
		#when distance is less than red threshold, turn LED's red
		if self.Dred > distance:
			#G = 0, R = 255
			RGValues.append(0b11111111)
			RGValues.append(0b00000000)
			
		return RGValues
	#setup list of PointStamped messages for ultrasonic readings and assign a frame id to each (for tf)
	def setupUltrasonicMessages(self, number_of_ultrasonics):
		ultrasonics = []
		for i in range(1,number_of_ultrasonics+1):
			message = PointStamped()
			message.header.frame_id = 'distance' + str(i)
			ultrasonics.append(message)
		return ultrasonics
	
	#setup list of Red and Green binary values for all ultrasonic readings
	def setupLEDMessages(self, number_of_ultrasonics):
		RGValues = []
		for k in range(0,2*number_of_ultrasonics):
			RGValues.append(0)
		return RGValues
	
	#setup list of publishers to publish each PointStamped message of an ultrasonic reading to its own ROS topic
	def setupPublishers(self, number_of_ultrasonics):
		ultrasonics = []
		for i in range(1,number_of_ultrasonics+1):
			publisher = rospy.Publisher('/distance' + str(i), PointStamped, queue_size=1)
			ultrasonics.append(publisher)
		return ultrasonics

def main(args):
	#define the serial port for the ultrasonics. This will change when uploaded to wheelchair
	ultrasonic_port = '/dev/ttyACM1'
	#define the serial port for the NeoPixel.  This will change when uploaded to wheelchair
	#LED_port = '/dev/ttyACM0'
	LED_port = 'COM8'
	#define the baud rate
	baud_rate = 115200
	#define the number of ultrasonics
	number_of_ultrasonics = 6
	#Green threshold
	Dgreen = 80
	#Red threshold
	Dred = 20

	#create an UltrasonicLEDSerialNode object, initialize to previously defined values
	un = UltrasonicLEDSerialNode(ultrasonic_port, LED_port, baud_rate, number_of_ultrasonics, Dgreen, Dred)
	
	while not rospy.is_shutdown():
		
		try:
			#read one line from serial port(one reading per ultrasonic at that time)
			data = un.read()
		
			try:
				#store the line of data from the serial port as a list of strings, one string for each ultrasonic reading(each reading is comma separated)
				distances = data.split(',')
				#converts data type of distance array from strings to unsigned integers
				distances = np.uint(distances)
				#convert durations in microseconds to distances in cm
				distances = [k/58 for k in distances]
				#rospy.loginfo(distances)

				#check to make sure the number of readings is the same as the number of ultrasonics previously defined
				if len(distances) == number_of_ultrasonics:
					#for each reading
					for i in range(0,len(distances)):
						#set the time stamp for the point stamped message that the reading will be stored in
						un.UltrasonicMessages[i].header.stamp = rospy.Time.now()
						#store the reading into the x parameter of its point stamped message
						un.UltrasonicMessages[i].point.x = distances[i]
						#distance = distances[i]
						#calculate Red and Green Values
						un.RGValues[2*i] = un.CalculateRGValues(distances[i])[0]
						un.RGValues[2*i+1] = un.CalculateRGValues(distances[i])[1]
						#publish each point stamped message to its own ROS topic
						un.publishers[i].publish(un.UltrasonicMessages[i])
					#rospy.loginfo(un.RGValues)
					#send Red and Green Values to Arduino
					un.write(un.RGValues)
					un.flush()
					
			#display any warnings that appear when formatting the data from the serial port
			except Exception as e:

				rospy.logwarn(e)
		#catch KeyboardInterrupt exception when node is killed, print "Shutting down!" to console
		except KeyboardInterrupt:

			print("Shutting down!")

#do everything in main
if __name__ == '__main__':
	main(sys.argv)
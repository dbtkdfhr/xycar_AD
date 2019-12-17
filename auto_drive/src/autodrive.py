#!/usr/bin/env python

import rospy, time
import cv2
from linedetector import LineDetector
from motordriver import MotorDriver
from std_msgs.msg import Int32MultiArray

usonic_data = None

class MovingAverage:
	def __init__(self,n):
		self.samples = n
	#	self.data = [70,100,100,100,70,70,70,70,70,]
		self.data = [50,50,50]
		self.weights = list(range(1,n+1))

	def add_sample(self, new_sample):

		if len(self.data) < self.samples:
			self.data.append(new_sample)
		else:
			self.data = self.data[1:] + [new_sample]
	def get_mm(self):
		return float(sum(self.data)) / len(self.data)

	def get_wmm(self):
		s = 0
		for i ,x in enumerate(self.data):
			s+= x* self.weights[i]
		return float(s) / sum(self.weights[:len(self.data)])


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
        my_angle = self.line_detector.angle
        #self.line_detector.show_images(line_l, line_r)
        car_angle = self.steer(my_angle)
	if car_angle == 60:
		self.driver.drive(car_angle+120,self.line_detector.speed-5)
		time.sleep(0.1)
	elif car_angle == -60:
		self.driver.drive(car_angle+120,self.line_detector.speed-5)
		time.sleep(0.05)
	else:
		self.driver.drive(95,self.line_detector.speed)
    def turn_right(self):
	for i in range(2):
		self.driver.drive(90,90)
		time.sleep(0.1)
		self.driver.drive(90,60)
		time.sleep(0.1)
	for back_cnt in range(10):
		self.driver.drive(90,60)
		time.sleep(0.1)
	self.driver.drive(170,self.line_detector.speed-5)
	time.sleep(1)
	self.driver.drive(40,self.line_detector.speed-5)
	time.sleep(4)
	self.driver.drive(110,self.line_detector.speed-5)
	time.sleep(1)
		
    def stop(self):
	self.driver.drive(90,90)

    def steer(self, angle):
	#print(angle)
	if(angle < 0):
		return 60
	elif(angle > 0):
		return -60
	#angle = angle*(-70)
        return angle

    def accelerate(self, angle, left, mid, right):
        return angle

    def exit(self):
        print('finished')


def callback(data):
	global usonic_data
	usonic_data = data.data

stop = False

if __name__ == '__main__':
    rospy.Subscriber('ultrasonic', Int32MultiArray,callback)

    global usonic_data

    front_mm = MovingAverage(3)

    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    start_time = time.time()
    while not rospy.is_shutdown():
	if time.time() - start_time > 50:
		car.stop()
		time.sleep(5)
		start_time = time.time()
        car.trace()
	#if usonic_data[1] < 85:
	if usonic_data[1] < 50:
		front_mm.add_sample(usonic_data[1])
	if front_mm.get_wmm() < 40:
		car.turn_right()
		front_mm = MovingAverage(3)
				
#	if cv2.waitKey(1) & 0xFF == ord('q'):
#		break
#	cv2.imshow('v',car.line_detector.cam_img)
        rate.sleep()
    rospy.on_shutdown(car.exit)

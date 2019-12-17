import cv2
import numpy as np
import time
from pyzbar import pyzbar

try:
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except:
    pass


class LineDetector:
    def __init__(self, topic, ros_node=True):
	self.angle = 0
	self.speed = 120
        self.ros_node = ros_node
        self.image_width = 640
        self.scan_width, self.scan_height = 400, 40
        self.area_width, self.area_height = 10, 10
        area = self.area_width * self.area_height
        self.pxl_cnt_threshold = area * 0.1
        self.linescan_offset = 15
        self.roi_vertical_pos = 280
        self.left, self.right = -1, -1

        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        if self.ros_node:
            self.bridge = CvBridge()
            rospy.Subscriber(topic, Image, self.conv_image)
            self.recorder = cv2.VideoWriter(
                '/home/nvidia/xycar/src/auto_drive/record.avi',
                cv2.VideoWriter_fourcc(*'MJPG'),
                30,
                (640, 480)
            )

    def __del__(self):
        if self.ros_node:
            self.recorder.release()
        cv2.destroyAllWindows()

    def region_of_interest(self, img, vertices):
        mask = np.zeros_like(img)
        match_mask_color = 255
        cv2.fillPoly(mask, vertices, match_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def conv_image(self, data):
	self.cam_img = self.bridge.imgmsg_to_cv2(data,'bgr8')
	barcodes = pyzbar.decode(self.cam_img)
	for barcode in barcodes:
		barcodeData = barcode.data.decode("utf-8")
		self.speed = int(barcodeData)
		print(barcodeData)
#	hsv = cv2.cvtColor(self.cam_img,cv2.COLOR_BGR2HSV)
#	hsv[:,:,2] -= 75
#	self.cam_img = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
	#cv2.imshow("dst",self.cam_img)
	#cv2.waitkey(0)
	#cv2.destroyAllWindows()
	
	#lab = cv2.cvtColor(self.cam_img,cv2.COLOR_BGR2LAB)
	#l,a,b = cv2.split(lab)
	
	#clahe = cv2.createCLAHE(clipLimit=3.0,tileGridSize=(8,8))
	#cl = clahe.apply(l)
	#limg = cv2.merge((cl,a,b))
	#self.cam_img = cv2.cvtColor(limg,cv2.COLOR_LAB2BGR)

	height = self.cam_img.shape[0]
        width = self.cam_img.shape[1]
        region_of_interest_vertices = [
            (0, height),
	    (width/2 , height/2),
            (width, height)
        ]
        gray_image = cv2.cvtColor(self.cam_img, cv2.COLOR_RGB2GRAY)
	#ret, self.cam_img = cv2.threshold(gray_image,100,255,cv2.THRESH_BINARY)
        #150->50
	canny_image = cv2.Canny(gray_image,200, 225)
        cropped_image = self.region_of_interest(canny_image,
                                                np.array([region_of_interest_vertices], np.int32), )
        lines = cv2.HoughLinesP(cropped_image,
                                rho=2,
                                theta=np.pi / 180,
                                threshold=50,
                                lines=np.array([]),
                                minLineLength=0,
                                maxLineGap=105)
        image_with_lines = self.detect_lines(self.cam_img, lines)
        return image_with_lines

    def cal_gradient(self,x1, y1, x2, y2):
	if float(x2 - x1) == 0:
	    return 0
        return float(y2 - y1) / float(x2 - x1)

    def detect_lines(self, img, lines):
        img = np.copy(img)
        blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        
	'''
	start_lines = lines[0]
	x1,y1,x2,y2 = start_lines[0][0],start_lines[0][1],start_lines[0][2],start_lines[0][3]
	if(y2 > 300):
		return self.cam_img
	gradient = self.cal_gradient(x1,y1,x2,y2)
	if(gradient >0 and gradient < 15):
		self.angle = gradient
		return self.cam_img
	elif gradient < 0 and gradient > -15:
		self.angle = gradient
		return self.cam_img
	'''
	if lines == None:
	    return self.cam_img

	for line in lines:
            for x1, y1, x2, y2 in line:
                #if ((y1 > 300 or y1 < 200)):
                #    break
		if(y1> 300):
		     break
                gradient = self.cal_gradient(x1, y1, x2, y2)
                if (gradient < 0):
                   # if (gradient < -1.5):
                    #    continue
		    self.angle = gradient
		    #cv2.line(blank_image, (x1,y1),(x2,y2),(0,255,0),thickness=10)
		    #self.cam_img = cv2.addWeighted(img,0.8,blank_image,1,0.0)
                    return self.cam_img
                else:
                    if (gradient > 1.5):
                        continue
		    self.angle = gradient
		    #cv2.line(blank_image, (x1,y1),(x2,y2),(0,255,0),thickness=10)
		    #self.cam_img = cv2.addWeighted(img,0.8,blank_image,1,0.0)
                    return self.cam_img
		break
	
	self.angle = 0
	#self.cam_img = cv2.addWeighted(img,0.8,blank_image,1,0.0)
        return self.cam_img

    def showimage(self):
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
	if cv2.waitKey(1) & 0xFF == ord('q'):
		return 0
	cv2.imshow("v",self.cam_img)
	cv2.destroyAllWindows()

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import roslib
import cv2
import numpy as np
from sensor_msgs.msg import Image,CompressedImage
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
# from facenet_pytorch import MTCNN

class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        # self.mtcnn = MTCNN()

        self.robot_name = rospy.get_param('~robot_name', 'qingzhou')
        if(self.robot_name=="/qingzhou_1" or self.robot_name=="/qingzhou_5"):
            self.image_sub = rospy.Subscriber(self.robot_name+"/camera_link/image_raw", Image, self.callback)
            self.image_pub = rospy.Publisher(self.robot_name+"/color_detection/annotated_image", Image, queue_size=10)
            self.loc_people = rospy.Publisher("loc_people", Point, queue_size=10)

        
        # self.image_sub = rospy.Subscriber("/qingzhou_1/camera_link/image_raw", Image, self.callback)
        # self.image_pub = rospy.Publisher("/color_detection/annotated_image", Image, queue_size=10)
        self.camera_matrix = np.array([[1499.641, 0, 1097.616],
                                                             [0., 1497.989, 772.371],
                                                              [0., 0., 1.]])
        self.dist_coeffs = np.array([[-0.1103, 0.0789, -0.0004, 0.0017, -0.0095]])

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        font = cv2.FONT_HERSHEY_SIMPLEX
        # boxes, _ = self.mtcnn.detect(cv_image)
        # if boxes is not None:
        #     for box in boxes:
        #         x1,y1,x2,y2 = box.astype(int)
        #         cv2.rectangle(cv_image,(x1,y1),(x2,y2),(255,0,0),2)
        #         cv2.putText(cv_image,"face",(x1,y1-5),font,0.7,(255,0,0),2)
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        template = cv2.imread("/home/djm/car/src/qingzhou_nav/image/temp2.png", cv2.IMREAD_GRAYSCALE)
        # cv2.imshow("template", template)
        w, h = template.shape[::-1]
 
        result = cv2.matchTemplate(gray_img, template, cv2.TM_CCOEFF_NORMED)
        # print(result)
        minval, maxval, minloc, maxloc = cv2.minMaxLoc(result)
        # print(minval, maxval, minloc, maxloc)
 
        loc = np.where(result >= 0.8)
        # print(loc)
        # for pt in zip(*loc[::-1]):
        #     cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 3)
        if (np.sum(loc)!=0):
            # print(1111111111111111111111111111)
            loc_ = Point()
            loc_.x=19.661200
            loc_.y=30.982501
            self.loc_people.publish(loc_)

        for pt in zip(*loc[::-1]):
            pt = maxloc
            cv2.rectangle(cv_image, pt, (pt[0] + w, pt[1] + h), (255, 255, 0), 2)
            cv2.putText(cv_image,"people",(pt[0],pt[1]-10),font,0.7,(255,255,0),2)

        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 红色的HSV值范围
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        # 绿色的HSV值范围
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([70, 255, 255])

        # 青色的HSV值范围
        lower_blue = np.array([78, 43, 46])
        upper_blue = np.array([99, 255, 255])

        # 对图像进行掩模处理，提取红色和绿色区域
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # 对红色和绿色区域进行膨胀操作，填充空洞
        kernel = np.ones((5,5),np.uint8)
        mask_red = cv2.dilate(mask_red, kernel, iterations=1)
        mask_green = cv2.dilate(mask_green, kernel, iterations=1)
        mask_blue = cv2.dilate(mask_blue, kernel, iterations=1)

        # 检测红色和绿色物体的轮廓
        _, contours_red, _  = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        
        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > 3000: 
                x,y,w,h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),2)
        
        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area > 2500:
                x,y,w,h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(cv_image,"tree",(x,y-5),font,0.7,(0,255,0),2)
        
        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if area > 3000: 
                x,y,w,h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)



        # 计算最大检测框到相机的距离
        max_width = 0
        max_height = 0
        for contour in contours_red + contours_green + contours_blue:
            x, y, w, h = cv2.boundingRect(contour)
            if w > max_width:
                max_width = w
            if h > max_height:
                max_height = h
        max_size = max(max_width, max_height)
        object_points = np.array([[0, 0, 0],
                                  [max_size, 0, 0],
                                  [0, max_size, 0],
                                  [0, 0, -max_size]], dtype=np.float32)
        image_points = np.array([[1097.616, 772.371],
                                 [1097.616 + max_size, 772.371],
                                 [1097.616, 772.371 + max_size],
                                 [1097.616, 772.371]], dtype=np.float32)
        _, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)

        # 计算物体到相机的距离
        distance = np.linalg.norm(tvec)
        
        #压缩图像
        # _,compressed_img,_ =cv2.imencode("jpeg",cv_image,[int(cv2.IMWRITE_JPEG_QUALITY),50])
        # annotated_img_msg = CompressedImage()
        # annotated_img_msg.header.stamp = rospy.Time.now()
        # annotated_img_msg.format = "jpeg"
        # annotated_img_msg.data =np.array(cv2.imencode(".jpg",cv_image)[1]).tostring()
        # 发布带有边界框的图像到话题
        annotated_img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.image_pub.publish(annotated_img_msg)

       

        # 显示处理后的图像
        cv2.imshow("Detection", cv_image)
        print("Distance to object:", distance)
        cv2.waitKey(1)

def main():
    rospy.init_node('color_detector', anonymous=True)
    detector = ColorDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

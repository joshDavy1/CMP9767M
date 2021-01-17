#!/usr/bin/env python

## Imports 

import rospy
import cv2
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
from geometry_msgs.msg import Pose,PoseArray
import numpy as np

LOW_H  = 0.207
HIGH_H = 0.643
LOW_S  = 0.000
HIGH_S = 0.460
LOW_V  = 0.069
HIGH_V = 0.307

class filterAndBlob:

    ## Callback for image
    def image_callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)


    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once


    def __init__(self,low_h,low_s,low_v,high_h,high_s,high_v):
        # Init
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",Image,self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_camera/hd/camera_info', CameraInfo, self.camera_info_callback)
        self.pub = rospy.Publisher('weeds', PoseArray, queue_size=10)
        
        self.cv_image = None

        self.low_H = int(low_h*255)
        self.low_S = int(low_s*255)
        self.low_V = int(low_v*255)

        self.high_H = int(high_h*255)
        self.high_S = int(high_s*255)
        self.high_V = int(high_v*255)

        # BLOB DETECTOR
        params = cv2.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 10000
        params.maxArea = 500000

        # Filter by Distance
        params.minDistBetweenBlobs = 0.01

        # Filters
        params.filterByConvexity = False
        params.filterByCircularity = False
        params.filterByInertia = False

        self.blobDetector = cv2.SimpleBlobDetector_create(params)
        
    def hsvFilter(self,imageHSV) :
        """ Filters image by HSV values"""
        return cv2.inRange(imageHSV, (self.low_H, self.low_S, self.low_V), (self.high_H, self.high_S, self.high_V))


    def blobDetect(self,im) :
        """ Detects blobs in a image """
        keypoints = self.blobDetector.detect(im)
        im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return keypoints,im_with_keypoints

    def pointsToPoseArray(self,points,distance) :
        """  Takes an array of points, projects into camera frame and returns array of poses. """
        pose_array = PoseArray()
        pose_array.header.frame_id = self.camera_model.tfFrame()
        for pnt in points :
            pose = Pose()
            ray = self.camera_model.projectPixelTo3dRay(pnt)
            # https://python.hotexamples.com/examples/image_geometry/PinholeCameraModel/projectPixelTo3dRay/python-pinholecameramodel-projectpixelto3dray-method-examples.html
            ray_z = [el / ray[2] for el in ray]  # normalize the ray so its Z-component equals 1.0
            pt = [el * distance for el in ray_z]  # multiply the ray by the depth; its Z-component should now equal the depth value
            pose.position.x = pt[0]
            pose.position.y = pt[1]
            pose.position.z = pt[2]
            pose_array.poses.append(pose)
        return pose_array

    def run(self) :
        """ Run main loop """
        rospy.init_node('filterAndBlob', anonymous=True)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            # Convert image to hsv
            hsvImage = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            # Filter Image
            filteredImage = self.hsvFilter(hsvImage) 
            filteredImage = cv2.dilate(filteredImage, None, iterations=25)
            filteredImage = cv2.medianBlur(filteredImage,31)
            # Blob detection
            filteredImage = cv2.bitwise_not(filteredImage)
            keypoints,image = self.blobDetect(filteredImage)
            # Create pose_array object
            points = []
            for i in keypoints :
                points.append((i.pt[0],i.pt[1]))
            pose_array = self.pointsToPoseArray(points,0.45)
            # Publish
            self.pub.publish(pose_array)
            rate.sleep()



           
if __name__ == '__main__':
    try:
        fil = filterAndBlob(low_h = LOW_H,low_s = LOW_S,low_v = LOW_V,
                    high_h = HIGH_H, high_s = HIGH_S, high_v= HIGH_V) 
        fil.run()
    except rospy.ROSInterruptException:
        pass
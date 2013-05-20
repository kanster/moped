#!/usr/bin/env python
################################################################################
#                                                                              
# imagesenderlist.py 
#
# Copyright: Carnegie Mellon University and Intel Corporation
# Author: Alvaro Collet (acollet@cs.cmu.edu)
#
################################################################################
import roslib; roslib.load_manifest('imagesender')
import rospy
import cv_bridge
import sys
import os
import cv
from sensor_msgs.msg import Image 
from sensor_msgs.msg import CameraInfo

DEBUG = True

# ------------------------------------------------------------------ #
class image_publisher:

    def __init__(self):
        rospy.loginfo("Starting __init__")

        self.image_pub = rospy.Publisher("Image", Image)
        self.cam_pub = rospy.Publisher("CameraInfo", CameraInfo)
        self.bridge = cv_bridge.CvBridge()
        self.cam_data = CameraInfo()
        self.img_data = ''
        self.read_camdata()
        self.last_img = ''
        if DEBUG:
            rospy.loginfo( "Done with __init__")

    def read_camdata(self):
        """read_camdata - Read camera parameters from file
        """

        self.cam_data.height = 480
        self.cam_data.width = 640
        KK_fx = rospy.get_param('KK_fx', 1100)
        KK_fy = rospy.get_param('KK_fy', 1100)       
        KK_cx = rospy.get_param('KK_cx', self.cam_data.width/2)      
        KK_cy = rospy.get_param('KK_cy', self.cam_data.height/2)       
        D = [0.0, 0.0, 0.0, 0.0, 0.0]
        D[0] = rospy.get_param('kc_k1', 0)
        D[1] = rospy.get_param('kc_k2', 0)       
        D[2] = rospy.get_param('kc_p1', 0)      
        D[3] = rospy.get_param('kc_p2', 0)         
        self.cam_data.D = D

        self.cam_data.K[0] = KK_fx
        self.cam_data.K[4] = KK_fy
        self.cam_data.K[2] = KK_cx
        self.cam_data.K[5] = KK_cy
        self.cam_data.K[8] = 1

        self.cam_data.R[0] = 1; self.cam_data.R[1] = 0; self.cam_data.R[2] = 0;
        self.cam_data.R[3] = 0; self.cam_data.R[4] = 1; self.cam_data.R[5] = 0;
        self.cam_data.R[6] = 0; self.cam_data.R[7] = 0; self.cam_data.R[8] = 1;
        
        self.img_data = rospy.get_param('imgmsg_type', 'bgr8')
        self.refresh_rate = rospy.get_param('refresh_rate', 1)

        if DEBUG:
            rospy.loginfo( "Done with read_camdata")

    def read_img(self, filename):
        """read_img - Read an image and encapsulate it in ROS format
        """
    
        try:
            cv_img = cv.LoadImage(filename)
            bw_img = cv.CreateImage((cv_img.width, cv_img.height), cv.IPL_DEPTH_8U, 1) 
            cv.CvtColor(cv_img, bw_img, cv.CV_RGB2GRAY)
            self.cam_data.height = bw_img.height
            self.cam_data.width = bw_img.width
            self.img = self.bridge.cv_to_imgmsg(bw_img)
        except cv_bridge.CvBridgeError, e:
            print e
        
        if DEBUG:
            rospy.loginfo( "Done reading image %s", filename)

        self.last_img = filename
        return self.img


    def publish(self):
        self.image_pub.publish(self.img)
        self.cam_pub.publish(self.cam_data)

        if DEBUG:
            rospy.loginfo( "Done publishing image %s", self.last_img)
        

    def loopfiles(self, pathname='./', filename='checkfile.txt'):
        """ getfiles - Constantly search for specific folder, load images and publish

            This function looks for a file 'filename' within 'pathname' that
            contains an image name to send. This file is constantly monitored,
            and when it is modified (and contains a new image name), the image
            is read and sent through the network.
        """

        fname = os.path.join(pathname, filename)
   
        rospy.loginfo( "Debug: %d, file: %s", DEBUG, fname)

        if os.path.exists(fname):
            f = open(fname, 'rt')
              
            # Loop over all filenames in file
            for line in f:
            
                new_img = line.rstrip()
            
                if DEBUG:
                    rospy.loginfo( "Checking file %s", fname)

                
                # self.last_img is not new_img and
                if os.path.exists(os.path.join(pathname, new_img)):
                    self.read_img(os.path.join(pathname, new_img))
    
                    self.publish()
                    
                else:
                    if DEBUG:
                        rospy.loginfo("File %s does not exist", \
                                  os.path.join(pathname, new_img))

            
                rospy.sleep(1./self.refresh_rate)

                # Do we have to exit?
                if rospy.is_shutdown():
                    break 

            f.close()
# ------------------------------------------------------------------ #
def main(args):
    
    # ROS publisher startup
    ip = image_publisher()
    rospy.init_node('imagesender', anonymous=True)
    
    # Check correct number of arguments
    if len(args) < 3:
        rospy.loginfo( "Invalid number of arguments. Quitting...")
        return
    try:
        if DEBUG:
            rospy.loginfo("Starting program")
        ip.loopfiles(args[1], args[2])
    except KeyboardInterrupt:
        rospy.loginfo("Done. Shutting down")

# ------------------------------------------------------------------ #
if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass


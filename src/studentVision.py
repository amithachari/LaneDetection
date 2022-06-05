import time
import math
import numpy as np
import cv2
import rospy
import skimage
import colorsys
import matplotlib.pyplot as plt

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology



class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        #self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)


    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        # 1. Convert the image to gray scale
        # 2. Gaussian blur the image
        # 3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        # 4. Use cv2.addWeighted() to combine the results
        # 5. Convert each pixel to unint8, then apply threshold to get binary image

        grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('grayscale', grayscale)
        blur = cv2.GaussianBlur(grayscale, (3, 3), 0)
        # cv2.imshow('gaussian', blur)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        sobelx = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=3)

        alpha = 0.5
        beta = (1.0 - alpha)

        addweighted = cv2.addWeighted(sobelx, alpha, sobely, beta, 0.0)

        cvuint8 = cv2.convertScaleAbs(addweighted)
        # cv2.imshow('cvuint8', cvuint8)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # print(cvuint8)
        for i in range(0, np.shape(cvuint8)[0]):
            for j in range(0, np.shape(cvuint8)[1]):
                if cvuint8[i, j] < thresh_min or cvuint8[i, j] > thresh_max:
                    cvuint8[i, j] = 0
                else:
                    cvuint8[i, j] = 1
        plt.close('all')
        # print(cvuint8)
        # plt.imshow(cvuint8, cmap='binary')
        # plt.show()

        return cvuint8


    # def color_thresh(self, img, thresh=(100, 255)):
    #     """
    #     Convert RGB to HSL and threshold to binary image using S channel
    #     """
    #     #1. Convert the image from RGB to HSL
    #     #2. Apply threshold on S channel to get binary image
    #     #Hint: threshold on H to remove green grass
    #     ## TODO
    #     color_BGR = img
    #     color_HLS = cv2.cvtColor(color_BGR, cv2.COLOR_BGR2HLS)
    #     color_threshold_binary = np.zeros((np.shape(color_BGR)[0], np.shape(color_HLS)[1]))
    #     for i in range(0, np.shape(color_BGR)[0]):
    #         for j in range(0, np.shape(color_BGR)[1]):
    #             if color_HLS[i][j][2] < thresh[0] or color_HLS[i][j][2] > thresh[1]:
    #                 color_threshold_binary[i, j] = 0
    #             else:
    #                 color_threshold_binary[i, j] = 1
    #     plt.figure(1)
    #     plt.imshow(color_threshold_binary,cmap='binary')
    #     plt.show()
    #     # cv2.imshow("color_threshold", color_threshold_binary)
    #     # cv2.waitKey(0)
    #     ####

    #     return color_threshold_binary


    # def color_thresh(img, thresh_min = 100, thresh_max = 255):
    #     img_HLS = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    #     size = img_HLS.shape
    #     w, h = size[0], size[1]
    #     color_biout = np.zeros((w, h))
    #     for i in range(w):
    #         for j in range(h):
    #             if img_HLS[w-1, h-1, 2] >= thresh_min and img_HLS[w-1, h-1, 2] <= 255:
    #                 color_biout[i,j] = 1
    #             else:
    #                 color_biout[i,j] = 0
    #     return color_biout









    def color_thresh(self, img):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass
        ## TODO
        Hmax = 35
        Hmin = 20
        Lmin = 0
        Lmax = 255
        Smin = 0
        Smax = 25
        color_BGR = img
        color_HLS = cv2.cvtColor(color_BGR, cv2.COLOR_BGR2HLS)
        color_threshold_binary = np.zeros((np.shape(color_BGR)[0], np.shape(color_HLS)[1]))
        for i in range(0, np.shape(color_BGR)[0]):
            for j in range(0, np.shape(color_BGR)[1]):
                if color_HLS[i][j][0] < Hmax and color_HLS[i][j][0] > Hmin:
                    if color_HLS[i][j][1] > Lmin and color_HLS[i][j][2] > Smin:
                        color_threshold_binary[i, j] = 1
                elif color_HLS[i][j][1] > 140:
                    color_threshold_binary[i, j] = 1
                else:
                    color_threshold_binary[i, j] = 0
        # plt.figure(1)
        # plt.imshow(color_threshold_binary,cmap='binary')
        # plt.show()
        # cv2.imshow("color_threshold", color_threshold_binary)
        # cv2.waitKey(0)
        ####

        return color_threshold_binary


    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO
        SobelOutput = self.gradient_thresh(img)
        ColorOutput = self.color_thresh(img)
        ####

        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
        # print(binaryImage)
        # Remove noise from binary image
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)

        return binaryImage


    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TODO
        ### GEM 
	pts_base = np.float32([[0, 0], [0, 300], [300, 300], [300, 0]])
        # pts1 = np.float32([[250, 275], [30, 346], [574, 344], [430, 276]])
        pts1 = np.float32([[265, 263], [13, 371], [610, 365], [366, 263]])
 
        ###pts_base = np.float32([[0, 0], [0, 300], [300, 300], [300, 0]])
        # pts1 = np.float32([[250, 275], [30, 346], [574, 344], [430, 276]])
        ###pts1 = np.float32([[490, 247], [268, 371], [865, 371], [725, 247]])


        M = cv2.getPerspectiveTransform(pts1, pts_base)
        Minv = np.linalg.inv(M)
        imgint8 = img.astype(np.uint8)
        warped_img = cv2.warpPerspective(imgint8, M, (img.shape[1], img.shape[0]))
        warped_img = warped_img.astype(bool)
        warped_img = warped_img[:300,:300]
        plt.figure(1)
        plt.imshow(warped_img,cmap='binary')
        plt.show()
        ####

        return warped_img, M, Minv


    def detection(self, img):

        binary_img = self.combinedBinaryImage(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)

        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)

                if ret is not None:
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    self.detected = False

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
            else:
                print("Unable to detect lanes")

            return combine_fit_img, bird_fit_img


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)

#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LaneFollower(object):

    def __init__(self):

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.stop_sign = False
        self.stop_counter = 0

    def process(self, crop_img, hsv, height, width, range_down, range_top):
        mask = cv2.inRange(hsv, range_down, range_top)
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        cv2.circle(res, (int(cx), int(cy)), 10, (0, 0, 255), -1)

        return mask, res, cx, cy

    def calculate_distance_right_and_left(self, data, ishow=True):
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 100
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre + rows_to_watch)][1:width]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([50, 255, 255])

        lower_white = np.array([0, 0, 100])
        upper_white = np.array([0, 0, 255])

        mask_y, res_y, cx_y, cy_y = self.process(crop_img, hsv, height, width, lower_yellow, upper_yellow)
        mask_w, res_w, cx_w, cy_w = self.process(crop_img, hsv, height, width, lower_white, upper_white)

        if ishow:
            cv2.imshow("Original", cv_image)
            # cv2.imshow("HSV", hsv)
            # cv2.imshow("MASK_y", mask_y)
            cv2.imshow("RES_y", res_y)
            # cv2.imshow("MASK_w", mask_w)
            cv2.imshow("RES_w", res_w)

            cv2.waitKey(1)
        error_x_y = cx_y - width / 2
        error_x_w = width / 2 - cx_w

        return error_x_y, error_x_w

    def found_stop_sign(self, data, ishow=True):
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_image.shape
        descentre = 360
        rows_to_watch = 600
        crop_img = cv_image[300:height/2][1:width]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        red_lower_range = np.array([0, 80, 0])
        red_upper_range = np.array([20, 255, 255])

        mask_r, res_r, cx_r, cy_r = self.process(crop_img, hsv, height, width, red_lower_range, red_upper_range)

        if ishow:
            cv2.imshow("cv_image", cv_image)
            cv2.imshow("crop_img", crop_img)
            cv2.imshow("RES_r", res_r)

            cv2.waitKey(1)
        return cx_r > 1000

    def camera_callback(self, data):
        error_x_y, error_x_w = self.calculate_distance_right_and_left(data, False)
        new_stop_sign_val = self.found_stop_sign(data, False)
        move = Twist()

        print("Sign=" + str(new_stop_sign_val))
        if self.stop_sign is True and new_stop_sign_val is False:
            self.stop_counter += 1
            if self.stop_counter > 25:
                move.linear.x = 0.0
                move.angular.z = 0.0
                self.cmd_vel_pub.publish(move)
                return
        else:
            self.stop_sign = new_stop_sign_val

        move.linear.x = 0.2

        print("Counter=" + str(self.stop_counter))
        print("White=" + str(error_x_w))
        print("Yellow=" + str(error_x_y))
        if error_x_w > 670:
            print("==> Left 1")
            move.angular.z = 0.1
        elif error_x_w < 0:
            if error_x_y > 500:
                print("==> Right 2")
                move.angular.z = -0.2
            elif error_x_y < 150:
                print("==> Left 4")
                move.angular.z = 0.4
            elif error_x_y < 450:
                print("==> Left 4")
                move.angular.z = 0.2
            else:
                print("==> Straight 5")
                move.angular.z = 0.0
        elif error_x_w < 640:
            print("==> Right 6")
            move.angular.z = -0.1
        else:
            print("==> Straight 7")
            move.angular.z = 0.0
        self.cmd_vel_pub.publish(move)

    def clean_up(self):
        cv2.destroyAllWindows()


def main():
    rospy.init_node('lane_follow_node', anonymous=True)

    lane_follower_object = LaneFollower()

    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        lane_follower_object.clean_up()
        rospy.loginfo("shutdown time!")

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':
    main()

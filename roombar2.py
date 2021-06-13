import rospy
import pickle
import matplotlib.pyplot as plt
import cv2
import torch
import time
import math

import std_msgs.msg
from std_msgs.msg import String

import nav_msgs.msg
from nav_msgs.msg import Odometry

import sensor_msgs.msg
from sensor_msgs.msg import LaserScan

import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Twist

# import actionlib_msgs.msg
# from actionlib_msgs.msg import

import move_base_msgs.msg
from move_base_msgs.msg import MoveBaseActionGoal

import imaplib
import os
import email
import base64

class Roombar:

    """Initialiser"""
    def __init__(self):
        self.scan_ranges = [0.0]*4
        # self.qr_locations = {'Start Fridge': 0, 'End Fridge': 0, 'Rubbish': 0, 'Customer': 0}
        self.qr_locations = {}

        self.user_input = []
        # self.locations_detected = False
        # self.direction = ['LEFT_45', 'CENTRE', 'LEFT_90', 'LEFT_30']
        self.pose = None
        self.qr_code = None
        self.flagRight = 0
        # Init nodes
        rospy.init_node('roombar', anonymous=True)

        # Subscribe to the output of zbar
        rospy.Subscriber("barcode", String, self.qr_code_callback, queue_size = 2)
        rospy.Subscriber("odom_pose", Odometry, self.odom_callback, queue_size = 2)
        rospy.Subscriber("scan",LaserScan,self.lidar_callback,queue_size = 2)
        # rospy.Subscriber("")
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)
        self.goal_pub = rospy.Publisher("move_base_simple/goal",PoseStamped,queue_size = 2)


    # def qr_code_callback(self,barcode):
    #     self.qr_code = barcode.data

    def qr_code_callback(self,barcode):
        self.qr_code = barcode.data
        if self.qr_code not in self.qr_locations:
            qr_pose = self.pose.pose.pose #.pose.pose.position
            self.qr_locations[str(self.qr_code)] = qr_pose
            print(self.qr_code, qr_pose)

    def odom_callback(self,odom):
        self.pose = odom
        # self.pose.pose.pose.orientation.z =

    def lidar_callback(self,LaserScan):
        all_ranges = LaserScan.ranges
        scan_angles = [45, 0, 90, 30]

        for i in range(len(scan_angles)):
            self.scan_ranges[i] = all_ranges[scan_angles[i]]

    # def get_qr_pose(self,qr_code):
    #     # global self.scan_ranges, location_rubbish, location_customer, location_fridge_end, location_fridge_start, pose
    #     if self.pose is not None:
    #         qr_pose = self.pose.pose.pose.position

    #         if qr_code == 'Start Fridge':
    #             self.qr_locations['Start Fridge'] = qr_pose
    #             # location_fridge_start = qr_pose

    #         elif qr_code == "End Fridge":
    #             self.qr_locations['End Fridge'] = qr_pose
    #             # location_fridge_end = qr_pose

    #         elif qr_code == "Rubbish":
    #             self.qr_locations['Rubbish'] = qr_pose
    #             # location_rubbish = qr_pose

    #         elif qr_code == "Customer":
    #             self.qr_locations['Customer'] = qr_pose
    #             # location_customer = qr_pose

    #         else:
    #             print('Unrecognised')

    def add_qr_pose(self, new_pose):
        self.qr_poses.append(new_pose)

    def update_ranges(self, new_ranges):
        self.scan_ranges.clear()
        self.scan_ranges = new_ranges

    def pub_cmd_vel(self, linear, angular):
        cmd_vel = Twist()
        # print(cmd_vel)
        cmd_vel.linear.x  = linear
        cmd_vel.angular.z = angular
        self.cmd_vel_pub.publish(cmd_vel)
        # print(cmd_vel)

    def go_to_goal(self, dest):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose = dest
        self.goal_pub.publish(goal)
        rospy.sleep(10)

    def do_a_360(self):
        self.pub_cmd_vel(0,-1.3)
        rospy.sleep((2*math.pi)/1.2)
        self.pub_cmd_vel(0, 0)

        # while True:
        #
        #     if abs(self.scan_ranges[2] - left_dist) < 0.1:
        #         self.pub_cmd_vel(0,0)
        #         break

    def do_a_90(self):
        # left_dist = self.scan_ranges[2]
        self.pub_cmd_vel(0,-1.3)
        rospy.sleep((math.pi/2)/1.3)
        self.pub_cmd_vel(0, 0)

    def run_fridge_search(self, target_drink):
        self.go_to_goal(self.qr_locations["Start Fridge"])
        while self.qr_code != ("End Fridge" or target_drink):
            self.left_wall_follower()
        
        if self.qr_code == target_drink:

            self.do_a_360()

            # Bring back to customer
            self.go_to_goal(roombar.qr_locations["Customer"])

            # Wait until customer is finished
            while finished != "Yes":
                finished = raw_input("Finished? Yes/No: ")

            # Throw drink into rubbish
            self.go_to_goal(roombar.qr_locations["Rubbish"])
            self.do_a_360()
            self.user_input.pop()

        else:
            this_input = raw_input("Roombar cannot find your drink. Please select another: ")
            self.user_input.append(this_input)      


    def left_wall_follower(self):
        LEFT_45  = 0
        CENTER = 1
        LEFT_90  = 2
        LEFT_30  = 3
        STRAIGHT = 0.05

        # self.flagRight = 0
        # print("%.2f, %.2f, %.2f, %.2f"%(self.scan_ranges[0], self.scan_ranges[1], self.scan_ranges[2], self.scan_ranges[3]))

        # ------------------------------------STANDARD LEFT WALL FOLLOWER-----------------------------------
        # TURN RIGHT UNTIL there is nothing in its immediate front and there's something on the left
        if self.scan_ranges[CENTER] > 0.5 and self.scan_ranges[LEFT_90] < 0.5 and self.scan_ranges[LEFT_90] != 0:
            self.flagRight = 0
            # print("************ FINISHED TURNING RIGHT. RESUME COURSE ************")

        if self.flagRight == 0:
            # Turn left when   ros::Rate loop_rate(125)
            #45deg >>> 90 deg, ie. the robot is pointing too far away from the wall
            # Also turn left when there is a left hand corner
            if self.scan_ranges[LEFT_45] > 2.5*self.scan_ranges[LEFT_90] or (self.scan_ranges[LEFT_45] == 0 and self.scan_ranges[LEFT_90] > 0.1):
                # Turn left hard
                self.pub_cmd_vel(0.1, 1.0)
                # print("-------------Left hard")

            # Turn left if too far from the wall
            # Reduced this a little (Sam, Fri 12:30pm)
            elif self.scan_ranges[LEFT_90] >= 0.23 and self.scan_ranges[LEFT_45] >= 0.3:
                # Turn left slightly
                self.pub_cmd_vel(0.05, 0.2)
                # print("-------------Left slight")

            # Collision avoidance, try not to let the robot be heading directly towards the wall
            elif self.scan_ranges[LEFT_90] > self.scan_ranges[LEFT_45] or (self.scan_ranges[LEFT_90] == 0 and self.scan_ranges[LEFT_45] > 1):
                # Turn right medium
                self.pub_cmd_vel(0.05, -1*0.4)
                # print("-------------Right medium")

            # If something in front, something on left, hard turn right
            # If something in 1919front, nothing on left, hard turn right
            # I think self.scan_ranges[LEFT_45] < 0.5 has been put in twice.
            # else if (self.scan_ranges[CENTER] < 0.3 and self.scan_ranges[LEFT_45] < 0.5 and self.scan_ranges[LEFT_45] < 0.5
            #     or self.scan_ranges[CENTER] < 0.15 and self.scan_ranges[LEFT_90] > 2 or  self.scan_ranges[CENTER] < 0.2 and self.scan_ranges[LEFT_90] == 0)
            #
            elif (self.scan_ranges[CENTER] < 0.35 and self.scan_ranges[CENTER] !=0 and self.scan_ranges[LEFT_45] < 0.5) \
                or (self.scan_ranges[CENTER] < 0.15 and self.scan_ranges[CENTER] !=0 and self.scan_ranges[LEFT_90] > 2) \
                or (self.scan_ranges[CENTER] < 0.2 and self.scan_ranges[CENTER] !=0 and self.scan_ranges[LEFT_90] == 0):
                # Turn right hard
                self.flagRight = 1
                self.pub_cmd_vel(0.03, -1*1.3)
                # print("-------------Right hard")

            # If robot is too close to the wall
            elif self.scan_ranges[LEFT_90] < 0.18:
                # Turn right slightly
                self.pub_cmd_vel(0.05, -1 * 0.2)
                # print("-------------Right slight")


            # If robot has a wall that may block its wheel
            elif (self.scan_ranges[LEFT_45] < 0.29 or self.scan_ranges[LEFT_30] < 3*self.scan_ranges[CENTER] and self.scan_ranges[LEFT_30] < 0.25):
                # Turn right slightly
                self.pub_cmd_vel(0.03, -1 * 0.5)
                # print("-------------Right medium")


            # Backwards
            # Can consider uncommenting this, rarely used
            # else if (self.scan_ranges[LEFT_45]<0.1)
            #   prev_pose = pose
            #   # Back
            #   pub_cmd_vel(-0.1, -0.2)
            #   print("-------------Backwards")
            #
            # If everything is good, go straight
            # If something in front, nothing on the side, go straight
            else:
                # Go forward
                self.pub_cmd_vel(STRAIGHT, 0.0)
                # print("-------------Straight")


def read_emails(self):
    # Email Parameters
    port = 993
    email_user = 'roombarbot@gmail.com'
    email_pass = 'MTRX5700'
    mail = imaplib.IMAP4_SSL("imap.gmail.com", port)
    mail.login(email_user, email_pass)
    mail.select('Inbox')
    type, data = mail.search(None, 'ALL')
    mail_ids = data[0]
    id_list = mail_ids.split()


    for num in data[0].split():
        typ, data = mail.fetch(num, '(RFC822)')
        raw_email = data[0][1]

        # Decoding the email
        raw_email_string = raw_email.decode('utf-8')
        email_message = email.message_from_string(raw_email_string)

        # Downloading attachments from the email
        for part in email_message.walk():
            # If it contains body and attachment handle differently
            if part.get_content_maintype() == 'multipart':
                continue
            if part.get('Content-Disposition') is None:
                continue
            fileName = part.get_filename()
            if bool(fileName):
                filePath = os.path.join('', fileName)
                if not os.path.isfile(filePath) :
                    fp = open(filePath, 'wb')
                    fp.write(part.get_payload(decode=True))
                    fp.close()
                    subject = str(email_message).split("Subject: ", 1)[1].split("\nTo:", 1)[0]

    for response_part in data:
            if isinstance(response_part, tuple):
            # Splitting the email into respective parts
                msg = email.message_from_string(response_part[1].decode('utf-8'))
                email_subject = msg['subject']
                email_from = msg['from']
                email_body = msg['body']
                # print ('Subject : ' + email_subject + '\n')
                #print(msg.get_payload(decode=True))

        return email_subject

def main():
    try:
        print('start')
        # global qr_code
        # global pose

        # Initialsie roombar class
        roombar = Roombar()

        # goal_publisher = rospy.Publisher('move_base_simple/goal',PoseStamped,queue_size=1)
        prev_qr_code = roombar.qr_code

        # Initialisation
        # Start left wall following

        rate = rospy.Rate(125)
        rospy.sleep(2)
        while not rospy.is_shutdown():

            # If we haven't mapped all key locations
            if not ("Start Fridge" in roombar.qr_locations and \
                "End Fridge" in roombar.qr_locations and \
                "Customer" in roombar.qr_locations and \
                "Rubbish" in roombar.qr_locations):
                roombar.left_wall_follower()


            # If we have no user inputs
            elif not roombar.user_input:
                print("Everything mapped")
                roombar.pub_cmd_vel(0,0)
                roombar.go_to_goal(roombar.qr_locations["Customer"])
                # else:
                # print(roombar.qr_locations)
                subject = read_emails()
                if subject == ('Get Drink' or 'Customer'):
                    with open('drink.txt', "r") as file:  # Read the file
                        rows = file.readlines()
                        for row in rows:
                            roombar.user_input.append(row)

                    for this_input in roombar.user_input:
                        if this_input not in roombar.qr_locations:
                            print('Error with Email.')
                            this_input = raw_input("Manually choose drink(s) to fetch. Type drink names and separate with space: ")
                            input_list = this_input.split()
                            roombar.user_input = input_list
                            break

                elif subject == 'Rubbish':
                    roombar.go_to_goal(roombar.qr_locations["Rubbish"])
                else:
                    this_input = raw_input("Choose drink(s) to fetch. Type drink names and separate with space: ")
                    input_list = this_input.split()
                    roombar.user_input = input_list

                for this_input in roombar.user_input:
                    if this_input not in roombar.qr_locations:
                        fridge_search = raw_input("Do you want to search the fridge for", this_input, "? Yes/No: ")
                        if fridge_search == "Yes":
                            roombar.run_fridge_search(this_input)
                        else:
                            continue
            
            # If we have some user
            elif roombar.user_input:
                while roombar.user_input:
                    # Move to first input
                    target = roombar.user_input[0]
                    roombar.go_to_goal(roombar.qr_locations[target])
                    rospy.sleep(10)

                    # Pick up the drink
                    print('Picking up ', target)
                    roombar.do_a_360()
                    roombar.user_input.pop()


                # Bring back to customer
                roombar.go_to_goal(roombar.qr_locations["Customer"])
                rospy.sleep(10)

                # Wait until customer is finished
                while True:
                    finished = raw_input("Finished? Yes/No: ")
                    if finished == "Yes":
                        break
                    else:
                        print("Waiting")

                # Throw drink into rubbish
                roombar.go_to_goal(roombar.qr_locations["Rubbish"])
                rospy.sleep(10)
                print('Dropping the drink(s) in the bin')
                roombar.do_a_360()

                # Reset the list
                roombar.user_input = []

            rate.sleep()
            # rospy.spin()

        # while not rospy.is_shutdown():
        #     print(qr_code)
        rospy.spin()
        # Keep code open


    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        return
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        return


if __name__ == '__main__':
    main()

#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, Pose

import json

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import rospkg
import ros_numpy

import tf

class BebopCalibration():

    def __init__(self):

        rospack = rospkg.RosPack()
        positions_path = str(rospack.get_path('orb_slam')+'/config/maps/recorded_maps.json')

        rospy.init_node('Vel_Control_Node', anonymous=True)
        
        self.slam_pose_sub = rospy.Subscriber('/orb_slam2_mono/pose', PoseStamped, self.slam_pose_callback)
        self.decision = 0
        self.slam_pose = []

        config_file = rospy.get_param('~config_file',"default.json")

        try:
            with open(positions_path, 'r') as json_data_file:
                self.calibration_data = json.load(json_data_file)
        except:
            self.calibration_data = {"final_poses":[],"slam_poses":[]}
        
        self.map_name = "default"

        rospy.loginfo("Setup done!")
        

    def slam_pose_callback(self, data):

        self.slam_pose = ros_numpy.numpify(data.pose)
        self.count_callbacks += 1

        if self.count_callbacks > 30 and self.decision == 0:
            self.count_callbacks =0
            print (".")

            if self.map_name in self.calibration_data.keys() and len(self.calibration_data[self.map_name]) > 2:

                new_pose = np.dot(np.array(self.calibration_data[self.map_name]["pose_correction_matrix"]), self.slam_pose)
                new_pose = new_pose*self.calibration_data[self.map_name]["scale_factor_matrix"]
                print(ros_numpy.msgify(Pose,new_pose))

    def calculate_map_tf(self):

        x = np.array(self.calibration_data[self.map_name]["slam_poses"])
        y = np.array(self.calibration_data[self.map_name]["final_poses"])

        #calculate scale factor
        total_dist_slam = 0
        total_dist_final = 0
        c = 0
        for i in range(len(x)):
            for j in range(i+1, len(x)):
                print(i, j)
                total_dist_slam += np.sqrt(np.dot(x[i,:3,3]-x[j,:3,3],x[i,:3,3]-x[j,:3,3]))
                total_dist_final += np.sqrt(np.dot(y[i,:3,3]-y[j,:3,3],y[i,:3,3]-y[j,:3,3]))
                c += 1
        
        print(total_dist_final/c, total_dist_slam/c)
        scale_factor = total_dist_final/total_dist_slam
        print("scale_factor: ",str(scale_factor))

        #scale factor matrix multiplies only the cartesian coords of the pose
        sf_i = np.eye(4)
        sf_i *= scale_factor
        sf_i[3,3] = 1

        sfM = np.ones([4,4])
        sfM[:3,3] = scale_factor

        #yeah... i know that this is not the best solution, but it is working, and for now it is all that matters
        # tfM = np.dot(np.dot(np.linalg.inv(y[0]*sf_m),x[0]),sf_i)

        tfM = np.dot(np.dot(np.linalg.inv(np.dot(sf_i,x[0])),y[0]),sf_i)

        return tfM, sfM

    def run(self):
        while decision != 4 :
            rospy.loginfo("Select map: " + self.map_name)
            if self.map_name in self.calibration_data.keys() and len(self.calibration_data[self.map_name]) > 2:
                rospy.logwarn("Map already calibrated")

            else:
                rospy.loginfo("Map is not calibrated! Calibration data:")

            if self.map_name in self.calibration_data.keys():
                rospy.loginfo(self.calibration_data[self.map_name]) 
                rospy.loginfo("Please record at least " + str(2-len(self.calibration_data[self.map_name]["final_poses"]))+" more position")

            rospy.logwarn(' 1: Record this position as the new global position:\n 2: Change map\n 3: Reset calibration\n 4: Exit')
            rospy.loginfo("\n dots \".\" means that the Slam is publishing some pose")

            decision = 0
            decision = input()

            if decision == 1:
                if len(self.slam_pose)>0:
                    coord_raw = input("new coord (x,y,z) or (x,y,z,R,P,Y):")
                    coord = list(coord_raw)
                    if len(coord_raw)==3:
                        coord += [0,0,0]
                        rospy.loginfo("Compliting rotation with 0s")

                    if not len(coord_raw)==6:
                        rospy.logwarn("Error, make sure you are using comma separeted values")
                    else:
                        given_pose = Pose()
                        quarterion = tf.transformations.quaternion_from_euler(coord[3], coord[4], coord[5])
                        # print(quaternion)
                        given_pose.orientation.x,given_pose.orientation.y,given_pose.orientation.z,given_pose.orientation.w = quarterion
                        given_pose.position.x,given_pose.position.y,given_pose.position.z = coord[:3]

                        if map_name in self.calibration_data.keys():
                        
                            self.calibration_data[map_name]["slam_poses"] = self.calibration_data[map_name]["slam_poses"]+[self.slam_pose.tolist()]
                            self.calibration_data[map_name]["final_poses"] = self.calibration_data[map_name]["final_poses"]+[ros_numpy.numpify(given_pose).tolist()]
                            if len(self.calibration_data[map_name]["final_poses"])<2:
                                rospy.loginfo("Please record at least " + str(2-len(self.calibration_data[map_name]["final_poses"])) + " more position")
                            else:
                                print("calibrated")
                                tfM, sfM = self.calculate_map_tf()
                                self.calibration_data[map_name]["pose_correction_matrix"] = tfM.tolist()
                                self.calibration_data[map_name]["scale_factor_matrix"] = sfM.tolist()
                        else:
                            self.calibration_data[map_name]= {"slam_poses":[self.slam_pose.tolist()],"final_poses":[ros_numpy.numpify(given_pose).tolist()]}
                            print("please record at least"+str(2-len(self.calibration_data[map_name]["final_poses"]))+"more position")
                        print(self.calibration_data)
                        with open(self.positions_path, 'w') as json_data_file:
                            json.dump(self.calibration_data, json_data_file)
                        print("position saved")
                else:
                    print("no positions have been received yet!\nplease check if the node \"orb_slam2_ros\" is running")
            if decision == 2:
                print("existing maps:")
                print(self.calibration_data.keys())
                map_name = str(input("please write the name of the map edit or create: "))
                if map_name in self.calibration_data.keys():
                    print("existing map selected!")    
                    print(self.calibration_data[map_name])
                else:
                    print("new map selected!")
            if decision == 3: #delete map
                if str(input("are you sure? (y/N):")) == "y":
                    self.calibration_data.pop(map_name,None)
                    map_name = "default"
                    print("map deleted")
                    with open(self.positions_path, 'w') as json_data_file:
                                json.dump(self.calibration_data, json_data_file)

bebop = BebopCalibration()
rospy.sleep(3)
print(bebop.slam_pose)
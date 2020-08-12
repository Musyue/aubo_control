#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
from aubo_robotcontrol import *
import time
import numpy
import os
import socket
from aubo_kienamatics import *
from sensor_msgs.msg import JointState
import yaml
class AuboCollisionCheck():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
    def Init_node(self):
        rospy.init_node("aubo_collision_cehck")

    def deg_to_rad(self,tuplelist):
        dd = []
        for i in tuplelist:
            dd.append(i * pi / 180)
        return tuple(dd)
    def rad_to_degree(self,tuplelist):
        dd=[]
        for i in tuplelist:
            dd.append(i*180/math.pi)
        return dd
    def pub_state(self,robot_state):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = 'yue'
        js.name = ["base_joint1", 
                    "base_joint2",
                    "mobilebase_joint",
                    "rodclimbing_joint",
                    "shoulder_joint",
                    "upperArm_joint",
                    "foreArm_joint",
                    "wrist1_joint",
                    "wrist2_joint",
                    "wrist3_joint"
                    ]
        js.position = [robot_state[0],robot_state[1],robot_state[2],robot_state[3],robot_state[4],robot_state[5],robot_state[6],robot_state[7],robot_state[8],robot_state[9]]
        js.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        js.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub_state_.publish(js)
    def list_change_3_7_11(self,inital_list_data,list_data):
        temp=[]
        for i in range(len(inital_list_data)):
            if i==3:
                temp.append(list_data[0])
            elif i==7:
                temp.append(list_data[1])
            elif i==11:
                temp.append(list_data[2])
            else:
                temp.append(inital_list_data[i])
        return temp
    def deg_to_rad(self,listdata):
        dd=[]
        for i in listdata:
            dd.append(i*pi/180)
        return dd
def main():
    
    ratet=1
    Aub=AuboCollisionCheck()
    Aub.Init_node()
    o1=[]
    o2=[]
    o3=[]
    o4=[]
    o5=[]
    o6=[]
    rate = rospy.Rate(ratet)
    q_ref=[6.33,18.66,142.092,120.32,86.375,0.101]
    q_ref_rad=Aub.deg_to_rad(q_ref)
    # print(q_ref_rad)


    r_max_from_aubo=1.5
    D_distance=0.41
    r_cut = sqrt(r_max_from_aubo*r_max_from_aubo-D_distance*D_distance)
    
    o1.append(D_distance)
    o1.append(-1.0*sqrt(2)*r_cut/3)
    o1.append(sqrt(2)*r_cut/4)
    
    o2.append(D_distance)
    o2.append(0.0)
    o2.append(sqrt(2)*r_cut/4)

    o3.append(D_distance)
    o3.append(sqrt(2)*r_cut/3)
    o3.append(sqrt(2)*r_cut/4)

    o4.append(D_distance)
    o4.append(sqrt(2)*r_cut/3)
    o4.append(-sqrt(2)*r_cut/4)

    o5.append(D_distance)
    o5.append(0.0)
    o5.append(-sqrt(2)*r_cut/4)

    o6.append(D_distance)
    o6.append(-1.0*sqrt(2)*r_cut/3)
    o6.append(-sqrt(2)*r_cut/4)

    Aubo_k=Aubo_kinematics()

    count=0
    six_point=[o1,o2,o3,o4,o5,o6]
    count_o=0
    q_sol_without_collistion_dict={}
    num_count=0
    flag=1
    judge_self_collision_list=[]
    judge_count_flag=0
    judge_self_collision_flag=rospy.get_param('judge_self_collision_flag')
    rospy.logerr("judge =="+str(judge_self_collision_flag))
    judge_self_collision_list.append(judge_self_collision_flag)
    last_state=[]
    while not rospy.is_shutdown():

        if flag==1:
            # temp=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            temp1=[0.0,0.0,0.0,0.0]
            Aub.pub_state(temp1+q_ref_rad)
            flag=0
        else:
            if count_o<=5:
                new_t=Aub.list_change_3_7_11(Aubo_k.aubo_forward(q_ref),six_point[count_o])
                q_dict=Aubo_k.GetInverseResult_without_ref(new_t)
                
                # print(len(q_dict))
                if len(q_dict)!=0:
                    if count< len(q_dict):
                        temp=[0.0,0.0,0.0,0.0]


                        Aub.pub_state(temp+q_dict[count])
                        # print(q_dict[count],count)
                        # print(Aub.rad_to_degree(q_dict[count]))

                        judge_self_collision_flag=rospy.get_param('judge_self_collision_flag')

                        if count!=0:
                            rospy.loginfo("haha--> "+str(count_o))
                            rospy.logerr("last state pub judge =="+str(judge_self_collision_flag)+" "+str(q_dict[count-1]))
                        

                        # judge_self_collision_list.append(judge_self_collision_flag)
                        # if len(judge_self_collision_list)>=2:
                        #     if judge_self_collision_list[-2]==False:
                        #         rospy.logerr("The o : "+str(count_o)+" judge_self_collision sol "+str(count)+" ["+str(judge_self_collision_list[-2])+"]")
                        #         # print(q_sol_without_collistion_dict,count)
                        #         # temp=[0.0,0.0,0.0,0.0]
                        #         # Aub.pub_state(temp+q_dict[count-1])

                        #         q_sol_without_collistion_dict.update({num_count:q_dict[count]})
                        #         num_count+=1
                        count+=1
                        
                    else:

                        # print(q_dict[count],count)
                        count=0
                        count_o+=1
                        # num_count=0
                        # judge_self_collision_list=[]
                        # # print(q_sol_without_collistion_dict)
                        # if len(q_sol_without_collistion_dict)!=0:
                        #     rets,q_choose=Aubo_k.chooseIKonRefJoint(q_sol_without_collistion_dict,q_ref_rad)
                        #     # temp=[0.0,0.0,0.0,0.0]
                        #     # Aub.pub_state(temp+q_choose)
                        #     rospy.logerr("q_choose"+str(q_choose))
                        #     print(Aub.rad_to_degree(q_choose))
                        #     flag=0
                        # q_sol_without_collistion_dict={}
                        # break
                
                
               



        rate.sleep()
            

if __name__ == '__main__':
    main()
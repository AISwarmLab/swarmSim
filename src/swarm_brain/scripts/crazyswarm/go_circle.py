import sys
from os.path import abspath, join, dirname
sys.path.insert(0, join(abspath(dirname(__file__))))

import rospy
from pycrazyswarm.srv import *
import sys
from rospy import Duration
from geometry_msgs.msg import Point
import random
import uav_trajectory
from pycrazyswarm.msg import *
import tf_conversions
import numpy as np
from threading import Thread
# from pycrazyswarm import Crazyswarm
import re
import math



Z = 1.0
sleepRate = 30
quad_num = len(rospy.get_param('crazyflies'))
cf_swarm_state = dict()
cf_swarm_publishers = dict()

for id in range(1, quad_num+1):
    topic_cmd_velocity = "/cf" + str(id) + "/cmd_velocity_world"
    pub =  rospy.Publisher(topic_cmd_velocity, VelocityWorld, queue_size=10)
    cmdVelocityWorldMsg = VelocityWorld()
    cmdVelocityWorldMsg.header.seq = 0
    cmdVelocityWorldMsg.header.frame_id = "/cf" + str(id)
    cf_swarm_publishers[id] = [pub, cmdVelocityWorldMsg]

def circle_pos_calc(num_cf, center_pt, radius):
    goals = np.zeros((3,num_cf))

    for i in range(num_cf):
        goals[0,i] = center_pt[0] + radius * math.cos(i * 2 * math.pi / num_cf)
        goals[1,i]  = center_pt[1] + radius * math.sin(i * 2 * math.pi / num_cf)
        goals[2,i] = center_pt[2]

    return goals

def take_off(height):
    # 执行起飞操作的代码
    for id in range(1,quad_num+1):
        req = TakeoffRequest(id, 0, height, Duration(2.5))
        takeoff_client.call(req)

    # rospy.loginfo("飞机起飞")

def land():
    # 执行降落操作的代码
    for id in range(1,quad_num+1):
        req = LandRequest(id, 0, 0.04, Duration(2.5))
        land_client.call(req)

    rospy.loginfo("飞机降落")

def goto():
    center_pt = np.array([0.0, 0.0, 1.5])
    radius = 2.0
    goals = circle_pos_calc(quad_num, center_pt, radius)
    for id in range(1, quad_num+1):
        goal = Point()
        goal.x = goals[0,id-1]
        goal.y = goals[1,id-1]
        goal.z = goals[2,id-1]
        req = GoToRequest(id, 0, False, goal, 0.0, Duration(2.5))
        goto_client.call(req)

    rospy.loginfo("飞机到达指定位置")

def subPosition_CallBack(data):
        cf_id = int(re.findall(r'\d+', data.header.frame_id)[0])
        cf_swarm_state[cf_id] = data
        # print(data)

def handler1():
    for id in range(1,quad_num+1):
        topic_pos = "/cf" + str(id) + "/position"
        rospy.Subscriber(topic_pos, Position, subPosition_CallBack)

    rospy.spin()

def handler2(start_time):
    totalTime = 3.0
    center_circle = np.array([0.0, 0.0, 1.5])
    omega = 2 * np.pi / totalTime
    theta_delta = 2*math.pi/quad_num
    radius = 2.0
    kPosition = 1.0
    rate = rospy.Rate(10)
    goto_flag = False
    take_off_flag = False
    land_flag = False
    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - start_time).to_sec()

        if elapsed_time < 5:
            if not take_off_flag:
                height = 1.5  # 提示用户输入起飞高度
                take_off(height)
                take_off_flag = True
        elif elapsed_time < 10:
            if not goto_flag:
                goto()
                goto_flag = True
        elif elapsed_time < 30:
            # print("circle")
            for cf_id, state in cf_swarm_state.items():
                # print("cf_id:", cf_id)
                theta = (cf_id) * theta_delta

                vx = -radius * omega * np.sin(omega * elapsed_time + theta)
                vy = radius * omega * np.cos(omega * elapsed_time + theta)
                desiredPos = center_circle + radius * np.array(
                    [np.cos(omega * elapsed_time + theta), np.sin(omega * elapsed_time + theta), 0])
                errorX = desiredPos - np.array([state.x, state.y, state.z])

                pub, msg = cf_swarm_publishers[cf_id]
                msg.header.stamp = rospy.Time.now()
                msg.header.seq += 1
                msg.vel.x = vx + kPosition * errorX[0]
                msg.vel.y = vy + kPosition * errorX[1]
                msg.vel.z = kPosition * errorX[2]
                msg.yawRate = 0.0
                pub.publish(msg)

            rate.sleep()
        else:
            if not land_flag:
                land()
                land_flag = True








if __name__ == "__main__":
    rospy.init_node("Crazyswarm_GoCircle")

    startTime = rospy.Time.now()

    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv(dirname(__file__)+"/figure8.csv")

    takeoff_client = rospy.ServiceProxy("Takeoff", Takeoff)
    takeoff_client.wait_for_service()

    land_client = rospy.ServiceProxy("Land", Land)
    land_client.wait_for_service()

    goto_client = rospy.ServiceProxy("GoTo", GoTo)
    goto_client.wait_for_service()

    upload_traj_client = rospy.ServiceProxy("UploadTrajectory", UploadTrajectory)
    upload_traj_client.wait_for_service()

    start_traj_client = rospy.ServiceProxy("StartTrajectory", StartTrajectory)
    start_traj_client.wait_for_service()

    t1 = Thread(target=handler1, args=())
    t2 = Thread(target=handler2, args=(startTime,))

    t1.start()
    t2.start()
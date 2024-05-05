import sys
from os.path import abspath, join, dirname
sys.path.insert(0, join(abspath(dirname(__file__))))
print(abspath(dirname(__file__)))

import pybullet as p
import rospy
from nav_msgs.msg import Odometry
from pycrazyswarm.msg import *
import pybullet_data
import re
from threading import Thread
import random
import numpy as np
import tf

def robot_pose_callback(msg):
    agent_id = int(re.findall(r'\d+', msg.header.frame_id)[0])
    new_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, 0.01]
    orientation = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(orientation)
    # new_orien= [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]

    p.resetBasePositionAndOrientation(robotIds[agent_id-1], new_pos, p.getQuaternionFromEuler([1.57, euler[1], euler[2]]))

def drone_pose_callback(msg):
    cf_id = int(re.findall(r'\d+', msg.header.frame_id)[0])
    new_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    new_orien= [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]

    trajectory_dict[msg.header.frame_id].append(new_pos)
    p.resetBasePositionAndOrientation(droneIds[cf_id-1], new_pos, new_orien)



def handler1():
    rate = rospy.Rate(50)
    # 循环更新并绘制无人机位置
    while not rospy.is_shutdown():
        # 更新物理仿真
        p.stepSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        rate.sleep()


def handler2():
    yaw = 0.0
    pitch = -45.0
    camera_target_pos = [0, 0, 0]

    # 循环运行pybullet引擎
    while True:
        # 从键盘读取按键输入
        keys = p.getKeyboardEvents()

        # 根据按键输入更新相机位置和目标位置
        for key, state in keys.items():
            if key == p.B3G_UP_ARROW and (state & p.KEY_WAS_TRIGGERED):
                pitch -= 5.0
            elif key == p.B3G_DOWN_ARROW and (state & p.KEY_WAS_TRIGGERED):
                pitch += 5.0
            elif key == p.B3G_LEFT_ARROW and (state & p.KEY_WAS_TRIGGERED):
                yaw -= 5.0
            elif key == p.B3G_RIGHT_ARROW and (state & p.KEY_WAS_TRIGGERED):
                yaw += 5.0

        # 更新相机位置和目标位置
        p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=yaw, cameraPitch=pitch,
                                     cameraTargetPosition=camera_target_pos)

def handler3():

    line_width = 3

    while True:
        # 获取键盘事件
        events = p.getKeyboardEvents()

        # 检查是否按下了 't' 键
        if ord('t') in events:
            # 执行相应的操作
            print("键盘按下了 't'。")


            for key, traj in trajectory_dict.items():
                print(key, traj)
                line_color = [random.random(), random.random(), random.random()]
                for i in range(len(traj) - 1):
                    line_start_point = traj[i]
                    line_end_point = traj[i + 1]
                    p.addUserDebugLine(
                        lineFromXYZ=line_start_point, lineToXYZ=line_end_point,
                        lineColorRGB=line_color, lineWidth=line_width)

            break




if __name__ == "__main__":
    rospy.init_node('pybullet_drone_display')

    # 设置物理仿真环境
    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

    p.setGravity(0, 0, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 设置pybullet数据路径
    planeId = p.loadURDF("plane.urdf")
    p.setRealTimeSimulation(True)

    trajectory_dict = dict()

    quad_num = len(rospy.get_param('crazyflies'))



    # 添加无人机模型
    droneIds = []
    for id in range(quad_num):
        # pybullet init
        drone_urdf = dirname(__file__) + "/assets/cf2p.urdf"
        droneId = p.loadURDF(drone_urdf)
        droneIds.append(droneId)

        # ros
        topic_pos = "/cf" + str(int(id + 1)) + "/fullstate"
        rospy.Subscriber(topic_pos, FullState, drone_pose_callback)

        trajectory_dict["/cf"+str(int(id + 1))] = []

    t1 = Thread(target=handler1, args=())
    t2 = Thread(target=handler2, args=())
    t3 = Thread(target=handler3, args=())

    t1.start()
    t2.start()
    t3.start()







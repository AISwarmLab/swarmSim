import sys
from os.path import abspath, join, dirname
sys.path.insert(0, join(abspath(dirname(__file__))))


from utilities.transformations import *
from utilities.barrier_certificates import *
from utilities.misc import *
from utilities.controllers import *

import numpy as np
import rospy
from threading import Thread
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import re
import tf


# 获取无人车数量
kkswarm_num = rospy.get_param('num_robot')

kkswarm_state = dict()
kkswarm_publishers = dict()

# 定义无人车集群publishers
for id in range(1, kkswarm_num+1):
    topic_cmd_velocity = "/robot_" + str(id) + "/cmd_vel"
    pub =  rospy.Publisher(topic_cmd_velocity, Twist, queue_size=10)
    cmd_vel_msg = Twist()
    kkswarm_publishers[id] = [pub, cmd_vel_msg]


# 订阅无人车状态回调汉书
def subPose_CallBack(data):
        agent_id = int(re.findall(r'\d+', data.header.frame_id)[0])
        kkswarm_state[agent_id] = data

# 设置无人车集群subscriber
def handler1():
    for id in range(1,kkswarm_num+1):
        topic_pos = "/robot_" + str(id) + "/pose"
        rospy.Subscriber(topic_pos, Odometry, subPose_CallBack)

    rospy.spin()

# 无人车集群控制核心代码
def handler2():
    si_barrier_cert = cbf_single_integrator(barrier_gain=100, safety_radius=0.3, magnitude_limit=0.35)

    # 创建单积分位置控制器
    si_position_controller = create_si_position_controller()

    # 创建单积分与独轮车模型转换函数
    si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

    # 设置控制频率25Hz
    rate = rospy.Rate(50)

    # 设置算法相关参数 循环次数 内外半径 边界 交换的位置点
    num_cycles=10
    count = -1
    flag = 0

    radius = 0.8
    radius2 = 1.8
    xybound = radius*np.array([-1, 1, -1, 1])
    p_theta = 2*np.pi*(np.arange(0, 2*kkswarm_num/2, 2)/(2*kkswarm_num/2))
    p_circ = np.vstack([
                np.hstack([xybound[1]*np.cos(p_theta), xybound[1]*np.cos(p_theta+np.pi)]),
                np.hstack([xybound[3]*np.sin(p_theta), xybound[3]*np.sin(p_theta+np.pi)])
                ])
    
    xybound2 = radius2*np.array([-1, 1, -1, 1])
    p_theta2 = 2*np.pi*(np.arange(0, 2*int(kkswarm_num/2), 2)/(2*int(kkswarm_num/2)))
    p_circ2 = np.vstack([
            np.hstack([xybound2[1]*np.cos(p_theta2), xybound2[1]*np.cos(p_theta2+np.pi)]),
            np.hstack([xybound2[3]*np.sin(p_theta2), xybound2[3]*np.sin(p_theta2+np.pi)])
            ])
    x_goal = p_circ[:, :int(kkswarm_num/2)]
    x_goal2 = p_circ2[:, :int(kkswarm_num/2)]
    x_goals = np.concatenate((x_goal, x_goal2), axis=1)

    kkswarm_poses = np.zeros((3, kkswarm_num))

    # 主循环
    while not rospy.is_shutdown():
        if len(kkswarm_state) < kkswarm_num:
            continue
        for agent_id in range(1,kkswarm_num+1):
            odom = kkswarm_state[agent_id]
            kkswarm_poses[0, agent_id - 1] = odom.pose.pose.position.x
            kkswarm_poses[1, agent_id - 1] = odom.pose.pose.position.y
            orientation = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(orientation)
            kkswarm_poses[2, agent_id - 1] = euler[2]

        x_si = uni_to_si_states(kkswarm_poses)

        # 判断是否每个无人车是否到达目标点
        if(np.linalg.norm(x_goals - x_si) < 0.05):
            flag = 1-flag
            count += 1

        # 是否达到最大循环运行次数
        if count == num_cycles:
            break

        # 判断是否改变交换的位置点
        if(flag == 0):
            x_goal = p_circ[:, :int(kkswarm_num/2)]
            x_goal2 = p_circ2[:, :int(kkswarm_num/2)]
        else:
            x_goal = p_circ[:, int(kkswarm_num/2):]
            x_goal2 = p_circ2[:, int(kkswarm_num/2):]
        x_goals = np.concatenate((x_goal, x_goal2), axis=1)

        # 根据位置和目标点计算速度矢量
        dxi = si_position_controller(x_si,x_goals)

        # 考虑避障情况
        dxi = si_barrier_cert(dxi, x_si)

        # 将单积分转换线速度和角速度
        dxu = si_to_uni_dyn(dxi, kkswarm_poses)

        # 发布无人车控制指令
        for agent_id in range(1, kkswarm_num + 1):
            pub,msg = kkswarm_publishers[agent_id]
            msg.linear.x = dxu[0, agent_id - 1]
            msg.angular.z = dxu[1, agent_id - 1]
            pub.publish(msg)


        rate.sleep()

def main():
    # 初始化ROS节点
    rospy.init_node("swarm_brain")

    # 启动多线程
    t1 = Thread(target=handler1, args=())
    t2 = Thread(target=handler2, args=())

    t1.start()
    t2.start()

if __name__ == "__main__":
    main()




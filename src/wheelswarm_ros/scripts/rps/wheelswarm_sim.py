import sys
from os.path import abspath, join, dirname
sys.path.insert(0, join(abspath(dirname(__file__))))

import kkrobot
import numpy as np
import rospy
import random
import math

def main():
    # 获取无人车的数量
    N = rospy.get_param('num_robot')

    # 初始化ROS节点
    rospy.init_node("kkswarm_simulator", anonymous=True)

    # 自定义初始位置
    init_poses = np.zeros((3,N))

    for i in range(N):
        if i < int(N/2):
            radius = random.uniform(0, 1)
            theta = random.uniform(0, 2*math.pi)
            init_poses[0, i] = radius*math.cos(theta)
            init_poses[1, i] = radius*math.sin(theta)
            init_poses[2, i] = np.random.rand()*2*np.pi - np.pi
        else:
            break
    
    i = int(N/2)
    while i < N:
        radius = random.uniform(1, 2)
        theta = random.uniform(0, 2*math.pi)
        init_poses[0, i] = radius*math.cos(theta)
        init_poses[1, i] = radius*math.sin(theta)
        
        if (init_poses[0, i] < 1.0 and init_poses[0, i]> -1.0) or (init_poses[1, i] < 1.0 and init_poses[1, i]> -1.0):
            continue
        else:
            init_poses[2, i] = np.random.rand()*2*np.pi - np.pi
            i+=1


    # 初始化KKSwarm仿真器引擎
    r = kkrobot.KKRobot(number_of_robots=N, show_figure=True, sim_in_real_time=True, initial_conditions = init_poses)
    rate = rospy.Rate(1.0 / r.time_step)

    # 启动KKSwarm仿真器引擎
    while not rospy.is_shutdown():
        r.step()
        rate.sleep()

if __name__ == "__main__":
    main()




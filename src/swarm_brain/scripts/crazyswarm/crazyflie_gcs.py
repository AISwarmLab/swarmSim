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

quad_num = len(rospy.get_param('crazyflies'))

def take_off(height):
    # 执行起飞操作的代码
    for id in range(1,quad_num+1):
        req = TakeoffRequest(id, 0, height, Duration(2.5))
        takeoff_client.call(req)

    rospy.loginfo("飞机起飞")


def land():
    # 执行降落操作的代码
    for id in range(1,quad_num+1):
        req = LandRequest(id, 0, 0.04, Duration(2.5))
        land_client.call(req)

    rospy.loginfo("飞机降落")

def goto():

    for id in range(1, quad_num+1):
        goal = Point()
        goal.x = random.uniform(-4, 4)
        goal.y = random.uniform(-4, 4)
        goal.z = 2.0
        req = GoToRequest(id, 0, False, goal, 0.0, Duration(2.5))
        goto_client.call(req)

    rospy.loginfo("飞机到达指定位置")

def upload_traj(traj1):

    for id in range(1,quad_num+1):
        req = UploadTrajectoryRequest(id, 0, 0, traj1.polynomials)
        upload_traj_client.call(req)

    rospy.loginfo("上传飞机轨迹")


def start_traj():

    for id in range(1,quad_num+1):
        req = StartTrajectoryRequest(id, 0, 0, 1.0, False, True)
        start_traj_client.call(req)

    rospy.loginfo("按轨迹飞行")



def cmd_full_state():
    pos = [3.0,3.0,1.5]
    vel = [1.0,1.0,0.0]
    acc = [0.0,0.0,0.0]
    yaw = 0.0
    omega = [0.0,0.0,0.0]
    cmdFullStateMsg.header.stamp = rospy.Time.now()
    cmdFullStateMsg.header.seq += 1
    cmdFullStateMsg.pose.position.x = pos[0]
    cmdFullStateMsg.pose.position.y = pos[1]
    cmdFullStateMsg.pose.position.z = pos[2]
    cmdFullStateMsg.twist.linear.x = vel[0]
    cmdFullStateMsg.twist.linear.y = vel[1]
    cmdFullStateMsg.twist.linear.z = vel[2]
    cmdFullStateMsg.acc.x = acc[0]
    cmdFullStateMsg.acc.y = acc[1]
    cmdFullStateMsg.acc.z = acc[2]
    cmdFullStateMsg.pose.orientation = geometry_msgs.msg.Quaternion(
        *tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
    cmdFullStateMsg.twist.angular.x = omega[0]
    cmdFullStateMsg.twist.angular.y = omega[1]
    cmdFullStateMsg.twist.angular.z = omega[2]

    cmdFullStatePublisher.publish(cmdFullStateMsg)

    rospy.loginfo("精准飞行")

def cmd_pos():
    pos = [3.0, 3.0, 1.5]
    yaw = 0.0
    cmdPositionMsg.header.stamp = rospy.Time.now()
    cmdPositionMsg.header.seq += 1
    cmdPositionMsg.x = pos[0]
    cmdPositionMsg.y = pos[1]
    cmdPositionMsg.z = pos[2]
    cmdPositionMsg.yaw = yaw
    cmdPositionPublisher.publish(cmdPositionMsg)

    rospy.loginfo("向指定位置飞行-低级")

if __name__ == "__main__":
    rospy.init_node("Crazyswarm_GCS")

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

    cmdVelocityWorldPublisher = rospy.Publisher("/cf1/cmd_velocity_world", VelocityWorld, queue_size=1)
    cmdVelocityWorldMsg = VelocityWorld()
    cmdVelocityWorldMsg.header.seq = 0
    cmdVelocityWorldMsg.header.frame_id = "/cf1"

    cmdFullStatePublisher = rospy.Publisher("/cf1/cmd_full_state", FullState, queue_size=1)
    cmdFullStateMsg = FullState()
    cmdFullStateMsg.header.seq = 0
    cmdFullStateMsg.header.frame_id = "/cf1"

    cmdPositionPublisher = rospy.Publisher("cf1/cmd_position", Position, queue_size=1)
    cmdPositionMsg = Position()
    cmdPositionMsg.header.seq = 0
    cmdPositionMsg.header.frame_id = "/cf1"

    while not rospy.is_shutdown():
        command = input("请输入指令（take_off/land/goto/upload/fly/quit）：")  # 等待用户输入指令

        if command == "take_off":
            height = 0.5  # 提示用户输入起飞高度
            take_off(height)
        elif command == "land":
            land()
        elif command == "goto":
            goto()
        elif command == "upload":
            upload_traj(traj1)
        elif command == "fly":
            start_traj()
        elif command == "full":
            cmd_full_state()
        elif command == "pos":
            cmd_pos()
        elif command == "quit":
            break
        else:
            rospy.logwarn("无效的指令")
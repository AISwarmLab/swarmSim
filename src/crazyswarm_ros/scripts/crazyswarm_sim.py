import sys
from os.path import abspath, join, dirname
sys.path.insert(0, join(abspath(dirname(__file__))))

from pycrazyswarm import Crazyswarm
import rospy
import math
from pycrazyswarm.srv import *





def main():
    rospy.init_node('crazyswarm_sim', anonymous=True)

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    while not rospy.is_shutdown():
        timeHelper.step(timeHelper.dt)

        timeHelper.rate.sleep()

if __name__ == "__main__":
    main()

# AISwarmLab

<img src="./src/imgs/logo.jpg">

智能蜂群实验室（AI SwarmLab®）依托华东师范大学信息学部，汇集了 一支充满激情和创新的团队，致力于协同控制、强化学习、智能博弈、大模型等先进技术研究，专注于为研究人员和工程师提供跨域无人系统大规模集群虚实结合的轻量级可信仿真平台，为集群协同和博弈对抗算法提供集群数量规模可拓、场景复杂度可变、集群动态拓扑可调的综合解决方案。 



## 环境安装

操作系统要求：Ubuntu20.04

ROS：Noetic

Python：Python3.10

Mamba：Mambaforge-pypy3-Linux-x86_64

## 1. 安装Mamba

下载Mamba安装包

在终端输入`bash Mambaforge-pypy3-Linux-x86_64.sh`，并按照提示缺省安装。

## 2.安装Python和ROS

将swarm_simulator_ws源代码下载到本地，启动终端，进入源码目录，如

`cd ~/swarm_simulator_ws/src`

在终端输入`source ~/mambaforge-pypy3/bin/activate`，激活base环境。

激活base环境后，在终端输入`mamba env create -n ros_env_py310 --file ros_env_py310.yaml`

安装完成依赖环境后，关闭终端。
另外启动一个终端，激活ros_env_py310 环境，依次输入

`source ~/mambaforge-pypy3/bin/activate/ros_env_py310`

激活ros_env_py310 环境后，输入roscore。若有报错，请删除home文件夹下的.ros文件夹，重启，再次激活ros_env_py310 环境，输入roscore。

## 3.编译源代码

将swarm_simulator_ws源代码下载到本地，启动终端，激活ros_env_py310 环境，依次输入

`cd ~/swarm_simulator_ws/`

`catkin_make`

等待编译完成。

## 基本用法
## 1.WheelswarmSimulator
订阅无人车的状态消息，话题名称（如1号车）为"/robot_1/pose"，消息类型为Odometry，msg.pose.pose为无人车的位姿信息；

发布无人车的控制消息，话题名称（如1号车）为"/robot_1/cmd_vel"，消息类型为Twist，msg.linear.x为无人车的控制线速度，msg.angular.z为无人车的控制角速度

示例1

<img src="./imgs/wheelswarm_demo01.gif">

<center>图1 无人车集群互换位置避碰示例</center>

如图1所示，在半径为1m和2m的圆内分别随机放置10辆无人车，外环的10辆无人车彼此交换位置，内环的10辆无人车彼此交换位置，20辆无人车在交换位置过程中要考虑避碰情况。

**终端1（启动WheelswarmSimulator）：**

`source ~/mambaforge-pypy3/bin/activate/ros_env_py310`

`source ~/swarm_simulator_ws/devel/setup.bash`

`roslaunch wheelswarm_ros start_wheelswarm.launch`

**终端2（启动无人车集群控制示例程序）：**

`source ~/mambaforge-pypy3/bin/activate/ros_env_py310`

`source ~/swarm_simulator_ws/devel/setup.bash`

`roslaunch swarm_brain start_wheelswarm_swarm_brain.launch`

## 2.CrazyswarmSimulator
**高级飞行控制指令**
**起飞指令**

TakeoffRequest(id, groupMask, height, duration)，其中，id为无人机编号，groupMask为集群编号，缺省为0，height为无人机起飞高度(m)，duration为持续时间(s)；

**降落指令**

LandRequest(id, groupMask, height, duration)，其中，id为无人机编号，groupMask为集群编号，缺省为0，height为无人机降落高度(m)，duration为持续时间(s)；

**GoTo指令**

GoToRequest(id, groupMask, relative, goal, yaw, duration)，其中，id为无人机编号，groupMask为集群编号，缺省为0，relative是否为相对位置，缺省为False, goal为目标点，yaw为偏航角(rad)，duration为持续时间(s)；

**上传轨迹指令**

UploadTrajectoryRequest(id, trajectoryId, pieceOffset, pieces)，id为无人机编号，trajectoryId为上传轨迹id，pieceOffset为轨迹片段偏移量，pieces为轨迹片段；

**沿轨迹飞行指令**

StartTrajectoryRequest(id, groupMask, trajectoryId, timescale, reversed, relative)，id为无人机编号，groupMask为集群编号，缺省为0，timescale为时间缩放比例，缺省为1.0，reversed是否按照相反的轨迹顺序飞行，缺省为False，relative是否为相对位置，缺省为True；

**低级飞行控制指令**

**cmd_vel**

话题名称（如1号机）为"/cf1/cmd_vel"，消息类型为Twist，msg.twist.linear为无人机的线速度信息，msg.twist.angular为无人机的角速度信息；

**cmd_velocity_world**

话题名称（如1号机）为"/cf1/cmd_velocity_world"，消息类型为VelocityWorld，msg.vel为无人车的控制速度矢量（x,y,z），msg.yawRate为无人机的偏航角速度，缺省为0；

**cmd_postion**

话题名称（如1号机）为"/cf1/cmd_position"，消息类型为Position，msg.x、msg.y、msg.z为无人机的位置，msg.yaw为无人机的偏航角，缺省为0；

**cmd_full_state**

话题名称（如1号机）为"/cf1/cmd_full_state"，消息类型为FullState，msg.pose.position为无人机的位置，msg.pose.orientation为无人机的控制姿态（四元数），msg.twist.linear为无人机的控制线速度，msg.acc为无人机的控制加速度，msg.twist.angular为无人机的控制角速度。

### 示例1

如图2所示，在终端输入take_off、land、upload、fly、goto，分别实现起飞、降落、上传轨迹、按飞机飞行和GoTo等无人机高级控制指令。

<img src="./imgs/crazyswarm_demo01.gif">

<center>图2 无人机集群高级飞行控制示例</center>

**终端1（启动CrazyswarmSimulator）：**

`source ~/mambaforge-pypy3/bin/activate/ros_env_py310`

`source ~/swarm_simulator_ws/devel/setup.bash`

`roslaunch crazyswarm_ros start_crazyswarm_sim.launch`

**终端2（启动Pybullet视景窗口）：**

`source ~/mambaforge-pypy3/bin/activate/ros_env_py310`

`source ~/swarm_simulator_ws/devel/setup.bash`

`roslaunch swarm_view start_pybullet4crazyswarm.launch`

**终端3（启动无人机集群高级飞行控制示例程序）：**

`source ~/mambaforge-pypy3/bin/activate/ros_env_py310`

`source ~/swarm_simulator_ws/devel/setup.bash`

`roslaunch swarm_brain start_crazyswarm_crazyflie_gcs.launch`

### 示例2

如图3所示，10架无人机起飞后，绕着原点转圈飞行。

<img src="./imgs/crazyswarm_demo02.gif">

<center>图3 无人机集群转圈飞行示例</center>

**终端1（启动CrazyswarmSimulator）：**

`source ~/mambaforge-pypy3/bin/activate/ros_env_py310`

`source ~/swarm_simulator_ws/devel/setup.bash`

`roslaunch crazyswarm_ros start_crazyswarm_sim.launch`

**终端2（启动Pybullet视景窗口）：**

`source ~/mambaforge-pypy3/bin/activate/ros_env_py310`

`source ~/swarm_simulator_ws/devel/setup.bash`

`roslaunch swarm_view start_pybullet4crazyswarm.launch`

**终端3（启动无人机集群转圈飞行控制示例程序）：**

`source ~/mambaforge-pypy3/bin/activate/ros_env_py310`

`source ~/swarm_simulator_ws/devel/setup.bash`

`roslaunch swarm_brain start_crazyswarm_go_circle.launch`

## `联系我们`

在 SwarmLab®，我们不仅提供先进的解决方案，还提供专业的技术支持和定制化服务，确保您能够充分发挥产品的潜力，实现研究目标。我们的解决方案涵盖协同控制、强化学习、智能博弈、大模型等领域，并为您的研究提供全方位的支持。 

选择 SwarmLab®的产品，意味着选择了高品质、高性能和高效率。让我们携手合作，共同探索蜂群技术的未来，助力您的研究取得更大的成功！欢迎联系我们，了解更多关于 SwarmLab®信息，让我们一起开启创新之旅！ 

​        欢迎访问我们： 

​		https://swarmlab.net/  

​		https://swarmlab.cn/  

​		https://exbot.net/ 

​		微信视频号：SwarmLab

​		联系方式：15821292799 田先生

#### _Enjoy!_ :+1:

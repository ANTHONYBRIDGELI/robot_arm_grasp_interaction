## 基本设置
这是一个基于yolo图像识别，使用moveit!对机械臂进行控制，实现特征物体吸取搬移的仿真实验。<br />
实现基于ROS，在Gazebo仿真环境中进行实验。

## 项目运行环境
ubuntu20.04 \
ros-noetic \
不建议虚拟机运行，虚拟机可能带来较长的图像识别推理时间，导致任务无法执行 

## 安装依赖
```
sudo apt install ros-noetic-moveit ros-noetic-joint-trajectory-controller ros-noetic-moveit-core ros-noetic-moveit-ros-planning ros-noetic-moveit-ros-planning-interface
```
## 部署程序
打开终端，依次输入以下指令：
```
git clone https://github.com/ANTHONYBRIDGELI/robot_arm_grasp_interaction.git
cd robot_arm_grasp_interaction
pip install -r Resources/requirements.txt
catkin_make
```
检查是否存在报错，如果有报错请解决后重新运行catkin_make指令，直到无报错
然后执行：
```
chmod +x ./excute/controller ./excute/get_input ./excute/grasp_obj run.sh
```
打开显示隐藏文件夹的开关：

<img src="https://gitee.com/Anthony_Bridge/assets/raw/master/robot_arm_grasp_interaction/fig/fig1.png"  width="800" />

<br/>
进入将下载的程序的Resources文件夹中的models文件夹复制到主文件下的.gazebo文件夹
<img src="https://gitee.com/Anthony_Bridge/assets/raw/master/robot_arm_grasp_interaction/fig/fig2.png"  width="800" />

<br/>
<img src="https://gitee.com/Anthony_Bridge/assets/raw/master/robot_arm_grasp_interaction/fig/fig3.png"  width="800" />

在刚刚打开的终端中，执行：
```
./run.sh
```
启动项目
<img src="https://gitee.com/Anthony_Bridge/assets/raw/master/robot_arm_grasp_interaction/fig/fig4.png"  width="800" />
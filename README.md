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

## 获取注释源码
项目提供了带有十分完善的注释的源程序 \
controller.cpp(对应可执行文件controller)中机械臂核心控制类部分代码实例：
```
// 机械臂控制的class
class ArmController
{
public:
    // 初始化
    ArmController(ros::Publisher grasp_obj_pub_, ros::Publisher set_model_state_pub_, ros::NodeHandle nh_)
    {
        
        ros::AsyncSpinner spinner(1);
        spinner.start();

        // 接收部分参数
        set_model_state_pub = set_model_state_pub_; //设置Gazebo中模型状态的话题发布
        grasp_obj_pub = grasp_obj_pub_; //需要抓取的模型名称的话题发布
        nh = nh_; //ros NodeHandle

        // 进行一些初始化操作
        // 初始化number_init_loc，number_init_loc中储存了number0-number9共10个模型的初始位姿，每个位姿包括6个变量，分别为x,y,z,roll,pitch,yaw
        for(int i = 0;; i++)
        {
            number_init_loc.push_back({0, 0, 0, 0, 0, 0});
            // 从src/ar3_control/config/numberi_init.yaml文件中加载坐标
            if(nh.getParam("number" + to_string(i), number_init_loc[i]))
            {
                numberBeMovedTimes.push_back(0);
            }
            // 从number10开始，找不到这个变量了，退出循环
            else
            {
                break;
            }
        }
        // 初始化numberReleaseLoc，numberReleaseLoc中储存1-6号放置位置的x,y轴坐标，1-6号位置对应release_loc0-release_loc5
        for(int i = 0;; i++)
        {
            numberReleaseLoc.push_back({0, 0});
            if(nh.getParam("release_loc" + to_string(i), numberReleaseLoc[i]))
            {
                numberReleaseOcc.push_back(false);
            }
            else
            {
                break;
            }
        }
        // 模型位置初始化
        modelAllInit();
        // 控制机械臂回到"home"位姿
        pose_home();
    }

    // 控制机械臂回到初始化位置
    void pose_home()
    {
        // "home"位姿是在moveit中预设的一个位姿
        arm.setNamedTarget("home");
        arm.move(); //规划+运动
        ros::Duration(1).sleep();
    }

    // 搜索并抓取目标
    bool searchAndGrabTarget(string name)
    {
        // 将返回给交互窗口的数据设置为"Search"，标志进入搜索阶段
        interaction_back_inform.data = "Search";
        // 输出搜索目标
        ROS_INFO("搜索目标: %s", name.c_str());
        // 声明两个变量，和机械臂搜索有关
        int step = 1;
        int num;
        // 搜索开始后，机械臂会在圆弧的一定区间内往复运动一次，如果搜索到目标则停止搜索动作，进入抓取状态
        // 如果往复运动一次后没有进入抓取状态，则说明没有找到目标

        // 声明一个机械臂末端执行器在世界坐标系下的位姿变量
        geometry_msgs::Pose target_pose;
        // 声明一个bool量，标志是否完成目标抓取
        bool done_grip_target = false;
        // 生命一个bool量，标记是否需要开始检测目标
        bool start_check_target = false;
        // 左右摆动一次搜索目标
        for(num = 10; num >= 10; num += step)
        {
            // 设置末端执行器位姿为搜索路径的起点
            target_pose = circle_pose(num, 15, 0.4, 0.4);
            arm.setPoseTarget(target_pose);
            // 调用逆运动学求解器计算关节角度
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
            {
                // ROS_INFO("Planning Successful!");
                // 执行规划的轨迹
                arm.execute(my_plan);
                // 从home进入搜索路径时，先暂停1s，再允许目标检测
                // 这是因为在机械臂从home到路径第一个点的过程中，可能出现目标识别误检测的情况，因此暂停1s规避
                if(!start_check_target)
                {
                    ros::Duration(1).sleep();
                    start_check_target = true;
                }
            }
            else
            {
                // 如果当前路径点的规划失败，则直接规划下一个点，一般情况下不会这样
                num += step;
            }
            // 完成第一次规划后才开始检查目标，也可以说是进入路径后才开始目标检测
            if(start_check_target)
            {
                // 判断是否在当前摄像头图像中找到了需要寻找的目标
                bool find_target = check_target(name);
                if(find_target)
                {
                    ROS_INFO("找到目标: %s", name.c_str());
                    // 如果找到了目标，则开始对正，因为目标出现在摄像头图像中的时候，一般是在图像的边角位置，对正是指将机械臂末端对准目标，即将目标在摄像头图像的中心
                    // 进入对正和抓取阶段
                    bool aim_target_success = aim_target(name, target_pose);
                    // 返回true值，说明完成抓取
                    if(aim_target_success)
                    {
                        return true;
                    }
                    // 否则搜索失败，返回false
                    else
                    {
                        return false;
                    }
                }
            }
            // num可以在10-19内变化，如果num增加到了19，则将step修改为-1，num开始减少
            if(num == 19)
            {
                step = -1;
            }
            // ros::Duration(2).sleep();
        }
        // num从10增加到19又减少到9，说明完成一次往复搜索，退出循环，没有搜素到目标
        ROS_INFO("没有搜索到目标");
        // 标记任务失败
        interaction_back_inform.data = "Failed";
        // 返回false
        return false;
    }

    // 模型全部初始化
    void modelAllInit()
    {
        // 将标记模型被移动变量的vector中全部置0，认为模型没有被抓取过
        for(int i = 0; i < numberBeMovedTimes.size(); i++)
        {
            numberBeMovedTimes[i] = 0;
        }
        // 将放置位置的被占据状态也设置为0，认为放置位置没有被占用过
        for(int i = 0; i< numberReleaseOcc.size(); i++)
        {
            numberReleaseOcc[i] = false;
        }

        // 一次设置number0-number9的位置
        // 为了保证在多次连续抓取中能一直连续抓取同一个数字，比如最多可以抓取6次number0
        // 仿真场景中实际上已经添加了6套number0-number9的模型，只不过一开始只有一套模型在机械臂可以抓取到的范围里面
        // 其余模型都在场景中其他地方，当一个模型被抓取挪走后，直接从场景中将这个模型的另外一套对应的模型挪过来
        // 模型的命名规则可以在gazebo中看到，
        // 例如number0，一共有6个模型，Gazebo中的名称分别是number0_01, number0_01_clone, number0_01_clone_0, number0_01_clone_1, number0_01_clone_2, number0_01_clone_3
        for(int i = 0; i < 10; i++)
        {   
            // xy标记了放置的起点为(8,8)位置
            float x = 8;
            float y = 8;
            // extern_str中储存了number0字符以外的额外的字符，拼接后为完整的模型名称
            for(int q = 0; q < extern_str.size(); q++)
            {
                // 声明一个gazebo模型状态消息
                gazebo_msgs::ModelState object_state;
                // 拼接出完整的名称
                object_state.model_name = "number" + to_string(i) + extern_str[q];
                // 如果是number0_01，说明是应该出现在机械臂可以抓取范围内的第一套模型，则放置在模型的初始位置
                if(extern_str[q] == "_01")
                {
                    object_state.pose.position.x = number_init_loc[i][0];
                    object_state.pose.position.y = number_init_loc[i][1];
                }
                else
                {
                    // 其他套模型挪到场景的其他地方
                    object_state.pose.position.x = x + 0.2 * i;
                    object_state.pose.position.y = y + 0.2 * q;
                }
                object_state.pose.position.z = number_init_loc[i][2];
                // 获取欧拉角
                EulerAngles eular;
                eular.roll = number_init_loc[i][3];
                eular.pitch = number_init_loc[i][4];
                eular.yaw = number_init_loc[i][5];
                // 将欧拉角转换为四元数，并设置
                Quaternion quaternion = ToQuaternion(eular);
                object_state.pose.orientation.w = quaternion.w;
                object_state.pose.orientation.x = quaternion.x;
                object_state.pose.orientation.y = quaternion.y;
                object_state.pose.orientation.z = quaternion.z;
                // 发送5次，确保Gazebo接收到信息并完成模型设置
                for(int i = 0; i < 5; i++)
                {
                    set_model_state_pub.publish(object_state);
                    ros::Duration(0.01).sleep();
                }
            }
        }
    }
private:
    // 设置模型位置的发布话题
    ros::Publisher set_model_state_pub;
    // ros NodeHandle
    ros::NodeHandle nh;
    // 机械臂附近的模型的初始位置
    vector<vector<float>> number_init_loc;
    // 数字模型被移动的次数记录，涉及模型的具体名称判断
    vector<int> numberBeMovedTimes;
    // 模型被释放的位置，答案区域最多可以防止3个模型，储存了这个3个位置的坐标信息
    vector<vector<float>> numberReleaseLoc;
    // 模型释放位置的占用情况，记录了答案区域3个位置是否被占据，用于判断模型应该防止在哪个位置
    vector<bool> numberReleaseOcc;
    // 发布需要抓取的模型名字，发布给抓取功能包，以实现抓取效果仿真
    ros::Publisher grasp_obj_pub;
    // 创建MoveGroupInterface对象
    moveit::planning_interface::MoveGroupInterface arm = moveit_arm_init(); 
    // 储存还原模型完整名称需要额外拼接的字符串
    vector<string> extern_str = {"_01_clone_3", "_01_clone_2", "_01_clone_1", "_01_clone_0", "_01_clone", "_01"};

    // 对正目标的子函数
    bool aim_target(string name, geometry_msgs::Pose target_pose)
    {
        // 将反馈给交互窗口的信息设置为"Aim"，标志进入对正阶段
        interaction_back_inform.data = "Aim";
        // 设置对正允许的像素误差，这里是30个像素
        int diff_Tolerance = 30;
        // 进入循环
        while(ros::ok())
        {
            // 声明一个目标索引，对应目标在yolo_result.classes中的位置，初始化为-1，如果没有找到目标，则会保持-1的值
            int target_index = -1;
            // 遍历yolo_result.classes寻找name
            for(int i = 0; i < yolo_result.classes.size(); i++)
            {
                // 如果找到，则target_index为对应的索引，并结束for循环
                if(yolo_result.classes[i] == name)
                {
                    target_index = i;
                    break;
                }
            }
            // 如果target_index的值不为-1，说明在当前yolo识别结果中找到了需要抓取的目标，则尝试对正
            if(target_index != -1)
            {
                // 项目中相机的分辨率为1280*720，因此图像中心的像素坐标为(640,360)
                // 注意为图像坐标系，和一般的直角坐标系不同
                // 计算x和y轴的当前像素差距
                int diff_x = yolo_result.center[target_index][0] - 640;
                int diff_y = yolo_result.center[target_index][1] - 360;
                // 如果两个轴的像素差距都小于允许的误差，则认为已经对正目标
                if(abs(diff_x) < diff_Tolerance && abs(diff_y) < diff_Tolerance)
                {
                    // 进入抓取阶段
                    bool grabSuccess = grabTarget(name, target_pose);
                    return true;
                }
                // 如果没有对正，则继续挪动机械臂末端，直到对正
                else
                {
                    // 根据像素差距设置机械臂末端的位姿，注意图像坐标系和Gazebo世界坐标系的方向转换
                    target_pose.position.x += (-1) * diff_x * 0.0002;
                    target_pose.position.y += diff_y * 0.0002;
                    // 为了避免特殊情况（虽然一般不会出现），避免目标点设置到了机械臂不可到达的位置
                    // 计算当前目标点距离原点的平面距离，因为机械臂放置在坐标原点，所以这个值也是机械臂需要伸长的距离
                    float R = pow(pow(target_pose.position.x, 2) + pow(target_pose.position.y, 2), 0.5);
                    // 最远允许伸长0.5m，如果超过，则只伸长0.5m
                    if(R > 0.5)
                    {
                        target_pose.position.x *= (0.5 / R);
                        target_pose.position.y *= (0.5 / R);
                    }
                    // 逆运动学阶段并驱动
                    arm.setPoseTarget(target_pose);
                    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                    bool success = (arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    if (success)
                    {
                        // 执行规划的轨迹
                        arm.execute(my_plan);
                    }
                }
            }
            // 如果target_index的值为-1，说明当前没有找到目标，但是能够进入对正阶段，说明在当前搜索周期内肯定是看到过目标的，只是现在丢失了
            // 因此做丢失处理
            else
            {
                bool find_target_again = lost_target(name);
                // 如果丢失处理后依然找不到目标，则只能宣告搜索失败
                if(!find_target_again)
                {
                    return false;
                }
            }
        }
        return false;
    }
    // 目标丢失后的处理
    bool lost_target(string name)
    {
        ROS_INFO("目标丢失，等待5s");
        // 等待5s的时间，等待yolo识别图像，如果依然找不到目标，则目标丢失
        float lost_target_time = ros::Time::now().toSec();
        ros::Rate rate(30);
        // 循环等待
        while (ros::ok())
        {
            // 检查yolo识别结果中是否有需要抓取的模型
            bool find_target = check_target(name);
            // 如果找到目标，则继续对正
            if(find_target)
            {
                ROS_INFO("重新找到目标: %s", name.c_str());
                return true;
            }
            // 超过5s，退出
            if(ros::Time::now().toSec() - lost_target_time > 5)
            {
                return false;
            }
            rate.sleep();
        }
        return false;
    }
};
```

项目开发十分不易，为了项目的可持续发展以及为用户提供更加完善可靠的服务，获取源码需要收取一点微薄的费用，您的支持是我们前进的动力，我们十分感谢您提供的帮助。 \
获取源码请关注微信公众号"开发者工坊":

<img src="https://gitee.com/Anthony_Bridge/assets/raw/master/common/%E7%99%BD%E5%BA%95%E7%BB%BF%E5%AD%97%E4%BA%8C%E7%BB%B4%E7%A0%81%E5%92%8C%E5%90%8D%E7%A7%B0.png"  width="500" />

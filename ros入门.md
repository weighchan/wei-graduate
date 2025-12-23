[本项目学习参考地址](https://github.com/guyuehome/ros_21_tutorials)

常用命令： 

rostopic：话题相关

rosservice ：服务器相关（动态的）

rosparam：管理参数服务器

rosmsg：消息相关

rossrv：查询和管理.srv服务类型定义（静态的）

rosbag：记录数据

rqt：可视化工具                    rqt_graph: 节点间的可视化

rosnode: 查看系统节点（节点不能重名）

[ROS 一键安装程序开源地址](https://github.com/fishros/install)

```shell
$ wget http://fishros.com/install -O fishros && . fishros
```



### 1. 工作空间与功能包

**工作空间与相关内容**

------

workspace：存放工程文件的文件夹结构

|——build：编译过程中产生的中间文件

|——src：代码存放文件

|——devel：编译生成的目标文件

workspace创建：

```shell
$ mkdir -p ./catkin_ws/src
$ cd ./catkin_ws/src
$ catkin_init_workspace
```

编译与设置工作空间：

```shell
$ cd ..
$ catkin_make:编译工作空间（局部编译：catkin_make --cmake-args -DCATKIN_WHITELIST_PACKAGES="your_package_name"）
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH：检查环境变量
```

**功能包**

```shell
$ cd ~/catkin_ws/src
$ catkin_create_pkg yourname std_msgs rospy roscpp ...
$ ... 代表着相关的接口服务，想要使用必须在创建时就引用好，否则后期会很麻烦，如本包要使用海龟，需引入turtlesim
```

### 2. topic创建与订阅

**创建功能包**

```shell
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_service roscpp rospy std_msgs geometry_msgs turtlesim
```

**CMakeLists.txt**

```txt
添加在build之后
add_executable(person_publisher src/person_publisher.cpp) 
target_link_libraries(person_publisher ${catkin_LIBRARIES}) 
add_dependencies(person_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(person_subscriber src/person_subscriber.cpp) 
target_link_libraries(person_subscriber ${catkin_LIBRARIES}) 
add_dependencies(person_subscriber ${PROJECT_NAME}_generate_messages_cpp)
```

#### **2.1 Publisher编程实现**

如何实现一个发布者:

1. 初始化ROS节点; 
2. 向ROS Master注册节点信息，
3. 包括发布的话题名和话题中的消息类型; 
4. 创建消息数据；
5. 按照一定频率循环发布消息。

```python
import rospy
from learning_topic.msg import Person

def velocity_publisher():
	# ROS节点初始化
    rospy.init_node('person_publisher', anonymous=True)

	# 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    person_info_pub = rospy.Publisher('/person_info', Person, queue_size=10)

	#设置循环的频率
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
		# 初始化learning_topic::Person类型的消息
    	person_msg = Person()
    	person_msg.name = "Tom";
    	person_msg.age  = 18;
    	person_msg.sex  = Person.male;

		# 发布消息
        person_info_pub.publish(person_msg)
    	rospy.loginfo("Publsh person message[%s, %d, %d]", 
				person_msg.name, person_msg.age, person_msg.sex)

		# 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
```

```c++
#include <ros/ros.h>
#include "learning_topic/Person.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "person_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
    ros::Publisher person_info_pub = n.advertise<learning_topic::Person>("/person_info", 10);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        // 初始化learning_topic::Person类型的消息
    	learning_topic::Person person_msg;
		person_msg.name = "Tom";
		person_msg.age  = 18;
		person_msg.sex  = learning_topic::Person::male;

        // 发布消息
		person_info_pub.publish(person_msg);

       	ROS_INFO("Publish Person Info: name:%s  age:%d  sex:%d", 
				  person_msg.name.c_str(), person_msg.age, person_msg.sex);

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
```

#### **2.2 Subscriber编程实现**

如何实现一个订阅者:

1. 初始化ROS节点; 
2. 订阅需要订阅的话题；
3. 循环等待话题消息，接受到消息后进入回调函数; 
4. 在回调函数中完成消息的处理；

```python
import rospy
from learning_topic.msg import Person

def personInfoCallback(msg):
    rospy.loginfo("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg.name, msg.age, msg.sex)

def person_subscriber():
	# ROS节点初始化
    rospy.init_node('person_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/person_info", Person, personInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()

```

```c++
#include <ros/ros.h>
#include "learning_topic/Person.h"

// 接收到订阅的消息后，会进入消息回调函数
void personInfoCallback(const learning_topic::Person::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Subcribe Person Info: name:%s  age:%d  sex:%d", 
			 msg->name.c_str(), msg->age, msg->sex);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "person_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    ros::Subscriber person_info_sub = n.subscribe("/person_info", 10, personInfoCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}
```

#### **2.3 msg**

touch yourname.msg / yourname.txt ...

如何自定义消息文档:

1.  定义msg文件（touch yourname.msg）；

   ```msg
   string name
   uint8  age
   uint8  sex
   
   uint8 unknown = 0
   uint8 male    = 1
   uint8 female  = 2
   ```

2. 在package.xml中添加包依赖;

   ```txt
   <build_depend>message_generation</build_depend>
   <exec_depend>message_runtime</exec_depend>
   ```

3. 在CMakeLists.txt添加编译选项;

   ```txt
   find_package(...... message_generation) 
   add_message_files(FILES Person.msg) generate_messages(DEPENDENCIES std_msgs) 
   catkin_package(...... message_runtime)
   ```

4. 编译生成语言相关文件。

### 3. Service 和 Client

 **创建功能包**

```shell
$ cd~/catkin_ws/src
$ catkin_create_pkg learning_service roscpp rospy std_ msgs geometry_msgs turtlesim
```

**CMakeLists.txt**

```txt
add_executable(person_server src/person_server.cpp)
target_link_libraries(person_server ${catkin_LIBRARIES})
add_dependencies(person_server ${PROJECT_NAME}_gencpp) 

add_executable(person_client src/person_ client.cpp)
target_link_libraries(person_client ${catkin_LIBRARIES})
add_dependencies(person_client ${PROJECT_NAME}_gencpp)
```

#### **3.1 如何实现一个Client**

1. 初始化ROS节点；
2. 创建一个Client实例；
3. 发布服务请求数据；
4. 等待Server处理之后的结果。

```c++
#include <ros/ros.h>
#include "learning_service/Person.h"

int main(int argc, char** argv)
{
    // 初始化ROS节点
	ros::init(argc, argv, "person_client");

    // 创建节点句柄
	ros::NodeHandle node;

    // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
	ros::service::waitForService("/show_person");
	ros::ServiceClient person_client = node.serviceClient<learning_service::Person>("/show_person");

    // 初始化learning_service::Person的请求数据
	learning_service::Person srv;
	srv.request.name = "Tom";
	srv.request.age  = 20;
	srv.request.sex  = learning_service::Person::Request::male;

    // 请求服务调用
	ROS_INFO("Call service to show person[name:%s, age:%d, sex:%d]", 
			 srv.request.name.c_str(), srv.request.age, srv.request.sex);

	person_client.call(srv);

	// 显示服务调用结果
	ROS_INFO("Show person result : %s", srv.response.result.c_str());

	return 0;
};
```

```python
import sys
import rospy
from learning_service.srv import Person, PersonRequest

def person_client():
	# ROS节点初始化
    rospy.init_node('person_client')

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/show_person')
    try:
        person_client = rospy.ServiceProxy('/show_person', Person)

		# 请求服务调用，输入请求数据
        response = person_client("Tom", 20, PersonRequest.male)
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	#服务调用并显示调用结果
    print "Show person result : %s" %(person_client())
```

#### **3.2 如何实现一个Service**

1. 初始化ROS节点；
2. 创建一个Client实例；
3. 发布服务请求数据；
4. 等待Server处理之后的结果。

```c++
#include <ros/ros.h>
#include "learning_service/Person.h"

// service回调函数，输入参数req，输出参数res
bool personCallback(learning_service::Person::Request  &req,
         			learning_service::Person::Response &res)
{
    // 显示请求数据
    ROS_INFO("Person: name:%s  age:%d  sex:%d", req.name.c_str(), req.age, req.sex);

	// 设置反馈数据
	res.result = "OK";

    return true;
}

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "person_server");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个名为/show_person的server，注册回调函数personCallback
    ros::ServiceServer person_service = n.advertiseService("/show_person", personCallback);

    // 循环等待回调函数
    ROS_INFO("Ready to show person informtion.");
    ros::spin();

    return 0;
}
```

```python
import rospy
from learning_service.srv import Person, PersonResponse

def personCallback(req):
	# 显示请求数据
    rospy.loginfo("Person: name:%s  age:%d  sex:%d", req.name, req.age, req.sex)

	# 反馈数据
    return PersonResponse("OK")

def person_server():
	# ROS节点初始化
    rospy.init_node('person_server')

	# 创建一个名为/show_person的server，注册回调函数personCallback
    s = rospy.Service('/show_person', Person, personCallback)

	# 循环等待回调函数
    print "Ready to show person informtion."
    rospy.spin()

if __name__ == "__main__":
    person_server()
```

#### **3.3 srv**

1. 定义srv文件；

   ```txt
   string name
   uint8 age
   uint8 sex 
   
   uint8 unknown=0
   uint8 male =1
   uint8 female =2
   ---
   string result
   ```

2. 在package.xml中添加包依赖;

   ```txt
   <build_ depend>message_generation</build_depend>
   <exec_depend>message_runtime</exec_depend>
   ```

3. 在CMakeLists.txt添加编译选项

   ```txt
   find_package(...... message_generation) 
   add_service_files(FILES Person.srv)
   generate_messages(DEPENDENCIES std_msgs) 
   catkin_package(...... message_runtime)
   ```

4. 编译生成语言相关文件。

### 4. PARAM

参数的使用与编程方法——[学习参考链接](http://wiki.ros.org/Parameter%20Serve)

**创建功能包**

```shell
$ cd~/catkin_ws/src 
$ catkin_create_pkg learning_parameter roscpp rospy std_srvs
```

**参数命令使用（rosparam）**

```shell
$ 列出当前多有参数
$ rosparam list 
$ 显示某个参数值
$ rosparam get param_key 
$ 设置某个参数值
$ rosparam set param_key param_value 
$ 保存参数到文件
$ rosparam dump file_name 
$ 从文件读取参数
$ rosparam load file_name 
$ 删除参数
$ rosparam delete param_key
```

#### **4.1 如何获取/设置参数**

1. 初始化ROS节点；
2. get函数获取参数；
3. set函数设置参数。

```c++
#include <string>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char **argv)
{
	int red, green, blue;

    // ROS节点初始化
    ros::init(argc, argv, "parameter_config");

    // 创建节点句柄
    ros::NodeHandle node;

    // 读取背景颜色参数
	ros::param::get("/background_r", red);
	ros::param::get("/background_g", green);
	ros::param::get("/background_b", blue);

	ROS_INFO("Get Backgroud Color[%d, %d, %d]", red, green, blue);

	// 设置背景颜色参数
	ros::param::set("/background_r", 255);
	ros::param::set("/background_g", 255);
	ros::param::set("/background_b", 255);

	ROS_INFO("Set Backgroud Color[255, 255, 255]");

    // 读取背景颜色参数
	ros::param::get("/background_r", red);
	ros::param::get("/background_g", green);
	ros::param::get("/background_b", blue);

	ROS_INFO("Re-get Backgroud Color[%d, %d, %d]", red, green, blue);

	// 调用服务，刷新背景颜色
	ros::service::waitForService("/clear");
	ros::ServiceClient clear_background = node.serviceClient<std_srvs::Empty>("/clear");
	std_srvs::Empty srv;
	clear_background.call(srv);
	
	sleep(1);

    return 0;
}
```

```python
import sys
import rospy
from std_srvs.srv import Empty

def parameter_config():
	# ROS节点初始化
    rospy.init_node('parameter_config', anonymous=True)

	# 读取背景颜色参数
    red   = rospy.get_param('/background_r')
    green = rospy.get_param('/background_g')
    blue  = rospy.get_param('/background_b')

    rospy.loginfo("Get Backgroud Color[%d, %d, %d]", red, green, blue)

	# 设置背景颜色参数
    rospy.set_param("/background_r", 255);
    rospy.set_param("/background_g", 255);
    rospy.set_param("/background_b", 255);

    rospy.loginfo("Set Backgroud Color[255, 255, 255]");

	# 读取背景颜色参数
    red   = rospy.get_param('/background_r')
    green = rospy.get_param('/background_g')
    blue  = rospy.get_param('/background_b')

    rospy.loginfo("Get Backgroud Color[%d, %d, %d]", red, green, blue)

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/clear')
    try:
        clear_background = rospy.ServiceProxy('/clear', Empty)

		# 请求服务调用，输入请求数据
        response = clear_background()
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    parameter_config()
```

**配置CMakeLists.txt**

```txt
add executable(parameter_config src/parameter_config.cpp) target_link_libraries(parameter_config $tcatkin_LIBRARIES) 
```

### 5. TF

TF（坐标系管理系统）——[《机器人学导论》](https://z-library.co/book/13704231)

**机器人中的坐标变换**

```shell
$ sudo apt-get install ros-yourname-turtle-tf
$ roslaunch turtle_tf turtle_tf_demo.launch
$ rosrun turtlesim turtle_teleop_key
$ rosrun tf view_frames

$ tf tf_echo turtle1 turtle2
$ rosrun rviz rviz -d`rospack find turtle_tf` /rviz/turtle_rviz.rviz
```

#### **5.1 tf坐标系广播与监听的编程实现**

创建功能包

```shell
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_tf roscpp rospy tf turtlesim
```

##### 5.1.1 如何实现一个tf广播器：

1. 定义TF广播器（TransformBroadcaster）；
2. 创建坐标变换值；
3. 发布坐标变换（sendTransform）。

```c++
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
	// 创建tf的广播器
	static tf::TransformBroadcaster br;

	// 初始化tf数据
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, msg->theta);
	transform.setRotation(q);

	// 广播world与海龟坐标系之间的tf数据
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
	ros::init(argc, argv, "my_tf_broadcaster");

	// 输入参数作为海龟的名字
	if (argc != 2)
	{
		ROS_ERROR("need turtle name as argument"); 
		return -1;
	}

	turtle_name = argv[1];

	// 订阅海龟的位姿话题
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

    // 循环等待回调函数
	ros::spin();

	return 0;
};
```

```python
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()
```

##### 5.1.2 如何实现一个TF监听器：

1. 定义TF监听器（TransformListenner）；
2. 查找坐标变换（waitForTransform，lookupTransform）。

```c++
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
	// 初始化ROS节点
	ros::init(argc, argv, "my_tf_listener");

    // 创建节点句柄
	ros::NodeHandle node;

	// 请求产生turtle2
	ros::service::waitForService("/spawn");
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv;
	add_turtle.call(srv);

	// 创建发布turtle2速度控制指令的发布者
	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

	// 创建tf的监听器
	tf::TransformListener listener;

	ros::Rate rate(10.0);
	while (node.ok())
	{
		// 获取turtle1与turtle2坐标系之间的tf数据
		tf::StampedTransform transform;
		try
		{
			listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(3.0));
			listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
		}
		catch (tf::TransformException &ex) 
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		// 根据turtle1与turtle2坐标系之间的位置关系，发布turtle2的速度控制指令
		geometry_msgs::Twist vel_msg;
		vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
				                        transform.getOrigin().x());
		vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
				                      pow(transform.getOrigin().y(), 2));
		turtle_vel.publish(vel_msg);

		rate.sleep();
	}
	return 0;
};
```

```python
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()
```

##### 5.1.3 配置CMakeLists.txt

```
add_executable(turtle_tf_ broadcaster src/turtle_tf_broadcaster.cpp) target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES}) 

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES}) 
```

**编译与运行**

```shell
$ cd~/catkin_wS 
$ catkin_make 
$ source devel/setup.bash 
$ roscore 
$ rosrun turtlesim turtlesim_node 
$ rosrun learning_ tf turtle_tf_broadcaster__name:=turtle1_tf_broadcaster /turtle1
$ rosrun learning_tf turtle_tf_broadcaster___name:=turtle2_tf_broadcaster /turtle2
$ rosrun learning_tf turtle_tf_listener 
$ rosrun turtlesim turtle_teleop_key
```

### 6. LAUNCH

Launch文件：通过XML文件实现多节点的配置和启动（**可自启动ros master**）。

```xml
<launch>
    <node pkg="turtlesim" name="sim1" type="turtlesim_ node"/>
    <node pkg="turtlesim" name="sim2" type="turtlesim node"/>
</launch>

<其中：>
<launch文件中的根元素采用<launch>标签定义>
<启动节点>
    <node pkg="package-name" type="'executable-name" name="node-name"/>
    <pkg:节点所在的功能包名称/>
    <type:节点的可执行文件名称/>
    <name:节点运行时的名称/>
    <output、respawn、required、ns、args/>
<标签示例>
    <param>/<rosparam>
        <设置ROS系统运行中的参数，存储在参数服务器中/>
        <param name="output_frame" value="odom"/><name:参数名 value:参数值/>
        <加载参数文件中的多个参数/>
        <rosparam file="params.yaml" command="load" ns= "params"/>
     <arg>
        <launch文件内部的局部变量，仅限于launch文件使用/>
        <arg name="arg-name" default="arg-value" /><name:参数名 value:参数值/>
         
        <调用/>
        <param name="foo" value="$(arg arg-name)" />
        <node name="node" pkg="package" type="type " args="$(arg arg-name)"/>
     <remap>
        <重映射ROS计算图资源的命名/>
        <remap from="/turtlebot/cmd_vel" to="/cmd_vel"/><from:原命名 to:映射之后的命名/>
     <include>
        <包含其他launch文件,类似C语言中的头文件包含/>
        <include file="$(dirname)/other.launch"/><file:包含的其他launch文件路径/>
</标签示例>
```

[更多标签参考](http://wiki.ros.org/roslaunch/XML)

### 7. URDF

pass



### 8. 可视化工具与虚拟仿真

**QT工具箱**

- 日志输出工具——rqt_console 
- 计算图可视化工具——rqt_graph 
- 数据绘图工具——rqt_plot 
- 图像渲染工具——rqt_image_view

**Rviz**

- Rviz是一款三维可视化工具，可以很好的兼容基于ROS软件框架的机器人平台。
- 在rviz中，可以使用可扩展标记语言XML对机器人、周围物体等任何实物进行尺寸、质量、位置、材质、关节等属性的描述,并且在界面中呈现出来。
- rviz还可以通过图形化的方式，实时显示机器人传感器的信息、机器人的运动状态、周围环境的变化等信息。
- 总而言之，rviz通过机器人模型参数、机器人发布的传感信息等数据，为用户进行所有可监测信息的图形化显示。用户和开发者也可以在rviz的控制界面下，通过按钮、滑动条、数值等方式，控制机器人的行为。

**Gazebo**

Gazebo是一款功能强大的三维物理仿真平台:

- 具备强大的物理引擎
- 高质量的图形渲染
- 方便的编程与图形接口
- 开源免费

其典型应用场景包括:

- 测试机器人算法
- 机器人的设计
- 现实情景下的回溯测试

### 9. 进阶学习路线

你可以用ROS干什么：

- [Gazabo+ROS+ros_control](http://wiki.ros.org/ros_control)
- [gmapping](http://wiki.ros.org/gmapping/)
- [hector](http://wiki.ros.org/hector_slam)

一些资源：[斯坦福大学公开课——机器人学](https://www.bilibili.com/video/av4506104/)，[Andrew Davison的机器人学讲座课程](http://www.doc.ic.ac.uk/~ajd/Robotics/index.html)，[ETH - Robotic Systems Lab](http://www.rsl.ethz.ch/education-students/lectures.html)，[ROS](https://www.ros.org)，[ROS Wiki](http://wiki.ros.org/)，[GitHub](https://github.com/)，[YouTube](https://www.youtube.com/)，[Bilibili](https://www.bilibili.com/)


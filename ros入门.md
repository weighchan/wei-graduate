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

wget http://fishros.com/install -O fishros && . fishros

### 1. 工作空间与功能包

**工作空间与相关内容**

------

workspace：存放工程文件的文件夹结构

|——build：编译过程中产生的中间文件

|——src：代码存放文件

|——devel：编译生成的目标文件

workspace创建：

```cmake
$ mkdir -p ./catkin_ws/src
$ cd ./catkin_ws/src
$ catkin_init_workspace
```

编译与设置工作空间：

```cmake
$ cd ..
$ catkin_make:编译工作空间（局部编译：catkin_make --cmake-args -DCATKIN_WHITELIST_PACKAGES="your_package_name"）
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH：检查环境变量
```

**功能包**

```cmake
$ cd ~/catkin_ws/src
$ catkin_create_pkg yourname std_msgs rospy roscpp ...
# ... 代表着相关的接口服务，想要使用必须在创建时就引用好，否则后期会很麻烦，如本包要使用海龟，需引入turtlesim
```

### 2. topic创建与订阅

**创建功能包**

```cmake
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

```cmake
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

### 4. PARAMETER

参数的使用与编程方法

**创建功能包**



### 5. TF





### 6. LAUNCH





### 7. URDF









### 8. 可视化工具与虚拟仿真







### 9. 进阶学习路线




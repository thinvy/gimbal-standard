# gimbal-standard
Gimbal standared controller of IRobot 2022 ECHG

### 工程结构

/application 后台程序和中断服务程序，其中Attitude为姿态解算线程，Calculate为云台控制线程，Debug为调试线程，Interrupt为中断服务

/algorithm 算法库

/bsp 板级外设支持包

/device 设备驱动（通信应用层接口）

/Config 工程配置与通信接口定义

其他均为CubeMX生成的代码结构，不建议更改，否则可能会造成CubeMX代码生成的不兼容



### 基础配置和调参

1、电机配置和接线

按照机器人制作规范完成云台全部电机的电源线连接，设置电机ID，建议（不强制）设置如下：

左摩擦轮0x201；右摩擦轮0x202，Yaw轴电机0x205；Pitch轴电机0x206；步兵哨兵拨盘0x203；英雄大拨盘0x207

以上配置可以满足通信总线的最高效率。连接CAN总线，这里要求除了YAW轴电机和英雄大拨盘其他的所有云台电机连接到同一条CAN总线上（CAN1、CAN2无所谓），上位机、YAW轴电机、英雄大拨盘、底盘、裁判系统通信板连到另一条CAN总线上（CAN1、CAN2无所谓）。



2、复制参数表

复制/Config/XXXParameter.h文件并重命名，比如7号哨兵可以命名为Sentry7Parameter.h，以下配置以一个哨兵云台为例。



3、编写设置文件

打开/Config/Setting.h，在文件开头添加一个自拟名称的宏定义作为条件编译的总条件，然后注释掉其他宏定义

![image-20220304143623348](C:\Users\qylann\AppData\Roaming\Typora\typora-user-images\image-20220304143623348.png)

编写详细的配置信息

![image-20220304143820072](C:\Users\qylann\AppData\Roaming\Typora\typora-user-images\image-20220304143820072.png)

其他都比较好理解，这里对 “IMU安装方向” 进行说明，该宏定义的含义是云台水平时陀螺仪坐标系xyz与云台姿态坐标系XYZ的对应关系，比如 “IMU_DIRECTION_rxryz_XYZ” 的含义是：云台的X轴的方向和imu水平放置时x轴的反方向相同（rx），云台Y轴方向和imu水平放置时y轴的反方向相同（ry），云台的Z轴的方向和imu水平放置时z轴的方向相同（z），即 “rxryz” 对应 “XYZ”。

一般这时就可以使用遥控器控制云台了



4、修改参数表的参数

精细调参，满足自瞄的控制和弹道要求。
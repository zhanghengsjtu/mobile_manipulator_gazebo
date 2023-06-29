#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>

#include <ros/console.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <control_msgs/JointControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <arm_controller_pub.h>
#include <gripper_controller_pub.h>
#include <wheel_controller_pub.h>
#include <KDL_IK.h>

int kfd = 0;
struct termios cooked, raw;

void init_keyboard()
{
  // 获取终端参数
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  // 设置非标准模式
  raw.c_lflag &= ~(ICANON | ECHO);
  // 设置最小字符数和等待时间
  raw.c_cc[VMIN] = 1;
  raw.c_cc[VTIME] = 0;
  // 设置终端参数
  tcsetattr(kfd, TCSANOW, &raw);
}

void restore_keyboard()
{
  // 恢复终端参数
  tcsetattr(kfd, TCSANOW, &cooked);
  tcflush(kfd, TCIFLUSH);
}

char getch()
{
  char c;
  fd_set fds;
  struct timeval tv;

  FD_ZERO(&fds);
  FD_SET(kfd, &fds);

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  // 设置文件描述符为非阻塞模式
  int flags = fcntl(kfd, F_GETFL, 0);
  fcntl(kfd, F_SETFL, flags | O_NONBLOCK);

  if (select(kfd + 1, &fds, NULL, NULL, &tv) == 1)
  {
    // 从终端读取一个字符
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
  }
  else
  {
    c = '\0';
  }

  // 恢复文件描述符的阻塞模式
  fcntl(kfd, F_SETFL, flags);

  return c;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_control");
  ros::NodeHandle keyborad;

  // 创建速度发布器
  ros::Publisher pub_robot1_v1= keyborad.advertise<std_msgs::Float64>("/robot1/front_left_controller/command", 30);
  ros::Publisher pub_robot1_v2= keyborad.advertise<std_msgs::Float64>("/robot1/front_right_controller/command", 30);
  ros::Publisher pub_robot1_v3= keyborad.advertise<std_msgs::Float64>("/robot1/rear_left_controller/command", 30);
  ros::Publisher pub_robot1_v4= keyborad.advertise<std_msgs::Float64>("/robot1/rear_right_controller/command", 30);

  // 初始化键盘
  init_keyboard();

  ros::AsyncSpinner spinner(10);
  spinner.start();
  ros::Duration(1).sleep(); //Duration is the simulation time, this is 2s in real world
  ros::Rate rate(50);

  bool key_pressed = false;   // 按键按下状态标志变量，初始为未按下状态
  bool key_released = true;   // 按键释放状态标志变量，初始为已释放状态
  std::vector<double> robot1_vel = {0.0, 0.0, 0.0}; // 速度向量，初始为零速度

  // 控制循环
  while (ros::ok())
  {
    // 读取键盘输入
    char c;
    c = getch();

    if (!key_pressed && key_released) // 只在按键未按下且已释放时执行
    {
      if (c != '\0') // 按键按下时
      {
        // 根据键盘输入设置速度命令
        switch (c)
        {
          case 'w':
            robot1_vel[0] = 0.1;  // 前
            break;
          case 's':
            robot1_vel[0] = -0.1;  // 后
            break;
          case 'a':
            robot1_vel[1] = 0.1;  // 左
            break;
          case 'd':
            robot1_vel[1] = -0.1;  // 右
            break;
          case 'j':
            robot1_vel[2] = 0.1;  // 顺时针
            break;
          case 'k':
            robot1_vel[2] = -0.1;  // 逆时针
            break;
          case 'q':
            // 退出程序
            restore_keyboard();
            return 0;
          default:
            break;
        }
        key_pressed = true; // 设置按键状态为已按下
        key_released = false; // 设置按键状态为未释放
      }
    }
    else if (key_pressed && c == '\0') // 在按键被释放之前，不再触发操作
    {
      key_pressed = false; // 设置按键状态为未按下
      key_released = true; // 设置按键状态为已释放
      robot1_vel = {0.0, 0.0, 0.0};
      send_wheel_controller_command(pub_robot1_v1, pub_robot1_v2, pub_robot1_v3, pub_robot1_v4, robot1_vel);
    }
    // 发布速度消息
    send_wheel_controller_command(pub_robot1_v1, pub_robot1_v2, pub_robot1_v3, pub_robot1_v4, robot1_vel);
    std::cout<<robot1_vel[0]<<','<<robot1_vel[1]<<','<<robot1_vel[2]<<std::endl;

    std::cout<<c<<std::endl;

    // 延时一段时间，以控制速度变化的频率
    rate.sleep();
    c = getch();
  }

  // 恢复终端参数
  restore_keyboard();

  return 0;
}

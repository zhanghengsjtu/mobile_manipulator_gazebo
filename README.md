根据编译过程中的提示安装对应的ros包即可

gazebo_pligins为驱动夹爪所需的依赖

调试：
roslaunch robot_gazebo single_mm_ur5.launch

rosrun robot_gazebo eefixed

rosrun robot_gazebo teleop_twist_keyboard

可用键盘控制小车运动
WSAD:前、后、左、右
JK:顺、逆时针

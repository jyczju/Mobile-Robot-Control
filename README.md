# Mobile-Robot-Control
本项目在ROS中采用gazebo仿真完成

路径规划：RRT*，并使用节点删除法对路径进行平滑

避障规划：DWA，具体实现中调大了heading项权重，以保证小车尽量沿着规划的路线走，以减少触发避障的机会，提高运行速度。注意：本项目由于未引入激光雷达模块，故无法探测障碍物，因此，代码实现中从map_server中直接读入障碍物信息（上帝视角），运算量较大

Demo视频：https://www.bilibili.com/video/BV1bM411b71R/

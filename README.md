# multi_robots_coordination

多机器人协调包

目前功能：处理地图，路径规划，多目标点发送，多机器人（turtlebot）启动

执行顺序：

map_server

测试A*算法：rosrun multi_robots_coordination test11.cpp

启动总控台：rosrun multi_robots_coordination multi_robots_coordination_master_node

    TODO：将call（srv）分别写到函数里可以单独调用，现在写在构造函数中
    
启动多个机器人，在每个机器人上分别执行：roslaunch multi_robots_coordination multirobot.launch


static_map：接受静态地图，处理地图，去除原始地图周围未知部分

astar_planner：根据上述地图提供的接口、当前位置和目标位置，利用A*算法规划出一条路径

multi_robots_coordination_master：将每个机器人对应的规划路径分别发送给每个机器人

multi_robots_coordination_slaver：

    TODO：每个机器人的执行部分，包括轨迹生成和轨迹跟踪，可以用patrol_robot包

multirobot.launch：用于启动多个机器人，turtlebot，map_server

turtlebot_minimal.launch：启动机器人，激光，AMCL，遥控

    TODO：dashgo_minimal.launch

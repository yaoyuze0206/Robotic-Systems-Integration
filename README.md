# Robotic-Systems-Integration
## Ⅰ. 硬件模块
### A. ESP32（micro-ROS）
1. 控制电机（麦克纳姆轮）运动
2. 控制舵机（升降装置）
3. 读取电机编码器数据
4. 与 Pi 5 通过 USB 串口通信
### B. Raspberry Pi 5
1. 摄像头 QR 码识别
2. LiDAR（MS200）定位与导航
3. 运行 ROS 2 节点（与 ESP32 通信）
4. 自主导航算法开发
## Ⅱ. 软件开发
### A. ESP32 底层驱动开发（电机 / 舵机控制）
【目标】让 ESP32 能独立控制电机（麦克纳姆轮）运动和舵机（升降装置）升降，为后续与 Pi 5 通信打基础。
#### 1.硬件引脚映射：
参考文档中 “Micro-ROS Controller Board 引脚分配表”，确定关键引脚：\
（1）电机引脚：如 M1 电机 PWM-A（GPNO5）、PWM-B（GPNO6），编码器 H1A（GPNO6）、H1B（GP307）；\
（2）舵机引脚：MG996 Servo 的 PWM 引脚（GPNO8，参考文档中 “Servo interface-51”）。
#### 2.编写驱动代码：
（1）电机控制：通过 PWM 信号控制电机正转、反转、制动（如使用analogWrite()函数调节占空比，实现速度控制）；\
（2）舵机控制：通过Servo.h库发送 PWM 信号（角度 0-180°），控制升降平台上下（如 10° 对应下降，170° 对应上升）；\
（3）编码器读取：通过中断引脚读取电机编码器数据，计算小车实际速度（用于运动闭环控制，避免打滑）。
#### 3.测试底层驱动：
烧录代码到 ESP32 后，通过串口监视器（VS Code/Arduino IDE 中打开，波特率 115200）发送指令（如 “M1+” 控制 M1 正转），验证电机 / 舵机是否正常响应。
### B. ESP32 与 Pi 5 的 micro-ROS/ROS 2 通信开发
【目标】实现 Pi 5（ROS 2）向 ESP32（micro-ROS）发送控制指令（如 “前进 1 米”“升降平台上升”），ESP32 向 Pi 5 反馈状态（如 “当前位置”“电机转速”）。
#### 1.定义 ROS 2 消息类型：
（1）使用标准消息：运动控制用geometry_msgs/Twist（线性速度、角速度），状态反馈用sensor_msgs/JointState（电机角度、转速）；\
（2）自定义消息（如需）：如robot_msg/RackState（货架是否抓取成功），需在 Pi 5 的 ROS 2 工作空间中创建消息包。
#### 2.ESP32 端 micro-ROS 节点开发：
（1）初始化 micro-ROS 节点（如esp32_motor_node），创建 “订阅者”（订阅 Pi 5 发送的Twist指令）和 “发布者”（发布电机状态）；\
（2）编写回调函数：当收到Twist指令时，解析线性速度（x/y 轴，对应麦克纳姆轮平移）和角速度（z 轴，对应旋转），转换为 4 个电机的 PWM 信号。
#### 3.Pi 5 端 ROS 2 节点开发：
（1）创建 “发布者” 节点（如pi_control_node），发送Twist指令（测试时可通过ros2 topic pub命令手动发送，如ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"）；\
（2）创建 “订阅者” 节点，接收 ESP32 发布的电机状态，验证通信是否正常。
### C. Pi 5 摄像头 QR 码识别开发
【目标】让 Pi 5 通过摄像头扫描场地 QR 码（起点、货架位置、终点），获取位置信息。
#### 1.摄像头驱动部署：
（1）在 Pi 5 上启用摄像头模块：通过raspi-config → “Interface Options” → “Camera” → 启用；\
（2）安装 ROS 2 摄像头功能包：sudo apt install ros-jazzy-usb-cam（若使用 USB 摄像头）或ros-jazzy-raspicam2（若使用 Pi Camera 3）；\
（3）启动摄像头节点：ros2 run raspicam2 raspicam2_node，此时会发布/image_raw图像话题。
#### 2.QR 码识别代码开发：
（1）基于pyzbar库（QR 码解码）和cv_bridge（ROS 图像与 OpenCV 转换），编写识别节点（参考步骤 2.2 中的qr_detection.py）；\
（2）优化识别逻辑：添加图像预处理（灰度化、降噪），提高 QR 码识别成功率；将识别到的 QR 码数据（如 “start_1”“rack_3”）发布为std_msgs/String话题（如/qr_data）。
#### 3.测试：
启动摄像头节点和 QR 识别节点，用手机显示 QR 码（模拟场地标识），观察终端是否正确输出 QR 码内容。
### D. Pi 5 LiDAR（MS200）定位与建图开发
【目标】通过 LiDAR 获取环境点云数据，实现小车自主定位和场地地图构建。
#### 1.LiDAR 驱动部署：
（1）参考文档中 MS200 LiDAR 链接（https://www.yahboom.net/study/MS200），下载 ROS 2 驱动包；\
（2）将驱动包放入 Pi 5 的 ROS 2 工作空间（~/ros2_ws/src），编译并安装：
```
cd ~/ros2_ws
colcon build --packages-select ms200_lidar
source install/setup.bash
```
#### 2.启动 LiDAR 节点：
（1）通过 USB 连接 MS200 LiDAR 与 Pi 5，确认设备端口（如/dev/ttyUSB1）；\
（2）启动驱动节点：ros2 run ms200_lidar ms200_lidar_node --ros-args -p serial_port:=/dev/ttyUSB1 -p baud_rate:=230400，此时会发布/scan激光点云话题。
#### 3.建图与定位：
（1）使用 ROS 2 导航框架（Nav2）：安装ros-jazzy-nav2-bringup，通过ros2 launch nav2_bringup navigation_launch.py启动导航节点；\
（2）构建场地地图：使用slam_toolbox（建图工具），启动建图节点：ros2 launch slam_toolbox online_async_launch.py，手动控制小车遍历场地，生成地图（保存为map.pgm和map.yaml）；\
（3）定位测试：加载已生成的地图，启动 AMCL（自适应蒙特卡洛定位）节点：ros2 launch nav2_bringup localization_launch.py map:=/path/to/your/map.yaml，观察 RViz（ROS 可视化工具）中小车是否能正确定位。
### E. 自主导航与任务逻辑开发
【目标】整合 QR 识别、LiDAR 定位、ESP32 控制，实现小车 “扫码启动→导航到货架→抓取货架→导航到终点→卸载” 的全流程自动化。
#### 1.导航路径规划：
（1）在 Pi 5 上使用 Nav2 的NavigateToPose动作，编写节点（如auto_nav_node），实现 “指定坐标点导航”（如从起点(0.0, 0.0, 0.0)导航到货架(2.0, 1.5, 0.0)）；\
（2）坐标点获取：通过建图时记录场地中货架、终点的坐标（在 RViz 中读取），或通过 QR 码数据关联预设坐标（如识别到 “rack_3” 则对应坐标(3.0, 2.0, 0.0)）。
#### 2.任务逻辑串联：
编写主控制节点（如main_task_node），按以下流程串联各模块：\
（1）订阅/qr_data话题，等待扫描到 “start” QR 码，启动任务；\
（2）发布导航指令，前往第一个货架坐标；\
（3）到达货架后，发布舵机控制指令（通过 ESP32），升起升降平台抓取货架；\
（4）导航到终点坐标，发布舵机指令，降下平台卸载货架；\
（5）重复步骤 2-4，直到所有货架完成运输或超时（3 分钟）。
#### 3.异常处理：
（1）添加碰撞检测：通过 LiDAR 数据检测前方障碍物，若距离小于 0.3 米，发送 “停止” 指令（/cmd_vel设为 0）；\
（2）超时处理：使用 ROS 2 的rclpy.time模块记录任务时间，超过 3 分钟自动停止。
### F. 系统集成测试与优化
【目标】确保所有模块协同工作，满足项目要求（3 分钟内完成货架运输，无碰撞）。
#### 1.单模块测试：
分别验证电机控制、QR 识别、LiDAR 定位、导航功能，确保每个模块独立运行正常。
#### 2.全系统联调：
（1）启动所有节点：ESP32 电机节点、Pi 5 摄像头节点、LiDAR 节点、主控制节点；\
（2）在模拟场地（按文档要求 300cm×200cm）放置 QR 码和货架，运行任务，观察小车是否按流程完成运输；\
（3）记录问题（如 QR 识别失败、导航偏移、碰撞），针对性优化（如调整摄像头角度、优化导航参数、增加避障距离）。
#### 3.性能优化：
（1）降低代码延迟：优化 QR 识别算法（减少图像处理时间），调整 ROS 2 话题队列大小（避免数据堆积）；\
（2）提高稳定性：增加代码容错（如通信中断时重试连接），加固硬件连接（避免运动中松动导致通信失效）。

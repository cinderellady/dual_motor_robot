================================================================================
                  M0603A 双电机 ROS 2 控制节点 - 完整技术文档
================================================================================
版本: 1.2
作者: ROS2 学习项目
日期: 2025-04-06

================================================================================
1. 总体介绍
================================================================================
本套代码用于在 ROS 2 环境下控制 M0603A FOC 电机（支持两轮差速机器人）。
主要特点：
- 两个电机共享同一个串口，通过电机 ID（1 或 2）区分指令。
- 使用类锁实现线程安全的串口访问，支持锁超时避免死锁。
- 自动检测电机是否在线（使能响应验证），离线电机不影响在线电机控制。
- 订阅 /cmd_vel 话题，通过差速模型解算左右轮目标转速（RPM）。
- 定时发布左右电机的实际转速、电流、温度、累计圈数、绝对角度。

文件列表：
  motor_driver.py          - 电机底层驱动类（M0603AMotor）
  dual_motor_controller.py - ROS 2 节点（可执行）

================================================================================
2. motor_driver.py - 详细说明
================================================================================

2.1 模块级常量
----------------------------------------
无公开常量。内部使用。

2.2 内部函数（私有）
----------------------------------------
2.2.1 _crc8_maxim(data: bytes) -> int
  功能: 计算 CRC-8/MAXIM 校验值（多项式 x^8 + x^5 + x^4 + 1，反转多项式 0x8C）。
  参数: data - 待校验的字节序列（不包含 CRC 本身）。
  返回: 8 位 CRC 值。

2.2.2 _build_cmd(motor_id: int, cmd_byte: int, data7: bytes) -> bytes
  功能: 构建一条完整的 10 字节命令帧。
  格式: [motor_id, cmd_byte, data7[0..6], crc8]
  参数:
    - motor_id: 电机 ID (1 或 2)
    - cmd_byte: 命令字节（0x64=驱动，0xA0=模式切换，0x74=获取里程）
    - data7: 7 字节数据（不足部分用 0x00 填充）
  返回: 完整的命令帧（bytes 对象）。

2.3 类 M0603AMotor
----------------------------------------
2.3.1 类变量（共享）
  _lock : threading.Lock      - 类锁，所有实例共享，保护串口访问。
  _serial : serial.Serial     - 共享串口对象（类变量），所有实例共用同一个串口。

2.3.2 类方法
  init_serial(port: str, baudrate: int = 38400) -> bool
    功能: 打开共享串口（只需调用一次）。如果已打开则直接返回 True。
    参数:
      - port: 串口设备路径，例如 "/dev/ttyACM0"
      - baudrate: 波特率，默认 38400
    返回: 成功返回 True，失败返回 False。
    注意: 此方法必须在创建任何 M0603AMotor 实例之前调用。

  close_serial() -> None
    功能: 关闭共享串口。节点退出时调用。

2.3.3 实例属性
  id : int                    - 电机 ID（1 或 2，只读）
  online : bool               - 电机是否在线（初始化成功后为 True，否则 False）
  _target_rpm : int           - 当前目标转速（用于 get_feedback 时保持不变）

2.3.4 实例方法
  __init__(self, motor_id: int)
    功能: 创建电机对象，设置 ID，初始化内部变量。
    参数: motor_id - 电机 ID（1 或 2）。

  _send_recv(self, cmd: bytes, wait: float = 0.05) -> bytes
    功能: 发送命令并接收响应（线程安全，带锁超时）。
    参数:
      - cmd: 完整的命令帧（10 字节）
      - wait: 发送后等待响应的延迟（秒）
    返回: 接收到的原始响应字节（可能为空）。
    内部行为:
      - 尝试获取类锁，超时时间 1 秒。若获取失败则打印警告并返回 b''。
      - 发送命令，等待 wait 秒，然后读取串口缓冲区的所有数据。
      - 无论是否发生异常，finally 块中释放锁。
    注意: 此方法通常不直接调用，由其他公开方法间接调用。

  enable(self) -> bool
    功能: 发送使能命令（模式切换为 0x08）。
    返回: True 表示收到有效响应（命令字节 0xA1），False 表示无响应或校验失败。

  disable(self) -> bool
    功能: 发送失能命令（模式切换为 0x09）。
    返回: True 表示收到有效响应，False 表示失败。

  set_speed(self, rpm: int) -> dict
    功能: 设置目标转速（范围 -380 ~ 380 RPM），同时返回电机的实时反馈。
    参数: rpm - 目标转速（整数）。
    返回: 字典，包含以下键（若通信失败则返回空字典 {}）：
        - 'actual_rpm' : float  实际转速（RPM）
        - 'current_A'  : float  电机电流（安培）
        - 'temp_c'     : int    温度（摄氏度）
        - 'fault'      : int    故障码（位掩码，见下方故障码说明）
    内部行为:
      - 如果 self.online == False，直接返回 {}（不发送指令）。
      - 限幅 rpm 到 [-380, 380]。
      - 更新 self._target_rpm 为实际发送的值。
      - 构建驱动指令（CMD_DRIVE = 0x64），数据部分为 2 字节转速（大端有符号16位，单位 0.1 RPM）+ 5 字节 0。
      - 调用 _send_recv 发送命令。
      - 解析响应（期望命令字节 0x65），提取实际转速、电流、温度、故障码。
    故障码位定义（参考电机手册）：
        bit0 (0x01) : 霍尔故障
        bit1 (0x02) : 过流故障
        bit3 (0x08) : 堵转故障
        bit4 (0x10) : 过温故障
        bit5 (0x20) : 断联故障
        bit6 (0x40) : 过欠压故障

  get_feedback(self) -> dict
    功能: 获取当前反馈（不改变电机转速）。
    返回: 同 set_speed 的返回字典。
    实现: 调用 set_speed(self._target_rpm)（使用最近一次的目标转速，因此电机速度不变）。

  get_odom(self) -> dict
    功能: 获取里程圈数和当前绝对角度。
    返回: 字典，包含以下键（若通信失败则返回空字典 {}）：
        - 'ticks'     : int   累计圈数（有符号32位，可正可负）
        - 'angle_deg' : float 当前绝对角度（0~360°）
        - 'fault'     : int   故障码（同上）
    内部行为:
      - 如果 self.online == False，直接返回 {}。
      - 构建获取里程命令（CMD_ODOM = 0x74），数据为 7 字节 0。
      - 调用 _send_recv（等待 0.1 秒）并解析响应（期望命令字节 0x75）。
      - 解析 4 字节有符号整数（大端）作为总圈数，2 字节无符号整数（大端）作为原始角度（0~32767 对应 0~360°）。

  init(self) -> bool
    功能: 初始化电机，包括使能、切换为速度环模式。
    返回: True 表示成功，False 表示失败（此时 self.online 被设为 False）。
    内部行为:
      - 调用 enable()，若失败则设置 online=False 并返回 False。
      - 等待 0.05 秒。
      - 发送模式切换命令，切换到速度环（MODE_SPEED = 0x02），期望响应 0xA1。
      - 若成功则设置 online=True，否则 online=False。
      - 打印相应日志。

2.4 注意事项
----------------------------------------
- 使用本驱动前必须先调用 M0603AMotor.init_serial() 打开串口。
- 多个电机对象共享同一个串口，但每个对象必须有不同的 motor_id。
- 所有公开方法（enable, set_speed, get_odom 等）都内部检查 online 标志，
  如果电机离线，方法会直接返回空或 False，不会发送命令。
- 锁超时时间固定为 1 秒，可修改 _send_recv 中的 acquire(timeout) 参数。

================================================================================
3. dual_motor_controller.py - 详细说明
================================================================================

3.1 模块级常量（可修改）
----------------------------------------
WHEEL_BASE = 0.30      # 轮距（米），左右轮中心距离
WHEEL_RADIUS = 0.05    # 轮半径（米）
MAX_RPM = 380          # 电机最大允许转速

3.2 内部函数
----------------------------------------
twist_to_rpm(vx: float, wz: float) -> tuple[float, float]
  功能: 将线速度（m/s）和角速度（rad/s）转换为左右轮目标转速（RPM）。
  公式:
    left_linear = vx - wz * WHEEL_BASE / 2.0
    right_linear = vx + wz * WHEEL_BASE / 2.0
    left_rpm = (left_linear / (2π * WHEEL_RADIUS)) * 60
    right_rpm = (right_linear / (2π * WHEEL_RADIUS)) * 60
  返回: (left_rpm, right_rpm)，已限幅到 [-MAX_RPM, MAX_RPM]。

3.3 类 DualMotorController (继承 rclpy.node.Node)
----------------------------------------
3.3.1 节点参数（可通过 --ros-args -p 设置）
  serial_port : string, 默认 "/dev/ttyACM0"
  baudrate    : int,    默认 38400
  left_id     : int,    默认 1
  right_id    : int,    默认 2
  pub_hz      : int,    默认 20   # 状态发布频率（Hz）

3.3.2 实例属性
  left   : M0603AMotor   - 左电机对象
  right  : M0603AMotor   - 右电机对象
  left_online  : bool    - 左电机是否在线（初始化返回值）
  right_online : bool    - 右电机是否在线
  发布者对象（10个）:
    l_speed, l_current, l_temp, l_ticks, l_angle
    r_speed, r_current, r_temp, r_ticks, r_angle
  订阅者对象:
    cmd_sub
  定时器:
    timer (用于 publish_states)

3.3.3 实例方法
  __init__(self)
    功能: 节点构造函数，执行以下步骤：
      1. 声明并读取 ROS 参数。
      2. 调用 M0603AMotor.init_serial() 打开串口。
      3. 创建 left 和 right 电机对象。
      4. 调用 left.init() 和 right.init()，保存 online 状态。
      5. 如果两个电机都离线，抛出异常退出节点；否则打印警告（若有电机离线）。
      6. 创建 10 个发布者（每个电机 5 个话题）。
      7. 创建订阅者，订阅 /cmd_vel，回调函数为 cmd_callback。
      8. 创建定时器，周期 = 1.0 / pub_hz，回调函数为 publish_states。
      9. 打印启动信息。

  cmd_callback(self, msg: Twist)
    功能: 处理 /cmd_vel 消息。
    行为:
      - 调用 twist_to_rpm 计算左右轮目标转速。
      - 如果 left_online 为 True，则调用 left.set_speed(int(left_rpm))。
      - 如果 right_online 为 True，则调用 right.set_speed(int(right_rpm))。
    注意: 即使只有一个电机在线，也会发送该电机的速度指令，另一电机被跳过。

  publish_states(self)
    功能: 定时读取电机反馈并发布话题。
    行为:
      - 如果 left_online:
          - 调用 left.get_feedback()，若返回非空，则发布 /left/actual_speed, /left/current, /left/temperature。
          - 调用 left.get_odom()，若返回非空，则发布 /left/odom_ticks, /left/position。
      - 同理对 right 电机执行相同操作。
    注意: 此方法由定时器触发，频率由 pub_hz 决定。如果某电机离线，则跳过其发布。

  shutdown_hook(self)
    功能: 节点退出时的清理工作。
    行为:
      - 如果 left_online，调用 left.set_speed(0) 和 left.disable()。
      - 如果 right_online，调用 right.set_speed(0) 和 right.disable()。
      - 调用 M0603AMotor.close_serial() 关闭串口。

3.3.4 主函数 main()
  功能: ROS 2 节点入口。
  行为:
    - rclpy.init()
    - 创建 DualMotorController 节点实例。
    - 调用 rclpy.spin(node) 进入事件循环。
    - 捕获 KeyboardInterrupt。
    - finally 块中调用 node.shutdown_hook(), node.destroy_node(), rclpy.shutdown()。

3.4 发布的话题类型及含义
----------------------------------------
话题名                 类型              含义
/left/actual_speed    std_msgs/Float32  左电机实际转速（RPM），可为负（反转）
/left/current         std_msgs/Float32  左电机电流（A），正值
/left/temperature     std_msgs/Float32  左电机温度（°C）
/left/odom_ticks      std_msgs/Int32    左电机累计圈数（里程，有符号，可正可负）
/left/position        std_msgs/Float32  左电机当前绝对角度（0~360°）

/right/* 同理。

3.5 订阅的话题
----------------------------------------
/cmd_vel (geometry_msgs/Twist)
  字段使用:
    linear.x  : 线速度（m/s），正为前进
    angular.z : 角速度（rad/s），正为左转（符合右手定则，俯视逆时针为正）

3.6 控制逻辑流程图
----------------------------------------
启动节点
   │
   ▼
打开串口 ──失败──> 退出
   │成功
   ▼
创建左右电机对象，调用 init()
   │
   ▼
记录 online 状态，打印日志
   │
   ▼
创建发布者、订阅者、定时器
   │
   ▼
进入 spin() 循环
   │
   ├─── 收到 /cmd_vel 消息 ──> cmd_callback()
   │                           │
   │                           ▼
   │                      计算左右目标 RPM
   │                           │
   │                           ▼
   │                      对在线电机调用 set_speed()
   │
   └─── 定时器触发 ──> publish_states()
                       │
                       ▼
                   对在线电机调用 get_feedback() 和 get_odom()
                       │
                       ▼
                   发布所有状态话题

用户按 Ctrl+C ──> shutdown_hook() 停止电机，关闭串口，退出

================================================================================
4. 安装与编译
================================================================================
4.1 依赖安装
  sudo apt update
  sudo apt install python3-serial
  # 确保已安装 ROS 2 (Humble) 和 colcon

4.2 创建工作空间（如果尚未创建）
  mkdir -p ~/ROS2_learn/learn_ws/src
  cd ~/ROS2_learn/learn_ws/src
  ros2 pkg create my_first_node --build-type ament_python --dependencies rclpy std_msgs geometry_msgs

4.3 放置代码文件
  将 motor_driver.py 和 dual_motor_controller.py 放入
  ~/ROS2_learn/learn_ws/src/my_first_node/my_first_node/ 目录下。

4.4 修改 setup.py
  在 my_first_node/setup.py 的 entry_points 中添加:
    entry_points={
        'console_scripts': [
            'dual_motor_controller = my_first_node.dual_motor_controller:main',
        ],
    },

4.5 编译
  cd ~/ROS2_learn/learn_ws
  colcon build --packages-select my_first_node
  source install/setup.bash

================================================================================
5. 运行与测试
================================================================================
5.1 启动节点（连接两个电机）
  ros2 run my_first_node dual_motor_controller --ros-args \
      -p serial_port:=/dev/ttyACM0 \
      -p left_id:=1 -p right_id:=2

5.2 发送速度指令（例如 0.2 m/s 直线前进，持续以 20Hz 发布）
  ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" -r 20

5.3 查看状态
  ros2 topic echo /left/actual_speed
  ros2 topic echo /right/odom_ticks

5.4 停止电机
  ros2 topic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}}"
  或者直接 Ctrl+C 停止节点（节点退出时会自动停止电机）。

================================================================================
6. 常见问题及解答
================================================================================
Q1: 运行节点时提示“右电机离线，将只控制左电机”，但右电机已连接。
A1: 检查右电机的 ID 线是否设置为高电平（ID2），以及电机是否已上电。
    如果只使用一个电机，可以忽略该警告。

Q2: 电机转速波动较大（±2~3 RPM）是否影响导航？
A2: 对于低速移动机器人（<0.5 m/s），该波动对路径跟踪影响较小。
    如需更高精度，可尝试提高 /cmd_vel 发布频率（如 -r 50）或调整电机 PID。

Q3: 停止发布 /cmd_vel 后电机仍在转动，如何自动停止？
A3: 本节点未实现超时保护。可以自行在节点中添加计时器，或使用 -r 选项持续发布速度 0。
    如果需要超时保护，请参考后续扩展说明。

Q4: 电机实际转速与目标转速有稳态误差（例如目标 100，实际 95）？
A4: 这可能是电机驱动器的速度环积分不足。尝试通过串口修改电机 PID 参数（需查阅电机手册），
    或者在节点中增加一个外部 PID 补偿（不推荐，因为驱动器内部已有速度环）。

Q5: 串口通信偶尔失败，返回空字典怎么办？
A5: 检查串口连接是否稳定，降低通信频率（pub_hz 不要超过 50），
    或者增加 _send_recv 中的 wait 参数（如 0.1 秒）。

================================================================================
7. 扩展与二次开发指南
================================================================================
7.1 添加指令超时保护
  在 DualMotorController 中添加：
    self.last_cmd_time = self.get_clock().now()
    self.cmd_timeout = 1.0  # 秒
  在 cmd_callback 中更新 self.last_cmd_time = self.get_clock().now()。
  在 publish_states 中检查超时：
    now = self.get_clock().now()
    if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
        self.left.set_speed(0); self.right.set_speed(0)

7.2 修改电机控制频率
  调整 /cmd_vel 的发布频率（-r 参数）和节点内部定时器频率（pub_hz）。
  建议两者保持一致或控制频率高于状态发布频率。

7.3 增加里程计计算节点
  可以编写单独的 odometry_node，订阅 /left/odom_ticks 和 /right/odom_ticks，
  通过差速模型计算机器人位姿，并发布 /odom 和 /tf。

7.4 支持更多电机（如四轮）
  可以复制 M0603AMotor 对象，但注意串口共享锁机制仍适用。
  需要修改 twist_to_rpm 函数以适配四轮模型。

================================================================================
8. 故障码位掩码解释
================================================================================
故障码（fault）为 8 位无符号整数，位定义如下（来自电机手册）：
  位0 (0x01): 霍尔传感器故障
  位1 (0x02): 过流故障
  位3 (0x08): 堵转故障
  位4 (0x10): 过温故障
  位5 (0x20): 断联故障（通信丢失）
  位6 (0x40): 过压或欠压故障
其他位保留。

示例：如果 fault = 0x12 (二进制 00010010)，表示同时存在过流(0x02)和过温(0x10)故障。

================================================================================
9. 版本更新记录
================================================================================
v1.0 (2025-04-01): 初始版本，支持双电机，单串口，轮询。
v1.1 (2025-04-05): 增加锁超时、响应验证、离线检测。
v1.2 (2025-04-06): 完善文档，增加故障码说明，优化代码注释。

================================================================================
10. 联系方式与技术支持
================================================================================
如有问题，请先查阅本文档。若仍无法解决，请在项目讨论区提问。
# car_agriculture · 六轮遥控小车

基于 **STM32F407VET6** 的六轮差速小车固件，使用 STM32 HAL、CMake 和 ARM GCC 开发。支持 SBUS 遥控驾驶、三路 TOFSense 测距、双舵机联动、PWM 电子开关、OLED 显示和 VOFA 串口监控。

仓库名称为 `car_agriculture`，固件工程名称为 `car_code_7_21`。本文按当前源码说明功能；历史框架文档中的部分引脚、初始化说明和参数已发生变化。

## 功能与当前状态

| 功能 | 当前状态 |
| --- | --- |
| 六轮遥控驾驶 | 已接入主循环，右摇杆控制前后与差速转向，支持原地转向 |
| 遥控失联处理 | 未收到合法帧、识别到失控标志或超过 200ms 无新帧时，车轮 PWM 清零、电子开关关闭、舵机回中 |
| 电机换向保护 | 换向前两侧 PWM 清零等待 500ms，切换方向后再等待 50ms，非阻塞执行 |
| 三路激光测距 | USART3 按 ID 0、1、2 轮询，每 30ms 查询一个模块 |
| 测距避障 | 算法代码保留，当前未接入车轮控制；CH8 已改为控制开关 4、5 |
| 双舵机 | CH3 控制，PE5/PE6 分别以 1400/1500μs 为中位反向联动 |
| 五路电子开关 | 五路输出已实现，CH5～CH7 分别控制前 3 路，CH8 同时控制第 4、5 路 |
| OLED | I2C SSD1306，显示三路距离及 CH1～CH8 原始值 |
| VOFA | USART6 输出 32 通道 JustFloat 数据，发送间隔至少 50ms |
| 循迹任务、编码器、陀螺仪、PID | 保留相关模块；当前未接入完整运行流程 |

## 硬件与接线

现有项目资料记录的硬件组合：STM32F407VET6、Jumper T12 MAX 遥控器、AION-RX-MINI-SE 接收机、六个 JGB37-520 五线电机、三个 TOFSense 模块、两个舵机、五路 PWM 电子开关和 SSD1306 I2C OLED。工程还保留 8 路循迹传感器、编码器与 JY62/CY-Z 航向传感器相关代码。

系统时钟配置以 25MHz 外部晶振为输入，PLL 配置对应 168MHz 系统时钟。更换控制板或晶振时，需要同步检查时钟和外设时序。

### 六轮电机

`motor_forward(speed1, …, speed6)` 的参数 1～3 对应右侧三轮，4～6 对应左侧三轮。PWM 比较值正常范围为 0～999，方向通过独立 F/R 引脚控制。

| 电机 | 位置 | PWM 引脚 | 定时器通道 | F/R 引脚 |
| --- | --- | --- | --- | --- |
| A / 1 | 右侧 | PE9 | TIM1 CH1 | PB3 |
| B / 2 | 右侧 | PE11 | TIM1 CH2 | PB5 |
| C / 3 | 右侧 | PE13 | TIM1 CH3 | PB7 |
| D / 4 | 左侧 | PE14 | TIM1 CH4 | PD0 |
| E / 5 | 左侧 | PA2 | TIM5 CH3 | PD2 |
| F / 6 | 左侧 | PA3 | TIM5 CH4 | PD4 |

当前代码对 **D / 4 号电机的 F/R 电平单独取反**，用于补偿实际接线方向。更换电机或线束后，需核对 `motor.c` 初始化和 `yaokong.c` 方向控制逻辑。AIN2～FIN2 不参与当前五线电机驱动。

### 串口、显示与舵机

| 用途 | MCU 引脚 | 配置 / 说明 |
| --- | --- | --- |
| SBUS 接收 | PD6（USART2 RX） | 100000 波特，8 数据位、偶校验、2 停止位；HAL 使用 9B 字长实现 8E2 |
| TOFSense 查询发送 | PC10（USART3 TX） | 921600，8N1 |
| TOFSense 数据接收 | PB11（USART3 RX） | 三个模块配置独立 ID，并使用查询模式 |
| 文本诊断输出 | PA9（USART1 TX） | 115200，8N1，接 USB-TTL 的 RX |
| VOFA 输出 | PC6（USART6 TX） | 115200，8N1，接 USB-TTL 的 RX |
| OLED SCL / SDA | PB8 / PB9（I2C1） | SSD1306，7 位地址 0x3C |
| 舵机 1 / 2 | PE5 / PE6 | TIM9 CH1 / CH2，周期 20ms |
| 预留航向传感器 | PA9 / PA10（USART1 TX / RX） | 需启用 `jy62_init()`；与现有诊断串口共用资源 |

接收机需要输出 **SBUS**。若输出为反相 SBUS，需要外部反相电路，将信号转换为 MCU 串口可接收的极性。不能直接把 CRSF 数据当作 SBUS 使用。

TOFSense 模块分别设置为 ID 0、1、2，安装位置按车辆自身左右定义：**ID0 左前、ID1 正前、ID2 右前**。查询级联的接法需符合所用模块的硬件规格，不要将主动连续发送的普通串口输出直接并联。

控制信号按 3.3V TTL 电平核对并共地；电机、舵机和电子开关负载使用符合各自规格的供电，不通过 GPIO 供电。首次测试先架空车轮、断开最终负载，确认方向与输出状态。

### 五路电子开关

| 开关 | 信号引脚 | 当前控制来源 |
| --- | --- | --- |
| 1 | PE0 | CH5 严格等于 1792 时开启 |
| 2 | PE7 | CH6 严格等于 1792 时开启 |
| 3 | PE8 | CH7 严格等于 1792 时开启 |
| 4 | PE10 | CH8 严格等于 1792 时开启 |
| 5 | PE12 | CH8 严格等于 1792 时开启 |

TIM6 中断调度 GPIO 边沿，输出约 50Hz 脉冲。默认 1000μs 表示关闭、2000μs 表示开启；模块实际通断方向需要实测，可用 `RC_SWITCH_INVERT_MASK` 逐路反转。程序发出关闭指令不代表已经测量到负载断电；复位或 MCU 停止时的通断由模块自身无信号行为决定。

完整接线、接口和校验方法见 [五路电子开关接线说明](五路电子开关接线说明.md)。

## 遥控操作

| SBUS 通道 | VOFA 通道 | 用途 |
| --- | --- | --- |
| CH1 | I6 | 右摇杆左右，控制转向 |
| CH2 | I7 | 右摇杆上下，控制前后速度 |
| CH3 | I8 | 左摇杆左右，联动两个舵机 |
| CH4 | I9 | 保留原始值监控，未分配当前控制动作 |
| CH5 | I10 | 电子开关 1 |
| CH6 | I11 | 电子开关 2 |
| CH7 | I12 | 电子开关 3 |
| CH8 | I13 | 同时控制电子开关 4、5 |

普通驾驶使用 `左侧 = 油门 + 转向`、`右侧 = 油门 - 转向` 的差速混控。通道中位默认 992，死区为 ±40 附近；通道宏使用从 0 开始的数组下标，例如 CH2 对应下标 1。

CH3 按上、中、下三档选择固定目标，拨杆在中间区域移动时都算作回中：

| CH3 原始值（从中档出发） | 档位 | 右舵机 1 / PE5 目标 | 左舵机 2 / PE6 目标 |
| --- | --- | --- | --- |
| 小于 742 | 下 / 后拨 | 970μs | 2100μs |
| 742～1242 | 中 | 1400μs | 1500μs |
| 大于 1242 | 上 | 1900μs | 900μs |

边界增加 40 个通道单位的回差防抖：下档回中需要 CH3 ≥ 782，上档回中需要 CH3 ≤ 1202。上/下档可直接切换，运动途中以最新档位为目标。通道数值增大默认对应上档；遥控器通道反向设置可交换上/下方向。沿用 CH3 输入，实际拨杆需在遥控器混控中分配到 CH3。

右舵机（舵机 1 / PE5）中位 1400μs、上档目标 1900μs、下档目标 970μs；左舵机（舵机 2 / PE6）中位 1500μs、上档 900μs、下档 2100μs。端点在 `User/motor.h` 独立配置，不再使用固定脉宽和或相同偏移。右侧从中位进入上档时脉宽增加（1400→1900μs），左侧脉宽减小（1500→900μs）。实际角度需按舵机和连杆测量，不再将右侧上档标作 +45°。

两路共用一条运动进度。每次切换档位，从两路当前指令位置出发，按各自到目标的剩余行程比例移动，在同一次 20ms 更新中到达端点。后拨回中、途中回中和反向都会重新规划，避免按相同脉宽步长控制不同长度行程而使短行程提前到达。这里同步的是 PWM 指令；程序没有舵机角度反馈，实际机械同步仍受负载、舵机速度、供电及连杆阻力影响。

SysTick 每 20ms 独立更新，三档逻辑位置顺序为下 → 中 → 上。向较高档位运动（包括后拨回中）采用慢速参数，较长的那一路最大每步 3μs（150μs/秒），每次更新速度增加 0.125μs/步。向较低档位运动最大每步 8μs，每次速度增加 0.5μs/步。接近目标时按剩余共享行程分别除以 16（抬升）/ 8（下降）减速，最终精确到达目标。切换目标从当前两路输出重新低速起步。主机模拟中：下档回中约 4.58 秒、中位到上档约 4.58 秒、中位到下档约 1.96 秒、下档到上档约 8.58 秒。

失联在下一次 20ms 更新时直接回到各自中位（右 1400μs、左 1500μs），并清除档位和轨迹记忆。`SERVO_LIFT_MAX_STEP_US` 为抬升速度上限，`SERVO_LIFT_ACCEL_Q8` 为抬升起步加速度（256 对应 1μs/步），`SERVO_LIFT_DIVISOR` 控制末段减速。中间区域和防抖仍由 `SERVO_RC_CENTER_BAND`、`SERVO_RC_HYSTERESIS` 控制。放慢能减轻启动冲击，但无法提高舵机静态扭矩。

CH5～CH8 使用**精确相等判断**：1792 触发对应功能，1791、1793 及其他值均不触发。请先通过 OLED 或 VOFA 确认遥控器实际输出，并在遥控器混控设置中分配好通道。

### CH8 / I13 控制电子开关 4、5

遥控链路正常且 CH8 严格等于 1792 时，同时开启开关 4（PE10）和开关 5（PE12）；其他值同时关闭。VOFA I13 显示 CH8 原始值，I4/I5 显示两路开关指令。遥控失联或失控时五路全部关闭。车轮始终使用右摇杆遥控，不再由 CH8 启用避障；测距采集与显示继续运行，`User/avoidance.c/h` 作为未接入的算法代码保留。

## 获取与编译

### 开发环境

- CMake 3.22 或更高版本。
- Ninja 构建工具。
- `arm-none-eabi-gcc` 工具链，包含相应链接器与 Newlib / nano 标准库。
- OpenOCD 与 ST-Link 或 CMSIS-DAP 调试器，用于烧录和调试。
- STM32CubeMX：仅在需要调整 `.ioc` 外设配置时使用。
- 本机 GCC：仅用于运行独立避障算法测试。

交叉编译工具链会查找项目内 `.tools/arm-none-eabi/usr/bin` 以及系统 `PATH`。`.tools/` 被 Git 忽略，从 GitHub 克隆后需自行准备工具链。

```bash
git clone https://github.com/JJL0612/car_agriculture.git
cd car_agriculture

# Debug
cmake --preset Debug
cmake --build --preset Debug

# Release（按需）
cmake --preset Release
cmake --build --preset Release
```

Debug 主要产物为 `build/Debug/car_code_7_21.elf`，构建目录还包含链接映射文件及编译命令数据库。Release 产物位于 `build/Release/`。当前构建配置未自动生成 HEX/BIN，需要时执行：

```bash
arm-none-eabi-objcopy -O ihex build/Debug/car_code_7_21.elf build/Debug/car_code_7_21.hex
arm-none-eabi-objcopy -O binary build/Debug/car_code_7_21.elf build/Debug/car_code_7_21.bin
```

### 烧录

在工程根目录，使用 ST-Link：

```bash
openocd -f stlink.cfg -c "program build/Debug/car_code_7_21.elf verify reset exit"
```

使用 CMSIS-DAP 时可将配置替换为 `daplink.cfg`。该配置当前显式设置 `FLASH_SIZE=0x40000`，与链接脚本中的 512KB Flash 配置不同；使用前需按实际芯片容量核对并调整。调试器通过 SWD 接线，连接 SWDIO、SWCLK、GND 及调试器要求的参考电压。

### 独立算法测试

避障模块不依赖 HAL，可在电脑上运行现有测试：

```bash
gcc -std=c11 -Wall -Wextra -Werror -pedantic -IUser \
  User/avoidance.c tests/avoidance_test.c -o /tmp/avoidance_test
/tmp/avoidance_test
```

测试覆盖通畅前进、左右转向、转向保持与释放、障碍停车、无效测量、数据过期及时间计数回绕等情况。该测试不验证板上电气输出、实际制动距离或完整遥控链路。

## 调试与监控

### VOFA

将 PC6 接 USB-TTL RX，连接 GND，使用 3.3V TTL 电平。VOFA 中选择 **115200 / 8N1 / 无流控**，协议引擎选择 **JustFloat**。

固件每隔至少 50ms 尝试使用 DMA 发送一帧：32 个小端 32 位浮点数，加 `00 00 80 7F` 帧尾，共 132 字节。实际频率受主循环耗时影响。

| VOFA 通道 | 数据 |
| --- | --- |
| I0 | 遥控链路状态，1 正常 / 0 异常 |
| I1～I5 | 五路电子开关指令，非实测反馈 |
| I6～I21 | SBUS CH1～CH16 原始值 |
| I22～I24 | 右侧三个电机的 PWM 比较值 |
| I25～I27 | 左侧三个电机的 PWM 比较值 |
| I28 | SBUS 接收回调累计次数 |
| I29 | 合法 SBUS 帧累计数 |
| I30 | 最近一次接收字节数 |
| I31 | 最近一帧 SBUS 标志字节 |

详细名称见 [VOFA 通道对照](VOFA通道对照.csv)。JustFloat 帧只包含数值，不包含名称；修改固件通道布局后需重新绑定控件。当前 32 通道中不含 TOFSense 距离和电机方向；PWM 值也不代表实测转速。失联后原始通道保留最后值，应结合 I0 判断有效性。

USART6 当前专用于 VOFA，`TOFSENSE_ENABLE_UART6_BUS` 必须保持 0。

### 文本串口与 OLED

PA9 输出 SBUS 和 TOFSense 文本诊断，串口助手设置 115200 / 8N1。可分别通过 `YAOKONG_DEBUG_ENABLE`、`TOFSENSE_DEBUG_ENABLE` 关闭，减少主循环中的阻塞打印耗时。

OLED 使用 I2C1，显示三路距离和遥控 CH1～CH8。代码中仍保留 SPI2 初始化和旧 OLED 的 RES/DC 定义，当前显示驱动实际使用 I2C。

### 常见问题

| 现象 | 排查方向 |
| --- | --- |
| 找不到交叉编译器 | 检查 `arm-none-eabi-gcc` 是否位于 PATH，或配置项目内工具链 |
| SBUS 无数据，I28 不增长 | 检查接收机供电、PD6 接线、输出协议、信号极性及 UART 错误 |
| I28 增长但 I29 不增长 | 检查是否输出 CRSF、SBUS 反相、100000/8E2 配置 |
| I29 增长但 I0 为 0 | 检查 I31 失控标志及帧是否持续更新 |
| 电子开关无法开启 | 检查对应 CH5～CH8 是否严格输出 1792，并确认 I0 为 1 |
| 某一路测距不更新 | 查看 USART1 的查询数、各 ID 合法帧数和坏帧数，核对 ID、波特率与接线 |
| VOFA 显示异常 | 确认接的是 PC6、选择 JustFloat，且没有混入 USART1 文本输出 |
| 电机换向有短暂停顿 | 当前设计包含换向等待；先核对保护状态，再按实车制动情况调参 |

## 代码结构与执行流程

```text
.
├── Core/                         # HAL 初始化、主循环及中断分发
├── Drivers/                      # STM32 HAL 与 CMSIS
├── User/
│   ├── yaokong.c/h               # SBUS 解析、混控、模式切换、失联与换向处理
│   ├── avoidance.c/h             # 三路测距避障算法与阈值
│   ├── motor.c/h                 # 六路电机 PWM 与双舵机
│   ├── rc_switch.c/h             # TIM6 调度五路电子开关脉冲
│   ├── tofsense.c/h              # NLink 解析及按 ID 查询
│   ├── vofa.c/h                  # JustFloat 监控输出
│   ├── oled.c/h                  # I2C SSD1306 显示
│   ├── sensor.c/h                # 保留的循迹算法
│   ├── task.c/h                  # 初始化入口与保留的任务状态机
│   ├── encoder.c/h               # 保留的编码器里程模块
│   ├── jy62.c/h                  # 保留的 CY-Z 航向解析
│   └── pid.c/h                   # 预留 PID 控制器
├── tests/avoidance_test.c         # 独立避障测试
├── cmake/                        # 交叉工具链及 CubeMX 构建配置
├── car_code_7_21.ioc              # CubeMX 外设配置
├── CMakeLists.txt                # 工程入口与应用源文件列表
├── CMakePresets.json             # Debug / Release 预设
├── STM32F407xx_FLASH.ld           # 链接脚本
├── startup_stm32f407xx.s          # 启动文件
├── stlink.cfg / daplink.cfg       # OpenOCD 配置
├── VOFA通道对照.csv
├── 五路电子开关接线说明.md
└── 整体代码框架说明.md             # 历史架构资料，部分说明需结合现代码核对
```

启动时先初始化 HAL、时钟及外设，再启动五路电子开关和应用初始化。`init()` 当前调用电机/舵机、SBUS、TOFSense 和 OLED 初始化。

主循环依次执行：

```text
yaokong_control()       遥控状态检查、五路开关更新、摇杆驾驶控制
vofa_task()             监控数据上传
tofsense_poll()        三路测距查询调度
tofsense_debug_task()  测距文本诊断
OLED_ShowDistance()    显示距离
OLED_ShowChannels()    显示遥控原始值
```

串口接收通过 DMA 与空闲事件回调进入相应协议解析；TIM6 回调用于电子开关脉冲调度。

### 扩展与参数修改

- 遥控通道、中位、死区、速度上限与换向等待：`User/yaokong.h`。
- 避障距离阈值、数据超时与行驶 PWM：`User/avoidance.h`。
- 舵机行程：`User/motor.h`；双舵机映射：`User/motor.c`。
- 电子开关脉冲极性：`User/rc_switch.h`。
- 测距数量、波特率和轮询间隔：`User/tofsense.h`。
- VOFA 发送间隔：`User/vofa.c`；通道布局：`User/vofa.h`。

当前未调用 `task_run()`、`Encoder_Init()` 或 `jy62_init()`，PID 也未接入控制回路。启用循迹任务前，需要补齐初始化、协调 USART1 用途，并设计遥控与任务的控制权切换，避免同时写入电机输出。PE5 现已用于舵机，不应沿用旧资料中的 KEY4 接线。

重新使用 CubeMX 生成代码时，保留 USER CODE 区域并检查应用源文件及外设配置。TIM6 当前由用户代码维护，不要再生成同名初始化函数造成重复定义。

## 更新到 GitHub

修改完成后，在工程根目录执行：

```bash
git add .
git commit -m "描述本次修改"
git push
```

`build/`、`.tools/`、`cmake-build-*/` 和 `.idea/` 已被忽略，不随源码上传。

## 许可

仓库当前没有统一的顶层 LICENSE。HAL、CMSIS 等第三方组件的许可见各自目录与源文件声明；发布或复用时应保留对应版权与许可信息。

# car_agriculture

基于 **STM32F407VET6** 的六轮农业遥控车固件，集成 SBUS 遥控、三路激光测距、舵机联动与串口监控。

`STM32 HAL` · `C11` · `CMake` · `ARM GCC`

## 功能

- **六轮差速驾驶**：前进、后退、转向与原地转向，带非阻塞换向保护。
- **双舵机联动**：CH3 三档控制，独立端点标定、平滑加减速与同步到位。
- **作业输出**：CH5～CH7 控制三路电子开关，CH8 控制两只 SG90 反向切换。
- **测距与监控**：三路 TOFSense、SSD1306 OLED、VOFA JustFloat 32 通道输出。
- **失联处理**：超过 200ms 无有效帧或收到失控标志时，停车、关闭开关、CH3 舵机回中，SG90 恢复默认位置。

> 当前为遥控版本。避障算法保留但未接入驾驶控制；循迹、编码器、航向传感器与 PID 尚未接入完整控制回路。

## 遥控与接口

| 通道 | 功能 | 信号引脚 |
| --- | --- | --- |
| CH1 / CH2 | 转向 / 前后速度 | 六路电机 PWM：PE9、PE11、PE13、PE14、PA2、PA3 |
| CH3 | 原有双舵机，上 / 中 / 下三档 | PE5 / PE6 |
| CH4 | 仅监控原始值 | — |
| CH5～CH7 | 电子开关 1～3 | PE0 / PE7 / PE8 |
| CH8 | 两只 SG90 反向切换 | PE10 / PE12 |

CH5～CH7 **严格等于 1792** 时开启对应开关。CH8 严格等于 1792 时，PE10 / PE12 目标为 **90° / 0°**；其他值、启动或失联时为 **0° / 90°**。角度需按实际舵机标定。

| 外设 | 接口 | 配置 |
| --- | --- | --- |
| SBUS 接收机 | USART2 RX：PD6 | 100000 / 8E2，反相 SBUS 需外部反相 |
| TOFSense ×3 | USART3 TX：PC10，RX：PB11 | 921600 / 8N1；ID 0 / 1 / 2 对应左前 / 正前 / 右前 |
| OLED | I2C1：PB8 / PB9 | SSD1306，地址 0x3C |
| VOFA | USART6 TX：PC6 | 115200 / 8N1，JustFloat |
| 文本诊断 | USART1 TX：PA9 | 115200 / 8N1 |

接线与输出标定见 [接线说明](五路电子开关接线说明.md)，监控通道见 [VOFA 通道对照](VOFA通道对照.csv)。舵机使用独立匹配电源，并与主控共地。

## 快速开始

准备 **CMake ≥ 3.22、Ninja、ARM GNU Toolchain**；烧录需要 OpenOCD 和 ST-Link。`arm-none-eabi-gcc` 需在 `PATH` 中，也可放在 `.tools/arm-none-eabi/usr/bin/`。

```bash
git clone https://github.com/JJL0612/car_agriculture.git
cd car_agriculture

cmake --preset Debug
cmake --build --preset Debug

openocd -f stlink.cfg -c "program build/Debug/car_code_7_21.elf verify reset exit"
```

产物：`build/Debug/car_code_7_21.elf`。构建 Release 时，将预设名换为 `Release`。使用 STM32CubeMX 调整外设时打开 `car_code_7_21.ioc`。

## 工程结构

```text
Core/           外设初始化、主循环与中断
Drivers/        STM32 HAL 与 CMSIS
User/           遥控、电机、舵机、测距与显示模块
cmake/          交叉编译工具链与 CubeMX 构建配置
tests/          主机端控制逻辑测试
```

常用参数入口：

| 文件 | 参数 |
| --- | --- |
| [User/yaokong.h](User/yaokong.h) | 遥控通道、死区、速度与换向等待 |
| [User/motor.h](User/motor.h) | CH3 双舵机端点与运动速度 |
| [User/rc_switch.h](User/rc_switch.h) | 电子开关极性、SG90 脉宽标定 |
| [User/tofsense.h](User/tofsense.h) | 测距配置与轮询间隔 |
| [User/vofa.h](User/vofa.h) | 监控通道布局 |

## 测试

需要 Python 3 和本机 GCC，在工程根目录运行：

```bash
python3 tests/remote_switch_test.py
python3 tests/servo_smoothing_test.py
python3 tests/rc_outputs_test.py

gcc -std=c11 -Wall -Wextra -Werror -pedantic -IUser \
  User/avoidance.c tests/avoidance_test.c -o /tmp/avoidance_test
/tmp/avoidance_test
```

主机测试覆盖遥控失联与恢复、开关组合、舵机轨迹、GPIO 脉冲时序及独立避障算法；实车供电、方向与机械行程需上车验证。

## 许可

当前未设置项目级 LICENSE。第三方 HAL、CMSIS 等组件遵循各自目录中的许可声明。

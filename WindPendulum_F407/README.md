# 风力摆 F407 固件

项目使用天空星青春版 STM32F407VET6，在 STM32CubeIDE 1.19.0 中构建；FreeRTOS Kernel 版本为课程资料中的 202406.04 LTS。当前开发电机1台架测试，完整 OLED 菜单待各题任务稳定后统一接入。工作区新版包含三键锁存控制与 100 ms ADC/电流状态页，本轮已于 2026-11-16 21:09（北京时间）完成编译、J-Link 下载校验及复位运行。

需求以根目录 [风力摆控制系统(B题).pdf](../风力摆控制系统(B题).pdf) 和用户确认的调整为准；训练题文档已删除，其附加验收项不再纳入项目。保留 IMU 静止校准与零偏处理；完整数据记录作为可选调试功能，电脑端串口采集已经可以按会话保存 CSV、原始字节和事件。

2026-11-16：默认 `app_config.h` 启用 `APP_MOTOR1_TEST_ENABLED=1`、`APP_OLED_ENABLED=0`、`APP_MOTOR1_STATUS_OLED_ENABLED=1`。应用运行电机1测试（优先级 5、10 ms）、串口日志和 OLED 状态页（优先级 1、100 ms）；页面显示状态、三键按下状态、PWM/CCR、U1 电压、PB0 原始 ADC/换算电流、EN 与停止原因，使用独立快照队列和差分刷新。PWM 初始化清零后 EN 默认拉高，待机和普通停止以 PWM=0 保持停转；KEY1=PC13、KEY2=PC0、KEY3=PC1。单击 KEY1 后固定 30%；单击 KEY2 后从 20% 每 100 ms 增加 1 个百分点，6 秒达到 80% 后保持；KEY3 立即将 PWM 清零。松开启动键继续运行，取消旧版 3 秒上限。PWM 为 20 kHz，其他三路始终为零。具体接线、操作和检查记录见 [电机1测试](电机1测试.md)。下文的题目控制链路仅在测试开关关闭时启动。

项目后续开发统一遵循 [项目逻辑框架](项目逻辑框架.md)：以 OLED 选择比赛题目 Q1–Q7，每问由独立任务块执行，并通过统一的参数检查、任务生命周期、结果记录和安全停止流程。

## 工程构建

后续统一复用 [编译烧录流程](编译烧录流程.md) 与 `Tools/build_flash.ps1`，在本工程目录执行 `pwsh -NoProfile -ExecutionPolicy Bypass -File .\Tools\build_flash.ps1 -Action BuildFlash` 即完成完整编译、下载、校验和运行。

在 CubeIDE 中选择 **File > Import > Existing Projects into Workspace**，导入 `WindPendulum_F407` 文件夹并构建 `Debug` 配置。`.ioc` 保存 CubeMX 引脚和外设配置。Debug/Release 均使用 Cortex-M4、FPv4-SP-D16 和 hard-float ABI；HAL/CMSIS 来源记录在 `Drivers/UPSTREAM_VERSIONS.txt`。

## 硬件接口

| 设备 | 接口配置 |
|---|---|
| OLED | SSD1306，128×64；I2C1 PB6/PB7，100 kHz，7 位地址 `0x3C`（HAL 地址 `0x78`） |
| ICM-45686 | SPI2 PB10/PC2/PC3，PE7 CS；SPI Mode 3，2.625 MHz；传感器量程 ±16 g、±2000 dps，ODR 800 Hz |
| 按键 | KEY1=PC13、KEY2=PC0、KEY3=PC1；台架为固定30%/斜坡20–80%/停止，题目菜单保留确认/上/下，低电平有效 |
| 调试串口 | USART1 PA9/PA10，115200 8-N-1；当前 USB-UART 枚举为 COM9 |
| 扩展串口 | USART2 PD5/PD6，115200 8-N-1 |

完整引脚及预留功能见 `硬件引脚分配.md`。SWD 保留 PA13/PA14，USB 和 TF 卡引脚不分配给本固件。

PB0 / ADC1_IN8 / I1 的电流换算、接线假设和标定见 [ADC电流检测](ADC电流检测.md)。PB1 / ADC1_IN9 / U1 的电压采样换算、VREFINT 补偿、滤波和校准步骤见 [ADC电压检测](ADC电压检测.md)。电机板 H3 的 `Vout` 位于 INA240 电流检测输出链路，H6-U1 是否接到该点需按飞线实物确认；当前读数是中性的 U1 引脚电压占位值，不是经验证的母线电压。

## 菜单和状态

根菜单包含 Live Status、IMU Data、Calibration、Display 和 About；IMU Data 下分 Attitude、Accelerometer、Gyroscope。Display 提供 OLED 对比度调节。状态页显示传感器在线状态、姿态角、加速度和陀螺仪原始值/换算值、温度及校准进度。

IMU 任务以 500 Hz 轮询数据。启动校准要求设备静止并连续采集 5000 个有效样本，约需 10 秒；期间检测到运动或加速度模长超限会重新累计。姿态阶段使用陀螺仪积分与加速度倾角进行互补融合，并在快速运动时降低加速度权重。

上述校准属于题目控制链路：计算三轴陀螺仪均值并在姿态积分前扣除零偏，偏置保存在 RAM，重启重校；当前独立电机1测试模式不启动 IMU 链路。后续保留校准进度/偏置显示，待机重校准入口尚待实现。

## FreeRTOS 与输出限制

任务链为 `imu -> fusion -> trajectory -> motor -> telemetry`，UI 通过队列读取显示状态。USART1 输出 CSV 遥测，覆盖 IMU、姿态、目标、PID 分项、PWM、原始电流 ADC 和模式。电脑端使用 [电脑串口采集](电脑串口采集.md) 保存每次会话的带主机时间 CSV、原始字节、事件和字段说明；当前采集器只解析电机1测试的 28 列协议，后续 IMU/题目遥测需增加对应适配器。采集器不向 MCU 写入，不改变采样与控制；它仍是串口快照记录，不能宣称 MCU 全采样率无丢失。

Q1 算法骨架已通过 `PendulumApp_RequestQ1Start` 接收轨迹和控制参数，并计算目标姿态、PID 分项及逻辑 PWM1..PWM4 请求值；目前尚未接入 OLED 题目菜单，混控参数也未实测标定。MotorTask 仍强制 PD7 电机使能为低、TIM1 四路比较值为零，PD8 激光保持低电平，因此算法请求不会驱动电机。主控 PWM 网名顺序与 TIM1 通道相反，焊接时需交叉连接，使电机板 PWM1..PWM4 分别落到 TIM1_CH1..CH4；PD10 暂不部署。

电机1测试为独立的执行器入口，只有测试任务可以输出 CH1，正常题目链路的门控仍保留。PD7 改为开漏并启用内部上拉，以便电机板 LM393/2N7002 下拉 EN 时不会与 MCU 推挽高电平冲突；5 V SN74HC08 对 3.3 V 输入的兼容性仍需实测。

## 下载与实测状态

调试探针使用 J-Link GDB Server、SWD、目标 `STM32F407VE`，建议初始速率 4000 kHz。CubeIDE 的 Debug Configuration 中选择 J-Link，不要使用课程模板里的 ST-LINK/F103 配置。编译成功不代表 OLED 或 IMU 已通过实物验证；上板结果应以 COM9 遥测和显示观察为准。

当前源码操作与实板历史记录见 [电机1测试](电机1测试.md)。本轮三键/电流新版已于 2026-11-16 21:09（北京时间）下载校验成功，`build/workflow/last-run.json` 中 `flashVerified=true`；非暂停读回四路 PWM=0、三键松开，PD7 命令高但实际读低，EN 外部下拉仍待排查。2026-11-16 16:19（北京时间）的旧版曾通过 J-Link 下载校验；以下为旧版实板记录：待机读回 READY、四路 PWM=0；PD7 命令高但引脚实际为低，外部下拉/保护状态待排查。OLED 显示效果及电机实际转动待现场观察；已验证 COM9 能收到 USART1 CSV；PA9 TX 到 J-Link RX、共地连接有效。

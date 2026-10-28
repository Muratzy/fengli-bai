# 风力摆 F407 固件

当前阶段使用天空星青春版 STM32F407VET6，在 STM32CubeIDE 1.19.0 中构建；FreeRTOS Kernel 版本为课程资料中的 202406.04 LTS。此阶段用于验证 OLED、ICM-45686、姿态显示和菜单，不运行风机闭环。

## 工程构建

在 CubeIDE 中选择 **File > Import > Existing Projects into Workspace**，导入 `WindPendulum_F407` 文件夹并构建 `Debug` 配置。`.ioc` 保存 CubeMX 引脚和外设配置。Debug/Release 均使用 Cortex-M4、FPv4-SP-D16 和 hard-float ABI；HAL/CMSIS 来源记录在 `Drivers/UPSTREAM_VERSIONS.txt`。

## 硬件接口

| 设备 | 接口配置 |
|---|---|
| OLED | SSD1306，128×64；I2C1 PB6/PB7，100 kHz，7 位地址 `0x3C`（HAL 地址 `0x78`） |
| ICM-45686 | SPI2 PB10/PC2/PC3，PE7 CS；SPI Mode 3，2.625 MHz；传感器量程 ±16 g、±2000 dps，ODR 800 Hz |
| 按键 | PC0 上、PC1 下、PC13 确认，低电平有效 |
| 调试串口 | USART1 PA9/PA10，115200 8-N-1；当前 USB-UART 枚举为 COM9 |
| 扩展串口 | USART2 PD5/PD6，115200 8-N-1 |

完整引脚及预留功能见 `硬件引脚分配.md`。SWD 保留 PA13/PA14，USB 和 TF 卡引脚不分配给本固件。

## 菜单和状态

根菜单包含 Live Status、IMU Data、Calibration、Display 和 About；IMU Data 下分 Attitude、Accelerometer、Gyroscope。Display 提供 OLED 对比度调节。状态页显示传感器在线状态、姿态角、加速度和陀螺仪原始值/换算值、温度及校准进度。

IMU 任务以 500 Hz 轮询数据。启动校准要求设备静止并连续采集 5000 个有效样本，约需 10 秒；期间检测到运动或加速度模长超限会重新累计。姿态阶段使用陀螺仪积分与加速度倾角进行互补融合，并在快速运动时降低加速度权重。

## FreeRTOS 与输出限制

任务链为 `imu -> fusion -> trajectory -> motor -> telemetry`，UI 通过队列读取显示状态。USART1 输出 CSV 遥测。

当前固件将 PD7 电机使能保持低电平，并在启动和 MotorTask 中持续将 TIM1 四路比较值设为零；PD8 激光也保持低电平。PID 参数、轨迹控制、风机混控、电流标定和故障闭环尚未启用，硬件未完成前不要解除这些限制。

## 下载与实测状态

调试探针使用 J-Link GDB Server、SWD、目标 `STM32F407VE`，建议初始速率 4000 kHz。CubeIDE 的 Debug Configuration 中选择 J-Link，不要使用课程模板里的 ST-LINK/F103 配置。编译成功不代表 OLED 或 IMU 已通过实物验证；上板结果应以 COM9 遥测和显示观察为准。

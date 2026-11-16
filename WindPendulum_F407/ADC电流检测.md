# ADC 电流检测与 OLED 显示

## 接线与换算参数

台架当前使用 PB0 / ADC1_IN8 / 主控 I1（H6 第 4 针）采集电机电流，假设飞线接电机板 H3 第 2 针 Vout。Vout 是 INA240 输出经过 1 kΩ、47 nF 滤波后的节点。此飞线还没有实测确认；若未接或接错，ADC 数值不代表电流。PB1 / ADC1_IN9 / U1 保留为中性的电压输入，不擅自改成电流输入。主控自身 INA181 的输出是 I2/PA2，不是 I1/PB0。

电机板 H3：第 1 针 Vi、第 2 针 Vout、第 3 针 +5V；第 3 针不是 GND。两板另行共地。ADC 输入必须处于 GND～VDDA；INA240 由 5 V 供电，输出可能超过 MCU ADC 允许范围，软件饱和标志不能替代硬件限压。

按项目电机板原理图：INA240A2PWR 增益 50 V/V，R23、R15 各 10 mΩ 并联得到 5 mΩ，REF 接地。该分流器测量电机板共用回流电流；仅电机1工作时用于观察其电流，不能将它描述成四路独立电流通道。

```text
Vout_V = ADC_RAW × VDDA_mV / 4095 / 1000
I_A = max(0, Vout_V - zero_V) / (50 × 0.005)
    = 4 × max(0, Vout_V - zero_V)
```

算法中使用微伏、微欧、毫安和 64 位中间量，避免浮点格式化开销及先四舍五入到毫伏引入的损失。零点以下钳位为零，适用于当前接地 REF 的单向测量。

## 代码与采样

- 纯算法：`Core/Inc/services/current_sense.h`、`Core/Src/services/current_sense.c`。
- 硬件与 FreeRTOS 适配：`Core/Src/diagnostics/motor1_test_stm32.c`。
- `g_current_config={50U,5000U,0U,1000U,3U}`：依次是放大倍数、采样电阻微欧、输出零点微伏、校正倍率千分比、滤波移位。

同一高优先级台架任务每 10 ms 读取 VREFINT、PB1 和 PB0，切换通道后丢弃一次转换，480 周期采样。ADC1 只有该任务操作；OLED/串口通过各自独立队列读取快照，不能重配 ADC 或写 PWM。电流 IIR 取 alpha=1/8，第一笔有效值初始化，失效样本不更新滤波器。它用于状态观察，没有实现快速过流关断。

ADC 的实际参考是外部 VDDA/3.3V_A。`app_config.h` 默认 `APP_ADC_VREFINT_CAL_RAW=0`，使用显式配置的标称 VREFINT 1.210 V 估计 VDDA，不声称已读取或核实本板出厂校准码。PB1 饱和或转换失败时，只要独立的 VREFINT 与 PB0 样本有效，电流仍可换算；VREFINT 无效或 VDDA 超出 1.8～3.6 V 时电流无效。

## OLED 与 UART

OLED 保持 100 ms 目标刷新，新行示例 `ADC:1241 I:4.000A`。ADC 是未滤波的最新 PB0 码，I 是滤波电流，两者的动态响应有意不同。ADC 转换失败显示 `ADC:ERR I:--`；参考无效显示 `I:--`；PB0 原始码达到 4090 时显示 `I:CLIP`，不显示貌似可信的安培值。ADC 成功并不证明飞线已连接。

CSV 保留 `current_adc` 和 `current_valid`（原始转换成功），另有 `current_ma`、`current_filtered_ma`、`current_calc_valid`、`current_overrange`、`reference_calibrated`。电流单位为 mA，计算值只在 `current_calc_valid=1` 时可用；未进行用户参考标定时 `reference_calibrated=0`。

## 校准与验证

1. 电机停止且模拟电源稳定时，万用表测 VDDA，同时读 VREFINT_RAW；设置 `APP_ADC_VREFINT_CAL_RAW=round(VREFINT_RAW×VDDA_mV/3300)`。
2. 确认无电流经过采样电阻后，测量输出零点电压，填写 `g_current_config.zero_uv`。当前默认 0，不在运行中自动扣除零点。
3. 在稳定且处于 ADC 量程内的已知电流点，对照万用表，设置 `gain_permille=round(I_reference/I_uncorrected×1000)`。使用零点修正后的电流作分母，至少另取一个电流点复核。

主机测试 `tests/test_current_sense.c` 覆盖约 1 A、4 A 换算、零点、增益、滤波、非法参数和无效样本。新版三键测试在 `tests/test_motor1_test.c`，电压测试在 `tests/test_voltage_sense.c`。本轮三组测试与 CubeIDE 构建均通过；已于 2026-11-16 21:09（北京时间）完成新版下载与 Flash 校验，尚未完成实板电流标定，显示小数位数不等于测量精度。

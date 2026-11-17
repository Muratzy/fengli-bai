# ADC 电流检测与 OLED 显示

2026-11-17：按用户要求只修正电流测量，参考 7Semi-INA240 加入 ADC 批次平均与显式零点标定接口。按键、PWM、EN 和电机控制策略保持原有行为；本轮不下载或启动电机。实测标定仍未完成。

## 参考仓库与适配范围

- 来源：[7Semi-INA240](https://github.com/7semi-solutions/7Semi-INA240)，MIT，版本 1.0.0。
- 本地完整克隆：`C:\Users\Muratsy\Desktop\7Semi-INA240`；核对提交：`08e91db372ada9359ad7998b05d5c53adf5a3fb9`。
- 参考入口：`src/7Semi_INA240.cpp` 的 `readAnalog`、`readShuntVoltage`、`readCurrent`、`calibrate`。本工程用纯 C、定点计算和既有 STM32 HAL 采样进行适配，不依赖 Arduino，不直接编译外部仓库。
- 原电流换算公式与参考库一致；本轮改善的是采样平均、避免先钳位造成零点附近偏高、显式标定和参数检查。参考示例里的 A3、10 mΩ、5 V ADC 参考及默认 `ADC参考/2` 零点都不适用于本板。来源与许可记录于 `ThirdPartyNotices.md`。

## 接线与换算参数

台架当前使用 PB0 / ADC1_IN8 / 主控 I1（H6 第 4 针）采集电机电流，假设飞线接电机板 H3 第 2 针 Vout。Vout 是 INA240 输出经过 1 kΩ、47 nF 滤波后的节点。此飞线还没有实测确认；若未接或接错，ADC 数值不代表电流。PB1 / ADC1_IN9 / U1 保留为中性的电压输入，不擅自改成电流输入。主控自身 INA181 的输出是 I2/PA2，不是 I1/PB0。

电机板 H3：第 1 针 Vi、第 2 针 Vout、第 3 针 +5V；第 3 针不是 GND。两板另行共地。ADC 输入必须处于 GND～VDDA；INA240 由 5 V 供电，输出可能超过 MCU ADC 允许范围，软件饱和标志不能替代硬件限压。

按项目电机板原理图：INA240A2PWR 增益 50 V/V，R23、R15 各 10 mΩ 并联得到 5 mΩ，REF 接地。该分流器测量电机板共用回流电流；仅电机1工作时用于观察其电流，不能将它描述成四路独立电流通道。

```text
Vout_V = sum(ADC_RAW[i]) × VDDA_mV / (N × 4095 × 1000)
I_A = max(0, Vout_V - zero_V) / (50 × 0.005)
    = 4 × max(0, Vout_V - zero_V)
```

算法中使用微伏、微欧、毫安和 64 位中间量，先对 ADC 批次求和，再直接换算平均微伏，保留分数 ADC 码，避免先截断平均码或四舍五入到毫伏造成损失。之后扣除输出零点、在零点以下钳位为零、换算为电流，再作 IIR 平滑。接地 REF 的硬件只按单向测量处理。

## 代码与采样

- 纯算法：`Core/Inc/services/current_sense.h`、`Core/Src/services/current_sense.c`。
- 硬件与 FreeRTOS 适配：`Core/Src/diagnostics/motor1_test_stm32.c`。
- `g_current_config`：固定放大倍数 50、采样电阻 5000 μΩ、滤波移位 3；零点和校正倍率从 `app_config.h` 读取。
- `APP_CURRENT_SAMPLE_COUNT=8`：每个 10 ms 周期保留 8 笔 PB0 转换；允许 1～250，不能照搬参考库默认的 100 笔到现有任务时序。
- `APP_CURRENT_ZERO_UV=0`：输出零点占位，单位 μV；`APP_CURRENT_GAIN_PERMILLE=1000`：比例校正。两者都未完成实物标定。

同一高优先级台架任务每 10 ms 读取 VREFINT、PB1 和 PB0，切换通道后丢弃一次转换，480 周期采样。VREFINT/PB1 各保留 1 笔，PB0 保留连续 8 笔；每周期共 13 次转换，由相邻 VREFINT 样本估计的 VDDA 换算整个短批次。ADC1 仍只有该任务操作；OLED/串口通过各自独立队列读取快照。没有复制参考库的 `delay` 或在任务外新增 ADC 写入者。

`CurrentSense_UpdateSamples` 完成批次检查和换算；`CurrentSense_Update` 保留为单样本兼容入口。电流 IIR 仍取 alpha=1/8，首笔有效批次初始化。任一 PB0 原始码达到 4090、转换失败、参考无效、参数非法或计算溢出时，本批电流无效，旧滤波状态不更新；不能用其余样本平均掉一次饱和。`CurrentSense_Result` 另保留批次大小、Q8 平均码和平均微伏供代码/调试器检查，不扩展串口字段。

这仍是状态观察用的批次平均和滤波，不是电流闭环或快速过流关断。采样未与 PWM 相位同步，改动后的控制周期抖动及 PWM 纹波影响需在另行授权的上板测试中验证。

ADC 的实际参考是外部 VDDA/3.3V_A。`app_config.h` 默认 `APP_ADC_VREFINT_CAL_RAW=0`，使用显式配置的标称 VREFINT 1.210 V 估计 VDDA，不声称已读取或核实本板出厂校准码。PB1 饱和或转换失败时，只要独立的 VREFINT 与 PB0 样本有效，电流仍可换算；VREFINT 无效或 VDDA 超出 1.8～3.6 V 时电流无效。

## OLED 与 UART

OLED 保持 100 ms 目标刷新，例如 `ADC:1241 I:4.000A`。ADC 是本批最后一笔 PB0 原始码，I 是批次平均并经 IIR 平滑后的电流，二者不要求逐笔换算一致。ADC 转换失败显示 `ADC:ERR I:--`；参考无效显示 `I:--`；批次中任一码达到 4090 时显示 `I:CLIP`，即使最后一笔原始码未饱和。ADC 成功并不证明飞线已连接。

CSV 仍为 28 列，保留 `current_adc`（本批最后一笔原始码）和 `current_valid`（本批 PB0 采集全部成功），`current_ma` 为批次平均/零点/增益校正后、尚未 IIR 平滑的电流，`current_filtered_ma` 为其 IIR 值。电流单位为 mA，须结合 `current_calc_valid` 和 `current_overrange`；`reference_calibrated` 仅表示已配置 VDDA 的用户参考码，不表示零点、分流器或整条电流链已标定。旧单样本固件可以视为 N=1，列顺序、单位和主机解析仍兼容。

## 校准与验证

1. 电机停止且模拟电源稳定时，万用表测 VDDA，同时读 VREFINT_RAW；设置 `APP_ADC_VREFINT_CAL_RAW=round(VREFINT_RAW×VDDA_mV/3300)`。
2. 确认无电流经过共用分流器后，采集稳定零点样本并求平均输出微伏，填写 `APP_CURRENT_ZERO_UV`。不能把 PWM=0、READY 或按下停止键当作零电流证明，不自动从历史待机日志扣除板级真实电流。
3. 已提供纯算法接口 `CurrentSense_CalibrateZero(state, config, samples, count, vdda_mv, samples_valid, zero_current_confirmed)`。两个有效/确认参数都必须为真；样本数为 1～250。仅成功时更新调用者的 `config.zero_uv` 并重置 IIR，失败保留原配置和滤波状态。它不会自行采样、写 Flash 或进入菜单，也未绑定按键/启动流程。若以后接入运行时标定，应由唯一采样任务在用户明确零电流确认后调用。
4. 在稳定且处于 ADC 量程内的已知电流点，对照万用表，以 `APP_CURRENT_GAIN_PERMILLE=1000` 时、扣零点后的电流为基准，设置 `APP_CURRENT_GAIN_PERMILLE=round(I_reference/I_uncorrected×1000)`；至少另取一个电流点复核。

本轮主机测试在 `tests/test_current_sense.c`，覆盖约 1 A/4 A、零点/增益/IIR、分数 ADC 平均、VDDA 变化、零点附近先平均再钳位、250 个位置逐笔饱和检测、无效批次不更新、显式标定成功重置/失败保留及数值溢出。主工程目录可复现：

```powershell
gcc -std=c11 -Wall -Wextra -Wconversion -Werror -I Core/Inc tests/test_current_sense.c Core/Src/services/current_sense.c -o build/test_current_sense.exe
.\build\test_current_sense.exe
```

执行前确保忽略的 `build/` 目录存在；编译失败时不能运行旧测试程序。固件验证只运行 `Tools/build_flash.ps1 -Action Build`，不会执行 `BuildFlash`。本轮构建/测试结果见根目录 `项目交接.md`。2026-11-16 21:09 的下载记录属于旧单样本固件；本轮算法尚未下载，未完成实板电流标定，显示小数位数不等于测量精度。

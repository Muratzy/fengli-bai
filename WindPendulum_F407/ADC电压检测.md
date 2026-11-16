# ADC 电压检测

## 通道与硬件边界

主控板 PB1 对应 ADC1_IN9，网络名 U1，接到 H6 第 3 路模拟量。主控原理图没有画出 U1 前端的电阻分压器，因此当前固件把 ADC 读数解释为 **PB1/U1 引脚电压**。现有参数为 1:1，输出字段 `u1_input_mv` 与 `u1_pin_mv` 相同；这只是占位配置，不代表已标定出母线电压。

附图中的 INA240A2 是电机板电流检测链路：两个 10 mΩ 采样电阻并联，等效 5 mΩ，INA240 增益为 50 V/V。它不能用于 PB1/U1 的电压换算。主控原理图上的 INA181 输出为 I2/PA2，并非本项目用于电机电流输入的 I1/PB0。

两张原理图还显示电机板 H3 有 `Vi`、`Vout`、`+5V`（H3 第 3 针不是 GND），其中 `Vout` 位于 INA240 输出链路；主控 H6 的 `U1` 需要按实际飞线确认是否接到了这个 `Vout`。若 H6-U1 实际接的是 H3-Vout，PB1 测到的是电流检测放大器输出电压，后续应按 INA240 增益和采样电阻换算电流，不能称为母线电压。当前模块保留 `u1_pin_mv` 这一中性名称，避免混淆。

PB1 是 MCU ADC 输入，外部输入必须限制在 GND 到 VDDA 范围内。当前 U1 未确认有分压保护，禁止将高于 MCU 允许范围的母线电压直接接入 PB1。接入母线前，先根据实际电路核实分压与保护器件，并确认 PB1 满量程不会超过 VDDA。

## 换算流程

每次采样依次完成：读取内部 VREFINT、读取 PB1/ADC1_IN9、检查参考值及 ADC 是否过量程、计算引脚电压、应用分压与校准参数，最后更新一阶低通滤波值。CubeMX 默认配置为 ADC1 单通道、PB1/ADC1_IN9、480 周期；应用代码仍以单次转换方式动态选择 ADC1_IN8、ADC1_IN9 或 VREFINT，不启用扫描/DMA。

STM32F407 的 ADC 参考是 MCU 的 VDDA（本板模拟电源为 3.3V_A），不是 ADC 内部产生的 3.3 V。固件采用显式配置的参考值，避免依赖尚未在本板核实的出厂校准地址。`app_config.h` 的 `APP_ADC_VREFINT_CAL_RAW=0` 时，按配置的标称 `APP_ADC_VREFINT_NOMINAL_MV=1210` 得到等效 3.3 V 参考码，仅为未校准估计。设置用户实测参考码后使用用户标定；ADC 实际参考仍为外部 VDDA。计算为：

```text
VDDA_mV = CONFIGURED_VREFINT_RAW_AT_3300 × 3300 / VREFINT_RAW
U1_pin_mV = ADC_RAW × VDDA_mV / 4095
U1_scaled_mV = U1_pin_mV × divider_numerator / divider_denominator
U1_corrected_mV = max(0, U1_scaled_mV - offset_mV) × gain_permille / 1000
```

若外部为电阻分压，`divider_numerator / divider_denominator = (R_TOP + R_BOTTOM) / R_BOTTOM`。如果实物没有分压，不能为了显示“母线电压”随意填写比例。

每 10 ms 采样一次 PB1 和 VREFINT。切换 ADC 通道后各丢弃一次转换，再取有效值；采样时间设为 480 个 ADC 时钟周期，以适应可能较高的源阻抗。OLED 和串口快照仍按约 100 ms 更新。滤波为 `alpha=1/8` 的一阶 IIR：

```text
y[n] = y[n-1] + (x[n] - y[n-1]) / 8
```

第一笔有效样本直接初始化滤波器。ADC 原始码达到 4090、VREFINT/配置参考码无效，或估算 VDDA 超出 1.8–3.6 V 时，样本标记无效且不更新滤波器。滤波结果用于显示和慢速遥测，不应替代示波器对 PWM 瞬态、纹波或快速跌落的测量。

## 校准

用户参考标定：电机停止、模拟电源稳定时，万用表测 VDDA_mV，同时读 VREFINT_RAW；填写 `APP_ADC_VREFINT_CAL_RAW = round(VREFINT_RAW × VDDA_mV / 3300)`。未标定时 CSV 的 `reference_calibrated=0`，转换成功与校准精度是不同概念。配置与算法位于 `Core/Src/diagnostics/motor1_test_stm32.c` 和 `Core/Src/services/voltage_sense.c`。目前占位配置为：

```c
{1U, 1U, 1000U, 0, 3U}
```

五项依次是分压倍率分子、分母、增益（千分比）、偏置（mV）和滤波右移位数。硬件确认后，先按电阻值填写分压倍率，再用至少两个安全且稳定的电压点与万用表对照拟合增益和偏置。偏置按 `U1_scaled_mV - offset_mV` 的方向参与计算。校准时 PB1 实际输入仍必须低于 VDDA；不要通过高压直接测试 ADC。

当前没有配置欠压/过压门限，因为 U1 的真实测量对象、分压网络和供电范围尚未确认。确定这些硬件参数后再添加阈值及故障动作。

## 运行与扩展

当前默认固件是电机1台架测试模式。USART1 CSV 新增 `voltage_adc`、`vrefint_adc`、`vdda_mv`、`u1_pin_mv`、`u1_input_mv`、`u1_filtered_mv` 和 `voltage_valid`；OLED 第 4 行同时显示 CCR1 和滤波后的 U1 电压。PB1 原始码通过串口 CSV 查看。`voltage_valid=0` 时不应使用该笔电压值。

题目控制任务启用后，应由统一 ADC 采样服务提供 PB1 电压快照，再经遥测队列送到题目任务、日志与 OLED。不要让多个任务同时重配并读取 ADC1。若要分析电机 PWM 引起的快速电源扰动，需要另行使用定时器触发 ADC + DMA，并依据模拟前端带宽、PWM 频率和目标瞬态宽度选择采样率；目前 100 Hz 的 PB1 轮询仅供慢速状态观察。

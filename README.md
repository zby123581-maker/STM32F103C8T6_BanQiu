# STM32F103C8T6_BanQiu

基于视觉坐标反馈的板球平衡控制工程。控制器接收上位视觉模块发送的小球坐标，通过串级位置/速度 PD 控制双轴舵机，使小球向标定中心移动。

当前工程的实际目标芯片是 **STM32F103C8**。代码使用 STM32F103 HAL，系统时钟为 72 MHz。

## 功能

- USART1 + DMA 空闲帧接收视觉坐标，115200 baud，8N1
- 像素坐标转换为以平台中心为零点的归一化误差
- 外环位置 PD 每 20 ms 计算目标速度
- 内环速度 PD 每 10 ms 调节舵机角度
- TIM3 CH1/CH2（PA6/PA7）输出双路舵机 PWM
- USART2 以 115200 baud 输出误差和舵机角度调试数据

## 坐标数据帧

视觉模块通过 USART1 发送固定 7 字节二进制帧：

| 字节 | 内容 |
| --- | --- |
| 0 | 帧头 `0xAA` |
| 1 | 帧头 `0x55` |
| 2 | X 坐标高字节 |
| 3 | X 坐标低字节 |
| 4 | Y 坐标高字节 |
| 5 | Y 坐标低字节 |
| 6 | 校验和 |

X、Y 按有符号 16 位大端整数解析。校验和为前 6 个字节之和的低 8 位：

```text
checksum = (0xAA + 0x55 + X_H + X_L + Y_H + Y_L) & 0xFF
```

当前相机标定中心为 `(374, 254)`，X/Y 归一化尺度分别为 `194.25` 和 `206.5`。更换相机位置或画面分辨率后，应在 `Core/Src/main.c` 中重新标定这些参数。

## 开发环境

- STM32CubeMX（打开 `duoji.ioc`）
- Keil MDK-ARM / uVision（打开 `MDK-ARM/duoji.uvprojx`）
- STM32F1 HAL 与 CMSIS（已包含在 `Drivers` 中）

## 使用

1. 用 STM32CubeMX 打开 `duoji.ioc`，确认目标器件、时钟和引脚配置。
2. 用 Keil uVision 打开 `MDK-ARM/duoji.uvprojx`。
3. 编译并下载到 STM32F103C8。
4. 将视觉模块串口连接到 USART1：PA10 接收坐标，PA9 为发送端。
5. 将两路舵机信号线连接到 PA6 和 PA7，并确保舵机使用独立、共地且容量足够的电源。
6. 根据机械结构调整 `offset_x`、`offset_y` 和两级 PD 参数。

控制算法和参数说明见 [串级PID说明.md](./串级PID说明.md)。

## 许可

本项目自行编写的代码和文档采用 MIT License。`Drivers/` 下的 STM32 HAL、CMSIS 及其他第三方文件保留各自原有许可证，详见对应目录中的 `LICENSE.txt`。


# MSPM0G3507 Artemis 小车控制固件

本工程把 `artemis-vicon` 的 task3 控制逻辑移植到 MSPM0G3507。单片机只负责解析仿真观测、运行航向/循迹控制和任务状态机，再通过串口把左右后轮目标速度返回给 `artemis-viconjs`；工程不包含真实电机、编码器或巡线传感器驱动。

## 硬件连接

串口使用 UART0，参数为 115200、8 数据位、1 停止位、无校验、无流控。

| MSPM0G3507 | 外接 USB-TTL | 说明 |
| --- | --- | --- |
| PA10 / UART0 TX | RX | 单片机发送命令 |
| PA11 / UART0 RX | TX | 单片机接收仿真观测 |
| GND | GND | 必须共地 |

USB-TTL 必须使用 3.3V TTL 电平，不要把 RS-232 或 5V 信号直接接入芯片。使用 LP-MSPM0G3507 BoosterPack 引脚时，PA10 位于 `J4_34`，PA11 位于 `J4_33`；按开发板跳线说明把 J21/J22 切换到外部接口，避免与 XDS-110 回传串口同时驱动。

PA14 为高电平点亮的黑线状态指示灯。推荐接法为 `PA14 -> 330R 电阻 -> LED -> GND`。巡线数据连续两帧确认接触或离开黑线后，LED 点亮 500ms；重叠事件之间保留 100ms 熄灭间隔。

## 运行流程

1. 启动 `artemis-mudri`。
2. 启动提交 `2ababdd` 或更新版本的 `artemis-viconjs`，选择 USB-TTL 对应的 COM 口和 115200 波特率。
3. 给 MSPM0G3507 上电或复位。
4. 固件自动发送 `START`，随后按 `STARTED -> STEP/OBS -> STOP -> FINISHED` 推进 task3。

等待桥接响应超过 6 秒，或收到 `ERR` 时，固件会重置控制器并自动重启仿真会话。任务完成后固件停在结束状态，下一轮需要复位单片机。

## 控制配置

公共参数位于 `artemis_config.h`。默认使用与 Python 配置一致的普通循迹 PID：

```c
#define ARTEMIS_LINE_PID_MODE ARTEMIS_LINE_PID_ORIGINAL
```

切换到模糊 PID：

```c
#define ARTEMIS_LINE_PID_MODE ARTEMIS_LINE_PID_FUZZY
```

编码器累计 tick 按 Mudri 默认标定 `10.62 tick/cm` 转换为前进距离。

## 构建与测试

Keil 工程位于 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`，使用 MSPM0 SDK `2.04.00.06` 和设备包 `TexasInstruments.MSPM0G1X0X_G3X0X_DFP 1.3.1`。构建前命令会调用 `keil/syscfg_local.bat` 重新生成 UART/GPIO 配置。

`keil/flash_padding.s` 与 scatter 文件中的 `ER_FLASH_PADDING` 用于保证 AXF 主加载区始终按 8 字节结束，这是 MSPM0 Flash Loader 进行 64 位写入所必需的；不要从 Keil 工程中删除该文件。

硬件无关逻辑可以使用 MinGW 在 PowerShell 中测试：

```powershell
.\tests\run_host_tests.ps1
```

测试覆盖串口协议、普通/模糊 PID、task3 完整动作切换、Python 黄金输出和 PA14 LED 边沿时序。

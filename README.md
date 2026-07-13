# MSPM0G3507 Artemis 小车仿真控制固件

本工程把 `artemis-vicon` 的小车控制逻辑移植到 MSPM0G3507。单片机只负责控制逻辑运算：接收桥接软件发来的仿真观测，计算左右轮目标速度，再通过串口回传给 `artemis-viconjs`。工程不包含真实电机、编码器或巡线传感器驱动。

## 硬件连接

串口使用 UART0，参数为 `115200 8N1`，无校验、无流控。

| MSPM0G3507 | 外接 USB-TTL | 说明 |
| --- | --- | --- |
| PA10 / UART0 TX | RX | 单片机发送 `START` / `STEP` / `STOP` |
| PA11 / UART0 RX | TX | 单片机接收 `STARTED` / `OBS` / `FINISHED` / `ERR` / `PARAM` |
| GND | GND | 必须共地 |

USB-TTL 必须使用 3.3V TTL 电平，不要把 RS-232 或 5V 信号直接接入芯片。使用 LP-MSPM0G3507 BoosterPack 引脚时，PA10 位于 `J4_34`，PA11 位于 `J4_33`；按开发板跳线说明把 J21/J22 切换到外部接口，避免和 XDS-110 回传串口同时驱动。

PA14 为高电平点亮的黑线状态指示灯。推荐接法为：

```text
PA14 -> 330R 电阻 -> LED -> GND
```

巡线数据连续确认接触或离开黑线后，LED 点亮 500ms；重叠事件会排队，两次闪烁之间保留 100ms 熄灭间隔。

## 运行流程

1. 启动 `artemis-mudri`。
2. 启动支持串口桥接的 `artemis-viconjs`，选择 USB-TTL 对应的 COM 口和 `115200` 波特率。
3. 给 MSPM0G3507 上电或复位。
4. 在 OLED 上选择任务，KEY1 启动。
5. 固件发送 `START`，随后按 `STARTED -> STEP/OBS -> STOP -> FINISHED` 推进任务。

握手阶段等待 `STARTED` 超时后，固件会重新发送 `START`。一旦握手成功进入正式任务，固件只在收到一帧 `OBS` 后发送下一条 `STEP`，避免任务运行中反复重启仿真。

## 控制配置

公共参数位于 [artemis_config.h](./artemis_config.h)。默认启用模糊巡线 PID：

```c
#define ARTEMIS_LINE_PID_MODE ARTEMIS_LINE_PID_FUZZY
```

如需切换到普通 PID：

```c
#define ARTEMIS_LINE_PID_MODE ARTEMIS_LINE_PID_ORIGINAL
```

编码器累计 tick 按 Mudri 默认标定 `10.62 tick/cm` 换算为前进距离。

## AI 调参工作流

本工程支持“不重新烧录的临时调参”：AI 或调参脚本通过桥接软件的本地 ZMQ `REQ/REP` 接口发送 `PARAM` 命令，桥接软件再把命令写入它独占的串口。MSPM0G3507 收到 `PARAM` 后只修改 RAM 中的运行时参数，不写 Flash；复位或重新烧录后会恢复 [artemis_config.h](./artemis_config.h) 中的默认值。

推荐闭环如下：

```text
AI / Codex  SUB <- tcp://192.168.1.24:5555  Mudri viewer-state
AI / Codex  REQ -> tcp://127.0.0.1:5560     Bridge AI command
Bridge      UART -> MSPM0G3507              PARAM / PARAM_RESET
MSPM0G3507  UART -> Bridge                  START / STEP / STOP
```

### 桥接软件设置

桥接软件正常连接串口后，确认以下端口可用：

| 用途 | 默认端点 | 说明 |
| --- | --- | --- |
| Mudri 控制端口 | `tcp://192.168.1.24:5556` | 桥接软件和仿真服务之间的请求/响应通道 |
| Viewer 状态流 | `tcp://192.168.1.24:5555` | AI 订阅车辆位置、姿态、速度等实时状态 |
| AI 命令端口 | `tcp://127.0.0.1:5560` | AI 通过桥接软件向单片机发送低频参数命令 |

AI 命令端口可以通过环境变量覆盖：

```dotenv
ARTEMIS_AI_COMMAND_ENDPOINT=tcp://127.0.0.1:5560
```

注意：桥接软件运行时会独占串口。AI 调参脚本不要直接打开同一个 COM 口，而是通过 `tcp://127.0.0.1:5560` 请求桥接软件转发参数。

### 健康检查

AI 命令口使用 ZMQ `REQ/REP`，请求内容是 UTF-8 JSON。

```json
{"type":"health","request_id":"check-1"}
```

正常返回示例：

```json
{"type":"ok","request_id":"check-1","bridgeRunning":true,"serialOpen":true,"lastError":""}
```

`serialOpen` 为 `true` 只表示桥接软件打开了串口；参数是否已被单片机应用，需要结合轨迹变化或后续固件 ACK 机制判断。

### 发送参数

桥接软件只接受 `PARAM ...` 和 `PARAM_RESET` 两类串口命令，拒绝 `START`、`STEP`、`STOP`、多行内容、非 ASCII 内容和过长命令。

设置单个参数：

```json
{"type":"board_param","line":"PARAM yaw_kp=0.50","request_id":"set-yaw-kp"}
```

重置所有运行时参数为固件默认值：

```json
{"type":"board_param_reset","line":"PARAM_RESET","request_id":"reset-params"}
```

桥接成功写入串口后的返回示例：

```json
{"type":"ok","request_id":"set-yaw-kp","written":"PARAM yaw_kp=0.50"}
```

### 可调参数

固件当前支持的运行时参数由 [artemis_runtime_params.c](./artemis_runtime_params.c) 中的表决定：

| 参数名 | 作用 |
| --- | --- |
| `line_mode` | `0` 为普通 PID，`1` 为模糊 PID |
| `yaw_kp` / `yaw_ki` / `yaw_kd` | 航向 PID 参数 |
| `yaw_filter_previous` / `yaw_filter_current` | 航向误差滤波权重，通常两者相加为 1 |
| `yaw_turn_scale` | 航向输出缩放，越大实际转向越弱 |
| `yaw_stable_error_deg` | 原地对准时认为航向稳定的角度误差阈值 |
| `line_kp` / `line_ki` / `line_kd` | 普通巡线 PID 参数 |
| `line_scale` | 巡线输出缩放，越大实际转向越弱 |
| `fuzzy_error_normalizer` | 模糊 PID 误差归一化分母 |
| `fuzzy_delta_normalizer` | 模糊 PID 误差变化量归一化分母 |
| `fuzzy_center_width` / `fuzzy_steady_width` | 模糊 PID 中 Ki 参与条件的宽度 |
| `fuzzy_kp_base` / `fuzzy_kp_error_gain` / `fuzzy_kp_delta_gain` | 模糊 PID 的 Kp 基础值和动态增益 |
| `fuzzy_kd_base` / `fuzzy_kd_error_gain` / `fuzzy_kd_delta_gain` | 模糊 PID 的 Kd 基础值和动态增益 |
| `fuzzy_ki_base` / `fuzzy_integral_limit` | 模糊 PID 的 Ki 基础值和积分限幅 |
| `drive_velocity` | 找线直行速度 |
| `track_velocity` | 巡线速度 |
| `entry_c_yaw_deg` / `entry_d_yaw_deg` | task3 两段找线航向 |
| `turn_settle_s` | task3 兼容用转向稳定时间 |
| `drive_max_s` / `track_max_s` | 找线和巡线动作最大持续时间 |
| `task2_ab_yaw_deg` / `task2_cd_yaw_deg` | task2 两段直线航向 |
| `task2_straight_min_cm` / `task2_arc_min_cm` | task2 直线/弧线阶段最小确认距离 |

### Codex 调参建议

AI 调参时建议按“观察 -> 小步修改 -> 再观察”的方式进行：

1. 订阅 `viewer-state`，记录 `pose.x_m`、`pose.y_m`、`pose.yaw_rad`、`kinematics.longitudinal_velocity_m_s`、`kinematics.yaw_rate_rad_s`。
2. 观察轨迹是否在关键点下沉、上抬、过冲或出线。
3. 通过 AI 命令口发送一组小幅 `PARAM` 调整。
4. 让小车至少跑过一圈，再决定是否继续调整。
5. 找到稳定参数后，把参数写回 [artemis_config.h](./artemis_config.h)，重新编译并烧录，完成固化。

仓库提供了一个基础日志工具：

```powershell
python .\tools\artemis_tuning_logger.py `
  --viewer tcp://192.168.1.24:5555 `
  --ai-command tcp://127.0.0.1:5560 `
  --output .\logs\artemis_tuning_params.jsonl `
  --interval-s 30
```

该工具会周期性记录桥接健康状态、最新 viewer-state 姿态和当前候选参数，便于回看不同参数组合下的轨迹表现。

手动发送参数的 Python 示例：

```python
import json
import zmq

ctx = zmq.Context.instance()
sock = ctx.socket(zmq.REQ)
sock.connect("tcp://127.0.0.1:5560")

sock.send_string(json.dumps({
    "type": "board_param",
    "line": "PARAM track_velocity=4.6",
    "request_id": "track-velocity-test",
}))
print(sock.recv_string())
```

一组常用起点参数：

```text
PARAM line_mode=1
PARAM drive_velocity=4.9
PARAM track_velocity=4.8
PARAM yaw_kp=0.50
PARAM yaw_ki=0.0
PARAM yaw_kd=0.04
PARAM yaw_turn_scale=4.4
PARAM line_kp=25.0
PARAM line_ki=0.0
PARAM line_kd=3.5
PARAM task2_ab_yaw_deg=0
PARAM task2_cd_yaw_deg=180
PARAM task2_straight_min_cm=82
PARAM task2_arc_min_cm=120
```

## 构建与测试

Keil 工程位于：

```text
keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx
```

工程使用 MSPM0 SDK `2.04.00.06` 和设备包 `TexasInstruments.MSPM0G1X0X_G3X0X_DFP 1.3.1`。构建前命令会调用 [keil/syscfg_local.bat](./keil/syscfg_local.bat) 重新生成 UART/GPIO 配置。

[keil/flash_padding.s](./keil/flash_padding.s) 用于保证 AXF 主加载区按 MSPM0 Flash Loader 需要的 8 字节边界结束，不要从 Keil 工程中删除该文件。

硬件无关逻辑可以用 MinGW 在 PowerShell 中测试：

```powershell
.\tests\run_host_tests.ps1
```

测试覆盖串口协议、运行时参数、普通/模糊 PID、任务动作切换、Python 对照输出和 PA14 LED 边沿时序。

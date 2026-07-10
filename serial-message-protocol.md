# 串口消息收发格式结构说明

本文档说明 `artemis-viconjs` 当前实现中的 MCU 串口协议格式。依据代码：


## 1. 通信模型

`artemis-viconjs` 是 MCU 和 `artemis-mudri` 之间的桥接程序。

```text
MCU <--- ASCII line serial ---> artemis-viconjs <--- ZMQ JSON ---> artemis-mudri
```

串口方向分为两类：

| 方向 | 发送方 | 接收方 | 内容 |
| --- | --- | --- | --- |
| RX | MCU | 桥接程序 | `START` / `STEP` / `STOP` 控制命令 |
| TX | 桥接程序 | MCU | `STARTED` / `OBS` / `FINISHED` / `ERR` 响应 |

桥接程序收到 MCU 的一行命令后，会解析命令，转换为 Mudri ZMQ JSON 请求，等待 Mudri 返回结果，然后再把结果格式化为一行串口响应写回 MCU。

## 2. 串口帧基础格式

### 2.1 编码和换行

当前实现使用 `serialport` 的 `ReadlineParser` 按行读取：

```text
delimiter: "\n"
encoding: "ascii"
```

因此每条串口消息是一行 ASCII 文本：

```text
<COMMAND> <arg1> <arg2> ...\n
```

发送端建议使用 LF 结尾：

```text
\n
```

桥接程序在处理命令前会去掉末尾的 `\n` 或 `\r\n`，并对整行执行 `trim()`。

### 2.2 分隔规则

一行消息由空白字符分隔为 token：

```text
STEP sequence_id=1 left=7.0 right=7.0
```

会被解析为：

```text
["STEP", "sequence_id=1", "left=7.0", "right=7.0"]
```

解析器支持单引号和双引号，也支持反斜杠转义。通常 MCU 固件侧不需要使用引号，保持无空格 token 最稳妥。

示例：

```text
STOP reason="manual stop"
```

会解析出 `reason=manual stop`。但响应中的 reason/message 会经过安全化处理，空白会被替换为下划线。

### 2.3 key=value 参数

多数命令参数使用 `key=value` 形式：

```text
START max_time_s=120 control_period_s=0.02 initial_progress_index=0 random_seed=1
```

规则：

| 规则 | 说明 |
| --- | --- |
| key 不能为空 | `=123` 会报错 |
| value 可以为空字符串 | 但数值字段为空会报类型错误 |
| 非位置参数必须写成 `key=value` | 例如 `START 120` 会报错 |
| 未识别 key 会被忽略 | 解析器只读取当前命令需要的 key |

## 3. MCU 发给桥接程序的命令

当前支持 3 类命令：

| 命令 | 别名 | 作用 |
| --- | --- | --- |
| `START` | `S` | 启动一次 Mudri 仿真/控制会话，并返回初始观测 |
| `STEP` | `T` | 发送一帧左右后轮目标速度，并返回下一帧观测或完成结果 |
| `STOP` | `X` | 停止当前会话，并返回完成结果 |

命令名大小写不敏感。`start`、`START`、`S` 都可以被识别。

### 3.1 START / S

格式：

```text
START [max_time_s=<float|null>] [control_period_s=<float|null>] [initial_progress_index=<int|null>] [random_seed=<int|null>]
```

短别名：

```text
S max_time_s=120 control_period_s=0.02 initial_progress_index=0 random_seed=1
```

字段说明：

| 字段 | 类型 | 默认值 | 是否必填 | 含义 |
| --- | --- | --- | --- | --- |
| `max_time_s` | float 或 `null` | `null` | 否 | Mudri 本次会话最大仿真时间，单位秒。为 `null` 或省略时不向 Mudri 请求里写入该字段。 |
| `control_period_s` | float 或 `null` | `0.02` | 否 | 控制周期，单位秒。省略时使用 `0.02`。为 `null` 时不向 Mudri 请求里写入该字段。 |
| `initial_progress_index` | int 或 `null` | `0` | 否 | 初始进度索引，用于指定路径/任务进度起点。省略时为 `0`。当前代码允许 `null`，但发送到 Mudri 时仍会带上该字段。 |
| `random_seed` | int 或 `null` | `null` | 否 | 随机种子。省略或 `null` 时不向 Mudri 请求里写入该字段。 |

示例 1：使用默认控制周期启动

```text
START
```

桥接到 Mudri 的 JSON 请求：

```json
{
  "type": "start",
  "control_period_s": 0.02,
  "initial_progress_index": 0
}
```

示例 2：指定完整启动参数

```text
START max_time_s=120 control_period_s=0.02 initial_progress_index=0 random_seed=1
```

桥接到 Mudri 的 JSON 请求：

```json
{
  "type": "start",
  "max_time_s": 120,
  "control_period_s": 0.02,
  "initial_progress_index": 0,
  "random_seed": 1
}
```

示例 3：使用短命令

```text
S max_time_s=60 random_seed=42
```

### 3.2 STEP / T

`STEP` 支持两种格式：位置参数格式和 `key=value` 格式。

#### 3.2.1 位置参数格式

格式：

```text
STEP <sequence_id> <rear_left_target_speed> <rear_right_target_speed>
```

示例：

```text
STEP 1 7.0 7.0
```

字段说明：

| 字段 | 类型 | 是否必填 | 含义 |
| --- | --- | --- | --- |
| `sequence_id` | int | 是 | MCU 发出的控制帧序号。用于让请求、响应和日志对应起来。 |
| `rear_left_target_speed` | float | 是 | 左后轮目标速度，字段名沿用 Mudri 接口。单位由 Mudri/底层模型定义。 |
| `rear_right_target_speed` | float | 是 | 右后轮目标速度，字段名沿用 Mudri 接口。单位由 Mudri/底层模型定义。 |

桥接到 Mudri 的 JSON 请求：

```json
{
  "type": "step",
  "sequence_id": 1,
  "rear_left_target_speed": 7,
  "rear_right_target_speed": 7
}
```

#### 3.2.2 key=value 格式

完整格式：

```text
STEP sequence_id=<int> rear_left_target_speed=<float> rear_right_target_speed=<float>
```

示例：

```text
STEP sequence_id=1 rear_left_target_speed=7.0 rear_right_target_speed=7.0
```

也可以使用别名：

```text
STEP seq=1 left=7.0 right=7.0
```

字段和别名：

| 标准字段 | 可用别名 | 类型 | 是否必填 | 含义 |
| --- | --- | --- | --- | --- |
| `sequence_id` | `seq` | int | 是 | 控制帧序号。 |
| `rear_left_target_speed` | `left`, `l` | float | 是 | 左后轮目标速度。 |
| `rear_right_target_speed` | `right`, `r` | float | 是 | 右后轮目标速度。 |

短命令示例：

```text
T seq=2 l=6.5 r=6.8
```

### 3.3 STOP / X

格式：

```text
STOP [reason]
```

或：

```text
STOP reason=<reason>
```

短别名：

```text
X reason=mcu_stop
```

字段说明：

| 字段 | 类型 | 默认值 | 是否必填 | 含义 |
| --- | --- | --- | --- | --- |
| `reason` | string | `mcu_stop` | 否 | 停止原因，会透传给 Mudri 的 stop 请求。 |

解析规则：

| 输入 | 解析出的 reason |
| --- | --- |
| `STOP` | `mcu_stop` |
| `STOP reason=user_stop` | `user_stop` |
| `STOP emergency stop` | `emergency_stop` |
| `X` | `mcu_stop` |

示例：

```text
STOP reason=button_pressed
```

桥接到 Mudri 的 JSON 请求：

```json
{
  "type": "stop",
  "reason": "button_pressed"
}
```

## 4. 桥接程序返回给 MCU 的响应

当前支持 4 类响应：

| 响应 | 触发条件 | 含义 |
| --- | --- | --- |
| `STARTED` | `START` 成功 | Mudri 会话已启动，并携带初始观测 |
| `OBS` | `STEP` 成功且会话未结束 | 返回一帧观测数据 |
| `FINISHED` | `STEP` 后会话结束，或 `STOP` 成功 | 返回结束原因和摘要 |
| `ERR` | 命令解析失败、Mudri 返回错误、ZMQ 超时等 | 返回错误信息 |

所有响应都是一行 ASCII 文本，并以 `\n` 结尾。

### 4.1 STARTED

格式：

```text
STARTED seq=<number> t=<number> yaw=<number> dig=<bits> enc_tl=<number> enc_tr=<number>
```

示例：

```text
STARTED seq=0 t=0 yaw=0 dig=0011100 enc_tl=0 enc_tr=0
```

字段说明：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `seq` | number | Mudri 返回的观测帧序号，对应 observation 的 `sequence_id`。 |
| `t` | number | 仿真时间，单位秒，对应 observation 的 `sim_time_s`。 |
| `yaw` | number | 航向角，单位度，对应 observation 的 `imu.yaw_deg`。 |
| `dig` | string | 线传感器数字值拼接成的位串。例如 `0011100`。每一位为 `0` 或 `1`。 |
| `enc_tl` | number | 左后轮累计编码器 tick，对应 observation 的 `encoder.rear_left_total_ticks`。 |
| `enc_tr` | number | 右后轮累计编码器 tick，对应 observation 的 `encoder.rear_right_total_ticks`。 |

注意：README 旧示例中出现过 `time_limit_s`、`control_period_s`、`distance_cm`、`digital` 等字段；当前 `serialProtocol.js` 的实际串口输出字段是上表中的 `seq/t/yaw/dig/enc_tl/enc_tr`。

### 4.2 OBS

格式：

```text
OBS seq=<number> t=<number> yaw=<number> dig=<bits> enc_tl=<number> enc_tr=<number>
```

示例：

```text
OBS seq=1 t=0.02 yaw=0.15 dig=0011100 enc_tl=12 enc_tr=12
```

字段含义与 `STARTED` 相同。区别是：

| 响应 | 场景 |
| --- | --- |
| `STARTED` | `START` 成功后的初始观测 |
| `OBS` | 每次 `STEP` 后的普通观测 |

### 4.3 FINISHED

格式：

```text
FINISHED reason=<reason> reached_goal=<0|1> elapsed_time_s=<number>
```

示例：

```text
FINISHED reason=goal reached_goal=1 elapsed_time_s=12.34
```

字段说明：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `reason` | string token | Mudri 返回的结束原因。空字符串会输出为 `none`；空白字符会被替换为 `_`。 |
| `reached_goal` | `0` 或 `1` | 是否到达目标。`1` 表示 true，`0` 表示 false。 |
| `elapsed_time_s` | number | 本次会话耗时，单位秒。来自 Mudri summary 的 `elapsed_time_s`。 |

### 4.4 ERR

格式：

```text
ERR message=<message>
```

示例：

```text
ERR message=missing_option:_sequence_id
```

字段说明：

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `message` | string token | 错误信息。空字符串会输出为 `none`；空白字符会被替换为 `_`。 |

常见错误示例：

| 输入 | 返回示例 | 原因 |
| --- | --- | --- |
| 空行 | 无响应 | 空行会被忽略，不计为命令。 |
| `ABC` | `ERR message=unknown_command:_ABC` | 未知命令。 |
| `START 120` | `ERR message=expected_key=value_token:_120` | `START` 参数必须是 `key=value`。 |
| `STEP 1 7.0` | `ERR message=missing_option:_sequence_id` | 不是完整的 3 个位置参数，会转入 key=value 解析，然后缺少字段。 |
| `STEP seq=a left=7 right=7` | `ERR message=sequence_id_must_be_an_integer` | `sequence_id` 必须是整数。 |
| `STEP seq=1 left=x right=7` | `ERR message=rear_left_target_speed_must_be_a_float` | 左轮速度必须是浮点数。 |

## 5. dig 字段生成规则

`dig` 来自 Mudri observation 中的线传感器数据，当前优先级如下：

1. 如果 `observation.line_sensor.digital` 是数组，直接使用该数组。
2. 否则读取 `observation.line_sensor_darkness`。
3. 如果没有 `line_sensor_darkness`，再读取 `observation.line_sensor.darkness`。
4. 如果仍没有数组，响应会变成 `ERR`。

当从 darkness 数组转换为 digital 时，使用桥接配置中的阈值：

```text
lineSensorDarknessThreshold = ARTEMIS_LINE_SENSOR_DARKNESS_THRESHOLD 或界面配置，默认 0.55
```

转换规则：

```text
darkness[i] >= threshold => 1
darkness[i] < threshold  => 0
```

布尔值和数字值转换为 bit 的规则：

| 原始值 | 输出 bit |
| --- | --- |
| `true` | `1` |
| `false` | `0` |
| `null` / `undefined` | `0` |
| 数字 `0` | `0` |
| 非零数字 | `1` |

示例：

```json
{
  "line_sensor": {
    "digital": [0, 0, 1, 1, 1, 0, 0]
  }
}
```

输出：

```text
dig=0011100
```

示例：从 darkness 生成 digital，阈值为 `0.55`

```json
{
  "line_sensor_darkness": [0.1, 0.2, 0.7, 0.9, 0.8, 0.3, 0.2]
}
```

输出：

```text
dig=0011100
```

## 6. 完整交互示例

### 6.1 正常启动、步进、停止

MCU 发送：

```text
START max_time_s=120 control_period_s=0.02 initial_progress_index=0 random_seed=1
```

桥接程序返回：

```text
STARTED seq=0 t=0 yaw=0 dig=0011100 enc_tl=0 enc_tr=0
```

MCU 发送：

```text
STEP seq=1 left=7.0 right=7.0
```

桥接程序返回：

```text
OBS seq=1 t=0.02 yaw=0.05 dig=0011100 enc_tl=12 enc_tr=12
```

MCU 发送：

```text
STEP seq=2 left=7.0 right=6.8
```

桥接程序返回：

```text
OBS seq=2 t=0.04 yaw=0.11 dig=0011000 enc_tl=24 enc_tr=23
```

MCU 发送：

```text
STOP reason=button_pressed
```

桥接程序返回：

```text
FINISHED reason=button_pressed reached_goal=0 elapsed_time_s=0.04
```

### 6.2 STEP 导致会话结束

MCU 发送：

```text
STEP 315 7.0 7.0
```

如果 Mudri 返回 `finished`，桥接程序不会返回 `OBS`，而是返回：

```text
FINISHED reason=goal reached_goal=1 elapsed_time_s=6.3
```

### 6.3 命令格式错误

MCU 发送：

```text
STEP seq=abc left=7.0 right=7.0
```

桥接程序返回：

```text
ERR message=sequence_id_must_be_an_integer
```

### 6.4 Mudri 请求超时

如果桥接程序已经打开串口，但向 Mudri 发送请求后超时，返回类似：

```text
ERR message=Mudri_request_timed_out_after_5000_ms:_tcp://127.0.0.1:5556
```

实际超时时间取决于桥接程序创建 Mudri 客户端时传入的 timeout。当前启动连接阶段使用 5000 ms。

## 7. 固件侧实现建议

1. 每次发送命令时以 `\n` 结尾。
2. 命令和字段名使用 ASCII。
3. `STEP` 建议优先使用位置参数格式，长度短且解析明确：

```text
STEP 1 7.0 7.0
```

4. 如果需要可读性，使用 key=value 格式：

```text
STEP seq=1 left=7.0 right=7.0
```

5. MCU 接收响应时，先按第一个空格前的响应类型分发：

```text
STARTED / OBS / FINISHED / ERR
```

6. 响应字段均为 `key=value`，建议固件侧按空格切分，再按第一个 `=` 切分 key 和 value。
7. `dig` 是连续 bit 字符串，不是数组；需要按字符逐位读取。
8. 收到 `ERR` 后，应记录 `message`，并根据业务决定重试、发送 `STOP`，或进入故障态。
9. 收到 `FINISHED` 后，应认为当前 Mudri 会话已经结束；下一轮控制需要重新发送 `START`。

## 8. 当前实现的协议摘要

```text
MCU -> Bridge:
  START [max_time_s=<float|null>] [control_period_s=<float|null>] [initial_progress_index=<int|null>] [random_seed=<int|null>]
  S     [max_time_s=<float|null>] [control_period_s=<float|null>] [initial_progress_index=<int|null>] [random_seed=<int|null>]

  STEP <sequence_id:int> <rear_left_target_speed:float> <rear_right_target_speed:float>
  STEP sequence_id=<int> rear_left_target_speed=<float> rear_right_target_speed=<float>
  STEP seq=<int> left=<float> right=<float>
  T    seq=<int> l=<float> r=<float>

  STOP [reason]
  STOP reason=<reason>
  X    [reason]

Bridge -> MCU:
  STARTED seq=<number> t=<number> yaw=<number> dig=<bits> enc_tl=<number> enc_tr=<number>
  OBS     seq=<number> t=<number> yaw=<number> dig=<bits> enc_tl=<number> enc_tr=<number>
  FINISHED reason=<reason> reached_goal=<0|1> elapsed_time_s=<number>
  ERR message=<message>
```
